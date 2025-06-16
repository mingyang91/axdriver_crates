//! Simple DWMAC Ethernet Driver Tutorial
//!
//! This is a simplified DWMAC driver designed for educational purposes.
//! It demonstrates the core concepts of ethernet driver development
//! without production-level complexity.

mod mdio;
mod regs;
use crate::dwmac::mdio::{Yt8531cPhy, YT8531C_BMSR, YT8531C_EXT_CHIP_CONFIG};
use crate::dwmac::regs::dma::{BUS_MODE, DESC_OWN, RDES3};
use crate::{EthernetAddress, NetBufPtr, NetDriverOps};
use axdriver_base::{BaseDriverOps, DevError, DevResult, DeviceType};
use core::ptr::{read_volatile, write_volatile, NonNull};
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use core::{u8, usize};

/// Hardware abstraction layer for DWMAC driver
pub trait DwmacHal: Send + Sync {
    /// Allocate DMA-coherent memory
    fn dma_alloc(size: usize) -> (PhysAddr, NonNull<u8>);

    /// Deallocate DMA-coherent memory
    unsafe fn dma_dealloc(paddr: PhysAddr, vaddr: NonNull<u8>, size: usize) -> i32;

    /// Convert physical to virtual address
    unsafe fn mmio_phys_to_virt(paddr: PhysAddr, size: usize) -> NonNull<u8>;

    /// Convert virtual to physical address
    unsafe fn mmio_virt_to_phys(vaddr: NonNull<u8>, size: usize) -> PhysAddr;

    /// Wait for a duration
    fn wait_until(duration: core::time::Duration) -> Result<(), &'static str>;

    /// Configure platform-specific settings (optional)
    fn configure_platform() -> Result<(), &'static str> {
        Ok(()) // Default: do nothing
    }
}

/// Physical address type
pub type PhysAddr = usize;

/// Buffer sizes
const TX_DESC_COUNT: usize = 8;
const RX_DESC_COUNT: usize = 8;
const MAX_FRAME_SIZE: usize = 1536;

#[repr(C, align(16))]
#[derive(Debug, Copy, Clone)]
struct DmaDescriptor {
    /// Buffer 1 address (low 32 bits) or additional info
    des0: u32,
    /// Buffer 2 address (low 32 bits) or additional info
    des1: u32,
    /// Buffer sizs, VLAN tag, control bits
    des2: u32,
    /// Control bits, packet length, status
    des3: u32,
}

impl DmaDescriptor {
    fn des0(&self) -> u32 {
        unsafe { read_volatile(&self.des0 as *const u32) }
    }

    fn set_des0(&mut self, des0: u32) {
        unsafe { write_volatile(&mut self.des0, des0) };
    }

    fn des1(&self) -> u32 {
        unsafe { read_volatile(&self.des1 as *const u32) }
    }

    fn set_des1(&mut self, des1: u32) {
        unsafe { write_volatile(&mut self.des1, des1) };
    }

    fn des2(&self) -> u32 {
        unsafe { read_volatile(&self.des2 as *const u32) }
    }

    fn set_des2(&mut self, des2: u32) {
        unsafe { write_volatile(&mut self.des2, des2) };
    }

    fn des3(&self) -> u32 {
        unsafe { read_volatile(&self.des3 as *const u32) }
    }

    fn set_des3(&mut self, des3: u32) {
        unsafe { write_volatile(&mut self.des3, des3) };
    }

    /// Set the descriptor as owned by DMA hardware
    pub fn set_own(&mut self) {
        unsafe {
            write_volatile(&mut self.des3, self.des3 | DESC_OWN);
        }
    }

    /// Clear the DMA ownership bit
    pub fn clear_own(&mut self) {
        unsafe {
            write_volatile(&mut self.des3, self.des3 & !DESC_OWN);
        }
    }

    /// Check if descriptor is owned by DMA hardware
    pub fn is_owned_by_dma(&self) -> bool {
        unsafe { (read_volatile(&self.des3 as *const u32) & DESC_OWN) != 0 }
    }
}

#[repr(C, align(16))]
#[derive(Debug, Copy, Clone)]
pub struct DmaExtendedDescriptor {
    basic: DmaDescriptor,
    // Extended fields for newer DWMAC versions
    // ext_status: u32,
    // reserved1: u32,
    // timestamp_low: u32,
    // timestamp_high: u32,
}

impl DmaExtendedDescriptor {
    const fn new() -> Self {
        Self {
            basic: DmaDescriptor {
                des0: 0,
                des1: 0,
                des2: 0,
                des3: 0,
            },
            // ext_status: 0,
            // reserved1: 0,
            // timestamp_low: 0,
            // timestamp_high: 0,
        }
    }
}

pub struct DescriptorRing<const N: usize, H: DwmacHal> {
    descriptors: [DmaExtendedDescriptor; N],
    buffers: [*mut u8; N],
    buffer_size: usize,
    head: AtomicUsize,
    tail: AtomicUsize,
    recycle_index: AtomicUsize,
    _phantom: core::marker::PhantomData<H>,
}

impl<const N: usize, H: DwmacHal> DescriptorRing<N, H> {
    fn new(buffer_size: usize) -> Self {
        Self {
            descriptors: [DmaExtendedDescriptor::new(); N],
            buffers: [core::ptr::null_mut(); N],
            buffer_size,
            head: AtomicUsize::new(0),
            tail: AtomicUsize::new(0),
            recycle_index: AtomicUsize::new(0),
            _phantom: core::marker::PhantomData,
        }
    }

    pub fn init_rx_ring(&mut self) -> Result<(), &'static str> {
        for (i, desc) in self.descriptors.iter_mut().enumerate() {
            let (buffer_phy, _) = H::dma_alloc(self.buffer_size);
            desc.basic.des0 = buffer_phy as u32;
            desc.basic.des1 = 0x0;
            desc.basic.des2 = self.buffer_size as u32;
            desc.basic.des3 = RDES3::BUFFER1_VALID_ADDR.bits() | RDES3::INT_ON_COMPLETION_EN.bits();
            desc.basic.set_own();
            self.buffers[i] = buffer_phy as *mut u8;
        }

        self.head.store(0, Ordering::Relaxed);
        self.tail.store(0, Ordering::Relaxed);
        self.recycle_index.store(0, Ordering::Relaxed);

        Ok(())
    }

    pub fn init_tx_ring(&mut self) -> Result<(), &'static str> {
        for desc in self.descriptors.iter_mut() {
            desc.basic.des0 = 0x0;
            desc.basic.des1 = 0x0;
            desc.basic.des2 = 0x0;
            desc.basic.des3 = 0x0;

            desc.basic.clear_own();
        }

        self.head.store(0, Ordering::Relaxed);
        self.tail.store(0, Ordering::Relaxed);
        for addr in self.buffers.iter_mut() {
            *addr = core::ptr::null_mut();
        }

        Ok(())
    }

    pub fn get_descriptor_address(&self, index: usize) -> u32 {
        &self.descriptors[index] as *const _ as u32
    }

    pub fn head(&self) -> usize {
        self.head.load(Ordering::Acquire) % N
    }

    pub fn tail(&self) -> usize {
        self.tail.load(Ordering::Acquire) % N
    }

    pub fn next_head(&self) -> usize {
        (self.head.load(Ordering::Acquire) + 1) % N
    }

    pub fn advance_head(&self) {
        self.head.fetch_add(1, Ordering::Release);
    }

    pub fn get_next_tx_descriptor(&self) -> Option<&DmaExtendedDescriptor> {
        let head = self.head();
        let tail = self.tail();
        let next = self.next_head();

        if next == tail {
            return None;
        }

        let current = &self.descriptors[head];
        if current.basic.is_owned_by_dma() {
            return None;
        }

        Some(&self.descriptors[head])
    }

    pub fn get_next_tx_descriptor_mut(&mut self) -> Option<&mut DmaExtendedDescriptor> {
        let head = self.head();
        let tail = self.tail();
        let next = self.next_head();

        if next == tail {
            return None;
        }

        let current = &self.descriptors[head];
        if current.basic.is_owned_by_dma() {
            return None;
        }

        Some(&mut self.descriptors[head])
    }

    pub fn get_completed_rx_descriptor(&mut self) -> Option<(usize, *mut u8)> {
        let tail = self.tail();
        let desc = &mut self.descriptors[tail];

        if desc.basic.is_owned_by_dma() {
            return None;
        }

        let result = if (desc.basic.des3() & RDES3::ERROR_SUMMARY.bits()) != 0 {
            log::error!("🔍 Error summary: {:#x}", desc.basic.des3());
            None
        } else {
            let packet_len = (desc.basic.des3() & 0x7fff) as usize;
            let buffer_ptr = self.buffers[tail];
            Some((packet_len, buffer_ptr))
        };

        desc.basic.clear_own();

        result
    }

    pub fn advance_rx_tail(&self) {
        self.tail.fetch_add(1, Ordering::Release);
    }

    pub fn reclaim_tx_descriptors(&mut self) -> usize {
        let mut reclaimed = 0;
        let tail = self.tail();
        let head = self.head();

        let mut current = tail;
        while current != head {
            let desc = &mut self.descriptors[current];
            if desc.basic.is_owned_by_dma() {
                break;
            }

            desc.basic.clear_own();
            desc.basic.des0 = 0x0;
            desc.basic.des1 = 0x0;
            desc.basic.des2 = 0x0;
            desc.basic.des3 = 0x0;

            self.buffers[current] = core::ptr::null_mut();

            current = (current + 1) % N;
            reclaimed += 1;
        }

        if reclaimed > 0 {
            self.tail.fetch_add(reclaimed, Ordering::Release);
        }

        reclaimed
    }

    pub fn recycle_rx_descriptor(&mut self, desc_index: usize) -> Result<(), &'static str> {
        let desc = &mut self.descriptors[desc_index];

        desc.basic.des3 = RDES3::BUFFER1_VALID_ADDR.bits() | RDES3::INT_ON_COMPLETION_EN.bits();
        desc.basic.set_own();

        Ok(())
    }

    pub fn is_full(&self) -> bool {
        self.next_head() == self.tail()
    }

    pub fn is_empty(&self) -> bool {
        self.head() == self.tail()
    }

    pub fn has_available_tx(&self) -> bool {
        !self.is_full() && !self.descriptors[self.head()].basic.is_owned_by_dma()
    }

    pub fn has_completed_rx(&self) -> bool {
        !self.descriptors[self.tail()].basic.is_owned_by_dma()
    }

    pub fn advance_recycle_index(&self) {
        self.recycle_index.fetch_add(1, Ordering::Release);
    }

    pub fn get_recycle_index(&self) -> usize {
        self.recycle_index.load(Ordering::Acquire) % N
    }
}

/// Simple DWMAC network interface
pub struct DwmacNic<H: DwmacHal> {
    base_addr: NonNull<u8>,
    mac_addr: [u8; 6],
    link_up: AtomicBool,

    tx_ring: DescriptorRing<TX_DESC_COUNT, H>,
    tx_dirty: AtomicUsize,
    rx_ring: DescriptorRing<RX_DESC_COUNT, H>,

    _phantom: core::marker::PhantomData<H>,
}

unsafe impl<H: DwmacHal> Send for DwmacNic<H> {}
unsafe impl<H: DwmacHal> Sync for DwmacNic<H> {}

impl<H: DwmacHal> DwmacNic<H> {
    /// Initialize the DWMAC device
    pub fn init(base_addr: NonNull<u8>, _size: usize) -> Result<Self, &'static str> {
        log::info!("🚀 Initializing DWMAC ethernet driver (tutorial version)");

        // Platform-specific setup
        H::configure_platform()?;

        let mut nic = Self {
            base_addr,
            mac_addr: [0x02, 0x03, 0x04, 0x05, 0x06, 0x07], // Default MAC
            link_up: AtomicBool::new(true),

            tx_ring: DescriptorRing::<TX_DESC_COUNT, H>::new(MAX_FRAME_SIZE),
            tx_dirty: AtomicUsize::new(0),
            rx_ring: DescriptorRing::<RX_DESC_COUNT, H>::new(MAX_FRAME_SIZE),

            _phantom: core::marker::PhantomData,
        };

        // Initialize hardware
        for i in 0..2 {
            let _ = nic.init_phy(i).inspect_err(|e| {
                log::error!("PHY{i} initialization failed: {:?}", e);
            });
        }

        nic.reset_dma()?;
        nic.setup_descriptor_rings()?;
        nic.start_dma()?;
        nic.enable_dma_interrupts()?;
        nic.setup_mac_address();
        nic.set_qos()?;
        nic.start_mac()?;

        let version = nic.read_reg(regs::mac::VERSION);
        log::info!("   🔍 MAC version: {:#x}", version);

        let intr_status = nic.read_reg(regs::dma::INTR_STATUS);
        log::info!("   🔍 DMA INTR_STATUS: {:#x}", intr_status);

        log::info!("✅ DWMAC initialization complete");
        Ok(nic)
    }

    /// Enable DMA interrupts
    fn enable_dma_interrupts(&self) -> Result<(), &'static str> {
        log::info!("🔧 Enabling DMA channel 0 interrupts...");
        log::debug!(
            "   🔍 Current: {:#x}",
            self.read_reg(regs::dma::CHAN_INTR_ENABLE)
        );
        self.write_reg(
            regs::dma::CHAN_INTR_ENABLE,
            regs::dma::InterruptMask::DEFAULT_MASK_4_10.bits(),
        );
        log::debug!(
            "   🔍 New: {:#x}",
            self.read_reg(regs::dma::CHAN_INTR_ENABLE)
        );

        log::info!("🔧 Enabling GMAC interrupts...");

        let status = self.read_reg(regs::mac::INTERRUPT_STATUS);
        log::info!("   🔍 Interrupt status: {:#x}", status);
        self.write_reg(regs::mac::INTERRUPT_STATUS, 0);

        if status != 0 {
            log::info!("🔧 Waiting for GMAC interrupt status to clear...");
            for _ in 0..100 {
                if self.read_reg(regs::mac::INTERRUPT_STATUS) != 0 {
                    break;
                }
                H::wait_until(core::time::Duration::from_millis(1))?;
            }

            if self.read_reg(regs::mac::INTERRUPT_STATUS) != 0 {
                log::error!("GMAC interrupt status not cleared");
            }
        }

        self.write_reg(
            regs::mac::INTERRUPT_ENABLE,
            regs::mac::INT_DEFAULT_ENABLE | regs::mac::MacInterruptEnable::RGMII.bits(),
        );
        let interrupt_enable = self.read_reg(regs::mac::INTERRUPT_ENABLE);
        log::debug!("   🔍 Interrupt enable: {:#x}", interrupt_enable);

        let status = self.read_reg(regs::mac::INTERRUPT_STATUS);
        log::info!("   🔍 Interrupt status: {:#x}", status);

        Ok(())
    }

    /// Reset the hardware
    fn reset_dma(&self) -> Result<(), &'static str> {
        log::info!("🔄 Resetting DMA Mode");

        log::info!("🔧 Disabling MAC");
        let config = self.read_reg(regs::mac::CONFIG);
        self.write_reg(
            regs::mac::CONFIG,
            config & !regs::mac::MacConfig::RE.bits() & !regs::mac::MacConfig::TE.bits(),
        );

        // Apply software reset
        let bus_mode = self.read_reg(regs::dma::BUS_MODE);
        log::debug!("🔍 BUS_MODE before: {:#x}", bus_mode);

        let sys_bus_mode = self.read_reg(regs::dma::SYS_BUS_MODE);
        log::debug!("🔍 SYS_BUS_MODE before: {:#x}", sys_bus_mode);

        self.write_reg(regs::dma::BUS_MODE, bus_mode | regs::dma::DMA_RESET);

        // Wait for reset to complete
        let mut timeout = 1000;
        while self.read_reg(regs::dma::BUS_MODE) & regs::dma::DMA_RESET != 0 {
            if timeout == 0 {
                return Err("Hardware reset timeout");
            }
            timeout -= 1;
            H::wait_until(core::time::Duration::from_millis(1))?;
        }

        let sys_bus_mode = self.read_reg(regs::dma::SYS_BUS_MODE);
        log::debug!("🔍 SYS_BUS_MODE before: {:#x}", sys_bus_mode);
        self.write_reg(
            regs::dma::SYS_BUS_MODE,
            sys_bus_mode
                | regs::dma::DmaSysBusMode::FB.bits()
                | regs::dma::DmaSysBusMode::MB.bits()
                | regs::dma::DmaSysBusMode::AAL.bits()
                | regs::dma::DmaSysBusMode::EAME.bits()
                | regs::dma::DmaSysBusMode::AXI_BLEN256.bits()
                | regs::dma::DmaSysBusMode::AXI_BLEN128.bits()
                | regs::dma::DmaSysBusMode::AXI_BLEN64.bits()
                | regs::dma::DmaSysBusMode::AXI_BLEN32.bits()
                | regs::dma::DmaSysBusMode::AXI_BLEN16.bits()
                | regs::dma::DmaSysBusMode::AXI_BLEN8.bits()
                | regs::dma::DmaSysBusMode::AXI_BLEN4.bits(),
        );
        log::debug!(
            "🔍 SYS_BUS_MODE after: {:#x}",
            self.read_reg(regs::dma::SYS_BUS_MODE)
        );

        let bus_mode = self.read_reg(regs::dma::BUS_MODE);
        self.write_reg(BUS_MODE, bus_mode | regs::dma::DMA_BUS_MODE_INTM_MODE1);

        log::info!(
            "✅ DMA Mode reset complete: {:#x}",
            self.read_reg(regs::dma::BUS_MODE)
        );
        Ok(())
    }

    /// Setup DMA descriptor rings
    fn setup_descriptor_rings(&mut self) -> Result<(), &'static str> {
        log::info!("🔧 Setting up descriptor rings");

        self.rx_ring.init_rx_ring()?;
        self.tx_ring.init_tx_ring()?;

        // Set RX ring base address
        self.write_reg(regs::dma::CHAN_RX_BASE_ADDR_HI, 0);
        self.write_reg(
            regs::dma::CHAN_RX_BASE_ADDR,
            self.rx_ring.get_descriptor_address(0),
        );
        log::debug!(
            "🔍 CHAN_RX_BASE_ADDR: {:#x}",
            self.read_reg(regs::dma::CHAN_RX_BASE_ADDR)
        );
        // Set RX ring length
        self.write_reg(regs::dma::CHAN_RX_RING_LEN, (RX_DESC_COUNT - 1) as u32);

        // Set RX ring end address
        self.write_reg(
            regs::dma::CHAN_RX_END_ADDR,
            self.rx_ring.get_descriptor_address(RX_DESC_COUNT - 1),
        );
        log::debug!(
            "🔍 CHAN_RX_END_ADDR: {:#x}",
            self.read_reg(regs::dma::CHAN_RX_END_ADDR)
        );

        // Set RX ring control
        let rx_ctrl = self.read_reg(regs::dma::CHAN_RX_CTRL);
        self.write_reg(
            regs::dma::CHAN_RX_CTRL,
            rx_ctrl | regs::dma::DMA_START_RX | regs::dma::DMA_RX_PBL_8,
        );
        log::debug!(
            "🔍 CHAN_RX_CTRL: {:#x}",
            self.read_reg(regs::dma::CHAN_RX_CTRL)
        );

        // Set TX ring base address
        self.write_reg(regs::dma::CHAN_TX_BASE_ADDR_HI, 0);
        self.write_reg(
            regs::dma::CHAN_TX_BASE_ADDR,
            self.tx_ring.get_descriptor_address(0),
        );
        log::debug!(
            "🔍 CHAN_TX_BASE_ADDR: {:#x}",
            self.read_reg(regs::dma::CHAN_TX_BASE_ADDR)
        );
        // Set TX ring length
        self.write_reg(regs::dma::CHAN_TX_RING_LEN, (TX_DESC_COUNT - 1) as u32);

        // Set TX ring end address
        self.write_reg(
            regs::dma::CHAN_TX_END_ADDR,
            self.tx_ring.get_descriptor_address(TX_DESC_COUNT - 1),
        );
        log::debug!(
            "🔍 CHAN_TX_END_ADDR: {:#x}",
            self.read_reg(regs::dma::CHAN_TX_END_ADDR)
        );

        // Set TX ring control
        let tx_ctrl = self.read_reg(regs::dma::CHAN_TX_CTRL);
        self.write_reg(
            regs::dma::CHAN_TX_CTRL,
            tx_ctrl | regs::dma::DMA_START_TX | regs::dma::DMA_TX_PBL_8,
        );
        log::debug!(
            "🔍 CHAN_TX_CTRL: {:#x}",
            self.read_reg(regs::dma::CHAN_TX_CTRL)
        );

        log::info!("✅ Descriptor rings ready");
        Ok(())
    }

    /// Configure MAC settings
    fn start_mac(&self) -> Result<(), &'static str> {
        log::info!("🔧 Configuring MAC");

        self.write_reg(regs::mac::CONFIG, regs::mac::CONFIG_DEFAULT);
        log::info!("🔧 MAC enabled");

        let version = self.read_reg(regs::mac::VERSION);
        log::info!("   🔍 MAC version: {:#x}", version);

        self.write_reg(
            regs::mac::FRAME_FILTER,
            regs::mac::PacketFilter::PR.bits() | regs::mac::PacketFilter::PM.bits(),
        );
        log::debug!(
            "🔍 FRAME_FILTER: {:#x}",
            self.read_reg(regs::mac::FRAME_FILTER)
        );

        Ok(())
    }

    /// Setup MAC address
    fn setup_mac_address(&self) {
        log::info!("🔧 Setting MAC address");

        let mac_high = ((self.mac_addr[5] as u32) << 8) | (self.mac_addr[4] as u32);
        let mac_low = ((self.mac_addr[3] as u32) << 24)
            | ((self.mac_addr[2] as u32) << 16)
            | ((self.mac_addr[1] as u32) << 8)
            | (self.mac_addr[0] as u32);

        self.write_reg(regs::mac::ADDRESS0_HIGH, mac_high | (1 << 31));
        self.write_reg(regs::mac::ADDRESS0_LOW, mac_low);
    }

    /// Start DMA operations
    fn start_dma(&self) -> Result<(), &'static str> {
        log::info!("🚀 Starting DMA");
        let chan_ctrl = self.read_reg(regs::dma::CHAN_BASE_ADDR);
        self.write_reg(regs::dma::CHAN_BASE_ADDR, chan_ctrl | 0x1);
        log::debug!(
            "🔍 CHAN_CTRL: {:#x}",
            self.read_reg(regs::dma::CHAN_BASE_ADDR)
        );

        Ok(())
    }

    /// Initialize PHY (simplified)
    fn init_phy(&self, phy_addr: u8) -> Result<(), &'static str> {
        log::info!("🔧 Initializing PHY (basic)");

        let phy = Yt8531cPhy::<H>::new(self.base_addr.as_ptr() as usize, phy_addr);
        let phy_id = phy.get_phy_id().map_err(|e| {
            log::warn!("⚠️  PHY not responding: {:?}", e);
            "PHY not responding"
        })?;

        log::info!("🔍 PHY ID: {:#x}", phy_id);
        if phy_id != 0x4f51e91b {
            log::error!("PHY ID mismatch: {:#x}", phy_id);
            return Err("PHY ID mismatch");
        }

        let bmsr = phy.read_reg(YT8531C_BMSR).map_err(|e| {
            log::warn!("⚠️  PHY not responding: {:?}", e);
            "PHY not responding"
        })?;
        log::info!("🔍 PHY BMSR: {:#x}", bmsr);

        let config = phy.read_ext_reg(YT8531C_EXT_CHIP_CONFIG).map_err(|e| {
            log::warn!("⚠️  PHY not responding: {:?}", e);
            "PHY not responding"
        })?;
        log::info!("🔍 PHY EXT_CHIP_CONFIG: {:#x}", config);

        log::error!("Not implemented");
        Ok(())
    }

    fn set_qos(&self) -> Result<(), &'static str> {
        self.write_reg(
            regs::dma::GMAC_Q0_TX_FLOW_CTRL,
            regs::dma::GMAC_Q0_TX_FLOW_CTRL_TFE,
        );
        log::debug!(
            "🔧 QOS set to TFE {:x}",
            self.read_reg(regs::dma::GMAC_Q0_TX_FLOW_CTRL)
        );
        Ok(())
    }

    /// Read hardware register
    fn read_reg(&self, offset: usize) -> u32 {
        unsafe {
            let addr = self.base_addr.as_ptr().add(offset) as *const u32;
            core::ptr::read_volatile(addr)
        }
    }

    /// Write hardware register
    fn write_reg(&self, offset: usize, value: u32) {
        unsafe {
            let addr = self.base_addr.as_ptr().add(offset) as *mut u32;
            core::ptr::write_volatile(addr, value);
        }
    }

    /// 写入TX DMA轮询请求寄存器
    fn start_tx_dma(&self) {
        self.write_reg(regs::dma::TX_POLL_DEMAND, 1);
    }

    // fn update_link_status(&self) {
    //     let status = self.read_reg(regs::mac::INTERRUPT_STATUS);
    //     if status & regs::mac::MacInterruptStatus::LINK_UP.bits() != 0 {
    //         self.link_up.store(true, Ordering::Release);
    //     } else {
    //         self.link_up.store(false, Ordering::Release);
    //     }
    // }
}

// Implement network driver traits
impl<H: DwmacHal> BaseDriverOps for DwmacNic<H> {
    fn device_type(&self) -> DeviceType {
        DeviceType::Net
    }

    fn device_name(&self) -> &str {
        "dwmac-tutorial"
    }
}

impl<H: DwmacHal> NetDriverOps for DwmacNic<H> {
    fn mac_address(&self) -> EthernetAddress {
        EthernetAddress(self.mac_addr)
    }

    fn can_transmit(&self) -> bool {
        self.link_up.load(Ordering::Acquire) && self.tx_ring.has_available_tx()
    }

    fn can_receive(&self) -> bool {
        self.link_up.load(Ordering::Acquire) && self.rx_ring.has_completed_rx()
    }

    fn rx_queue_size(&self) -> usize {
        RX_DESC_COUNT
    }

    fn tx_queue_size(&self) -> usize {
        TX_DESC_COUNT
    }

    fn transmit(&mut self, tx_buf: NetBufPtr) -> DevResult {
        if !self.can_transmit() {
            return Err(DevError::Again);
        }

        let desc = self
            .tx_ring
            .get_next_tx_descriptor_mut()
            .ok_or(DevError::NoMemory)?;

        desc.basic.set_des0(tx_buf.raw_ptr::<u8>() as u32);
        desc.basic.set_des1(0);
        desc.basic.set_des2(tx_buf.packet_len() as u32);
        desc.basic.set_des3(
            regs::dma::TDES3::FIRST_DESCRIPTOR.bits() | regs::dma::TDES3::LAST_DESCRIPTOR.bits(),
        );
        desc.basic.set_own();

        self.tx_ring.buffers[self.tx_ring.head()] = tx_buf.raw_ptr() as *mut u8;

        self.tx_ring.advance_head();
        self.tx_dirty.fetch_add(1, Ordering::AcqRel);

        self.start_tx_dma();

        log::trace!("Packet transmitted, TX index: {}", self.tx_ring.head());

        Ok(())
    }

    fn receive(&mut self) -> DevResult<NetBufPtr> {
        log::trace!("receive");
        if !self.can_receive() {
            return Err(DevError::Again);
        }

        let (packet_len, buffer_ptr) = self
            .rx_ring
            .get_completed_rx_descriptor()
            .ok_or(DevError::Again)?;

        let net_buf = NetBufPtr::new(
            NonNull::new(buffer_ptr).unwrap(),
            NonNull::new(buffer_ptr).unwrap(),
            packet_len,
        );

        self.rx_ring.advance_rx_tail();

        log::trace!(
            "Packet received, length: {}, RX index: {}",
            packet_len,
            self.rx_ring.tail()
        );

        Ok(net_buf)
    }

    fn recycle_rx_buffer(&mut self, rx_buf: NetBufPtr) -> DevResult {
        let recycle_index = self.rx_ring.get_recycle_index();
        let desc = &mut self.rx_ring.descriptors[recycle_index];

        desc.basic.set_des0(rx_buf.raw_ptr::<u8>() as u32);
        desc.basic.set_des1(0);
        desc.basic.set_des2(self.rx_ring.buffer_size as u32);
        desc.basic
            .set_des3(RDES3::BUFFER1_VALID_ADDR.bits() | RDES3::INT_ON_COMPLETION_EN.bits());
        desc.basic.set_own();

        self.rx_ring.buffers[recycle_index] = rx_buf.raw_ptr() as *mut u8;
        self.rx_ring.advance_recycle_index();
        log::trace!("RX buffer recycled, RX index: {}", recycle_index);

        Ok(())
    }

    fn recycle_tx_buffers(&mut self) -> DevResult {
        let reclaimed = self.tx_ring.reclaim_tx_descriptors();
        self.tx_dirty.fetch_sub(reclaimed, Ordering::Release);
        Ok(())
    }

    fn alloc_tx_buffer(&mut self, size: usize) -> DevResult<NetBufPtr> {
        if size > MAX_FRAME_SIZE {
            return Err(DevError::InvalidParam);
        }

        let (buf_phys, buf_virt) = H::dma_alloc(size);
        if buf_phys == 0 {
            return Err(DevError::NoMemory);
        }

        Ok(NetBufPtr::new(buf_virt, buf_virt, size))
    }
}
