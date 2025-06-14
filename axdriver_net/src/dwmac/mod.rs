//! Simple DWMAC Ethernet Driver Tutorial
//!
//! This is a simplified DWMAC driver designed for educational purposes.
//! It demonstrates the core concepts of ethernet driver development
//! without production-level complexity.

mod mdio;
mod regs;
use crate::dwmac::mdio::{Yt8531cPhy, YT8531C_BMSR, YT8531C_EXT_CHIP_CONFIG, YT8531C_PHYID1};
use crate::dwmac::regs::dma::{
    BUS_MODE, CHAN_RX_END_ADDR, DESC_OWN, DMA_RESET, RX_ERROR_SUMMARY, RX_FRAME_LENGTH_MASK,
    RX_FRAME_LENGTH_SHIFT, TX_ERROR_SUMMARY, TX_FIRST_SEGMENT, TX_INTERRUPT_COMPLETE,
    TX_LAST_SEGMENT,
};
use crate::dwmac::regs::mac::{GMII_ADDRESS, YTPHY_PAGE_DATA, YTPHY_PAGE_SELECT};
use crate::{EthernetAddress, NetBufPtr, NetDriverOps};
use axdriver_base::{BaseDriverOps, DevError, DevResult, DeviceType};
use core::ptr::{read_volatile, write_volatile, NonNull};
use core::sync::atomic::{fence, AtomicBool, AtomicUsize, Ordering};
use core::{u16, u8, usize};

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
const TX_DESC_COUNT: usize = 64;
const RX_DESC_COUNT: usize = 64;
const MAX_FRAME_SIZE: usize = 1536;

#[repr(C, align(16))]
#[derive(Debug, Copy, Clone)]
struct DmaDescriptor {
    status: u32,
    control: u32,
    buffer1: u32,
    buffer2: u32,
}

impl DmaDescriptor {
    fn status(&self) -> u32 {
        unsafe { read_volatile(&self.status as *const u32) }
    }

    fn set_status(&mut self, status: u32) {
        unsafe { write_volatile(&mut self.status, status) };
    }

    fn control(&self) -> u32 {
        unsafe { read_volatile(&self.control as *const u32) }
    }

    fn set_control(&mut self, control: u32) {
        unsafe { write_volatile(&mut self.control, control) };
    }

    fn buffer1(&self) -> u32 {
        unsafe { read_volatile(&self.buffer1 as *const u32) }
    }

    fn set_buffer1(&mut self, buffer1: u32) {
        unsafe { write_volatile(&mut self.buffer1, buffer1) };
    }

    fn buffer2(&self) -> u32 {
        unsafe { read_volatile(&self.buffer2 as *const u32) }
    }

    fn set_buffer2(&mut self, buffer2: u32) {
        unsafe { write_volatile(&mut self.buffer2, buffer2) };
    }
}

#[repr(C, align(16))]
#[derive(Debug, Copy, Clone)]
struct DmaExtendedDescriptor {
    basic: DmaDescriptor,
    // Extended fields for newer DWMAC versions
    ext_status: u32,
    reserved1: u32,
    timestamp_low: u32,
    timestamp_high: u32,
}

impl DmaExtendedDescriptor {
    const fn new() -> Self {
        Self {
            basic: DmaDescriptor {
                status: 0,
                control: 0,
                buffer1: 0,
                buffer2: 0,
            },
            ext_status: 0,
            reserved1: 0,
            timestamp_low: 0,
            timestamp_high: 0,
        }
    }
}

pub struct DescriptorRing<const N: usize, H: DwmacHal> {
    descriptors: [DmaExtendedDescriptor; N],
    buffers: [*mut u8; N],
    current_index: AtomicUsize,
    buffer_size: usize,
    _phantom: core::marker::PhantomData<H>,
}

impl<const N: usize, H: DwmacHal> DescriptorRing<N, H> {
    fn new(buffer_size: usize) -> Self {
        Self {
            descriptors: [DmaExtendedDescriptor::new(); N],
            buffers: [core::ptr::null_mut(); N],
            current_index: AtomicUsize::new(0),
            buffer_size,
            _phantom: core::marker::PhantomData,
        }
    }

    pub fn init_rx_ring(&mut self) -> Result<(), &'static str> {
        for i in 0..N {
            let (buffer_phy, _) = H::dma_alloc(self.buffer_size);
            self.buffers[i] = buffer_phy as *mut u8;

            self.descriptors[i].basic.status = DESC_OWN; // OWN bit
            self.descriptors[i].basic.control = self.buffer_size as u32 & 0x1fff; // Buffer size
            self.descriptors[i].basic.buffer1 = buffer_phy as u32;

            if i == N - 1 {
                self.descriptors[i].basic.buffer2 = &self.descriptors[0] as *const _ as u32;
            } else {
                self.descriptors[i].basic.buffer2 = &self.descriptors[i + 1] as *const _ as u32;
            }
        }

        Ok(())
    }

    pub fn init_tx_ring(&mut self) -> Result<(), &'static str> {
        for i in 0..N {
            self.descriptors[i].basic.status = 0x0; // OWN bit
            self.descriptors[i].basic.control = 0x0;
            self.descriptors[i].basic.buffer1 = 0x0;

            if i == N - 1 {
                self.descriptors[i].basic.buffer2 = &self.descriptors[0] as *const _ as u32;
            } else {
                self.descriptors[i].basic.buffer2 = &self.descriptors[i + 1] as *const _ as u32;
            }
        }

        Ok(())
    }

    pub fn get_descriptor_address(&self, index: usize) -> u32 {
        &self.descriptors[index] as *const _ as u32
    }
}

/// Simple DWMAC network interface
pub struct DwmacNic<H: DwmacHal> {
    base_addr: NonNull<u8>,
    mac_addr: [u8; 6],

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

        self.write_reg(regs::mac::INTERRUPT_ENABLE, regs::mac::INT_DEFAULT_ENABLE);
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
            self.tx_ring.get_descriptor_address(0), // No unsent descriptors
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

    fn tx_current(&self) -> usize {
        self.tx_ring.current_index.load(Ordering::Acquire) % TX_DESC_COUNT
    }

    fn rx_current(&self) -> usize {
        self.rx_ring.current_index.load(Ordering::Acquire) % RX_DESC_COUNT
    }

    fn tx_dirty(&self) -> usize {
        self.tx_dirty.load(Ordering::Acquire) % TX_DESC_COUNT
    }

    fn reset_rx_descriptor(&mut self, desc_index: usize) -> Result<(), DevError> {
        let desc = &mut self.rx_ring.descriptors[desc_index];
        let status = desc.basic.status();
        desc.basic.set_status(status | DESC_OWN);
        Ok(())
    }
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
        log::trace!("can_transmit");
        let current_desc = self.tx_ring.descriptors[self.tx_current()];
        let status = current_desc.basic.status();
        (status & regs::dma::DESC_OWN) == 0
    }

    fn can_receive(&self) -> bool {
        log::trace!("can_receive");
        let current_desc = self.rx_ring.descriptors[self.rx_current()];
        let status = current_desc.basic.status();
        (status & regs::dma::DESC_OWN) == 0 && (status & regs::dma::DESC_LAST) != 0
    }

    fn rx_queue_size(&self) -> usize {
        RX_DESC_COUNT
    }

    fn tx_queue_size(&self) -> usize {
        TX_DESC_COUNT
    }

    fn transmit(&mut self, tx_buf: NetBufPtr) -> DevResult {
        log::trace!("transmit");
        if !self.can_transmit() {
            return Err(DevError::Again);
        }

        let desc_index = self.tx_current();
        let desc = &mut self.tx_ring.descriptors[desc_index];

        desc.basic.set_buffer1(tx_buf.buf_ptr.as_ptr() as u32);
        desc.basic.set_control(tx_buf.len as u32);
        desc.basic
            .set_status(TX_FIRST_SEGMENT | TX_LAST_SEGMENT | TX_INTERRUPT_COMPLETE | DESC_OWN);

        self.tx_ring.current_index.fetch_add(1, Ordering::Release);

        let next_desc_index = self.tx_current();
        let next_desc_addr = self.tx_ring.get_descriptor_address(next_desc_index);

        self.write_reg(regs::dma::CHAN_TX_END_ADDR, next_desc_addr);

        log::trace!("Packet transmitted, TX index: {}", desc_index);

        Ok(())
    }

    fn receive(&mut self) -> DevResult<NetBufPtr> {
        log::trace!("receive");
        if !self.can_receive() {
            return Err(DevError::Again);
        }

        let desc_index = self.rx_current();
        let desc = &mut self.rx_ring.descriptors[desc_index];

        let status = desc.basic.status();

        if (status & RX_ERROR_SUMMARY) != 0 {
            self.reset_rx_descriptor(desc_index)?;
            log::trace!("RX descriptor reset, RX index: {}", desc_index);
            return Err(DevError::Again);
        }

        let frame_len = ((status & RX_FRAME_LENGTH_MASK) >> RX_FRAME_LENGTH_SHIFT) as usize;

        if frame_len < 64 || frame_len > 1518 {
            self.reset_rx_descriptor(desc_index)?;
            log::trace!(
                "RX frame length({}) invalid, RX index: {}",
                frame_len,
                desc_index
            );
            return Err(DevError::Again);
        }

        let buffer_addr = desc.basic.buffer1();

        let net_buf = NetBufPtr::new(
            NonNull::new(buffer_addr as *mut u8).unwrap(),
            NonNull::new(buffer_addr as *mut u8).unwrap(),
            frame_len - 4,
        );

        self.rx_ring.current_index.fetch_add(1, Ordering::Release);

        log::trace!(
            "Packet received, length: {}, RX index: {}",
            frame_len,
            desc_index
        );

        Ok(net_buf)
    }

    fn recycle_rx_buffer(&mut self, rx_buf: NetBufPtr) -> DevResult {
        let buffer_addr = rx_buf.buf_ptr.as_ptr() as u32;
        let mut desc_index = None;

        for i in 0..RX_DESC_COUNT {
            let desc = &self.rx_ring.descriptors[i];
            let desc_buffer = desc.basic.buffer1();

            if desc_buffer == buffer_addr {
                desc_index = Some(i);
                break;
            }
        }

        let Some(index) = desc_index else {
            log::trace!(
                "RX buffer not found, RX index: {}",
                desc_index.unwrap_or(usize::MAX)
            );
            return Err(DevError::BadState);
        };

        let desc = &mut self.rx_ring.descriptors[index];

        desc.basic.set_status(DESC_OWN);
        desc.basic.set_control(MAX_FRAME_SIZE as u32);

        self.write_reg(CHAN_RX_END_ADDR, self.rx_ring.get_descriptor_address(index));

        log::trace!("RX buffer recycled, RX index: {}", index);
        Ok(())
    }

    fn recycle_tx_buffers(&mut self) -> DevResult {
        let mut recycles = 0;

        while self.tx_dirty() != self.tx_current() {
            let dirty_index = self.tx_dirty();
            let dirty_desc = &mut self.tx_ring.descriptors[dirty_index];
            let status = dirty_desc.basic.status();

            if (status & DESC_OWN) != 0 {
                break;
            }

            if (status & TX_ERROR_SUMMARY) != 0 {
                log::trace!("TX error summary, TX index: {}", dirty_index);
            }

            dirty_desc.basic.set_status(0);
            dirty_desc.basic.set_control(0);
            dirty_desc.basic.set_buffer1(0);

            self.tx_dirty.fetch_add(1, Ordering::Release);
            recycles += 1;
        }

        if recycles > 0 {
            log::trace!("TX buffers recycled, recycles: {}", recycles);
        }

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
