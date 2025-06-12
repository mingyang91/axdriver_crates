//! Simple DWMAC Ethernet Driver Tutorial
//!
//! This is a simplified DWMAC driver designed for educational purposes.
//! It demonstrates the core concepts of ethernet driver development
//! without production-level complexity.

mod mdio;
mod regs;
use crate::dwmac::mdio::{Yt8531cPhy, YT8531C_BMSR, YT8531C_EXT_CHIP_CONFIG, YT8531C_PHYID1};
use crate::dwmac::regs::dma::{BUS_MODE, DMA_RESET};
use crate::dwmac::regs::mac::{GMII_ADDRESS, YTPHY_PAGE_DATA, YTPHY_PAGE_SELECT};
use crate::{EthernetAddress, NetBufPtr, NetDriverOps};
use axdriver_base::{BaseDriverOps, DevError, DevResult, DeviceType};
use core::ptr::NonNull;
use core::sync::atomic::{AtomicUsize, Ordering};
use core::{u16, u8};

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

/// Simple DMA descriptor
#[repr(C, align(16))]
#[derive(Debug)]
struct DmaDescriptor {
    status: u32,
    control: u32,
    buffer1: u32,
    buffer2: u32,
    // Extended fields for newer DWMAC versions
    ext_status: u32,
    reserved1: u32,
    timestamp_low: u32,
    timestamp_high: u32,
}

impl DmaDescriptor {
    const fn new() -> Self {
        Self {
            status: 0,
            control: 0,
            buffer1: 0,
            buffer2: 0,
            ext_status: 0,
            reserved1: 0,
            timestamp_low: 0,
            timestamp_high: 0,
        }
    }
}

/// Simple DWMAC network interface
pub struct DwmacNic<H: DwmacHal> {
    base_addr: NonNull<u8>,
    mac_addr: [u8; 6],

    // TX ring
    tx_descriptors: NonNull<DmaDescriptor>,
    tx_descriptors_phys: PhysAddr,
    tx_buffers: [Option<NetBufPtr>; TX_DESC_COUNT],
    tx_next: AtomicUsize,
    tx_clean: AtomicUsize,

    // RX ring
    rx_descriptors: NonNull<DmaDescriptor>,
    rx_descriptors_phys: PhysAddr,
    rx_buffers: [Option<NetBufPtr>; RX_DESC_COUNT],
    rx_next: AtomicUsize,

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

        // Allocate descriptor rings
        let tx_desc_size = core::mem::size_of::<DmaDescriptor>() * TX_DESC_COUNT;
        let rx_desc_size = core::mem::size_of::<DmaDescriptor>() * RX_DESC_COUNT;

        let (tx_descriptors_phys, tx_descriptors_virt) = H::dma_alloc(tx_desc_size);
        let (rx_descriptors_phys, rx_descriptors_virt) = H::dma_alloc(rx_desc_size);

        if tx_descriptors_phys == 0 || rx_descriptors_phys == 0 {
            return Err("Failed to allocate DMA memory");
        }

        let tx_descriptors = NonNull::new(tx_descriptors_virt.as_ptr() as *mut DmaDescriptor)
            .ok_or("Invalid TX descriptor pointer")?;
        let rx_descriptors = NonNull::new(rx_descriptors_virt.as_ptr() as *mut DmaDescriptor)
            .ok_or("Invalid RX descriptor pointer")?;

        let mut nic = Self {
            base_addr,
            mac_addr: [0x02, 0x03, 0x04, 0x05, 0x06, 0x07], // Default MAC

            tx_descriptors,
            tx_descriptors_phys,
            tx_buffers: [const { None }; TX_DESC_COUNT],
            tx_next: AtomicUsize::new(0),
            tx_clean: AtomicUsize::new(0),

            rx_descriptors,
            rx_descriptors_phys,
            rx_buffers: [const { None }; RX_DESC_COUNT],
            rx_next: AtomicUsize::new(0),

            _phantom: core::marker::PhantomData,
        };

        // Initialize hardware
        for i in 0..2 {
            let _ = nic.init_phy(i).inspect_err(|e| {
                log::error!("PHY{i} initialization failed: {:?}", e);
            });
        }

        nic.reset_dma()?;
        nic.configure_mac()?;
        nic.setup_descriptor_rings()?;
        nic.configure_dma()?;
        nic.start_dma()?;
        // nic.setup_mac_address();
        nic.set_qos()?;

        let version = nic.read_reg(regs::mac::VERSION);
        log::info!("   🔍 MAC version: {:#x}", version);

        log::info!("✅ DWMAC initialization complete");
        Ok(nic)
    }

    /// Enable DMA interrupts
    fn enable_dma_interrupts(&self) -> Result<(), &'static str> {
        log::info!("🔧 Enabling DMA interrupts...");
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

        let readback = self.read_reg(regs::dma::CHAN_INTR_ENABLE);
        log::info!("   🔍 Readback: {:#x}", readback);
        log::info!("🔧 Enabling GMAC channel 0 interrupts...");

        let status = self.read_reg(regs::mac::INTERRUPT_STATUS);
        log::info!("   🔍 Interrupt status: {:#x}", status);
        self.write_reg(regs::mac::INTERRUPT_STATUS, 0);

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

        // Initialize TX descriptors
        unsafe {
            let tx_slice =
                core::slice::from_raw_parts_mut(self.tx_descriptors.as_ptr(), TX_DESC_COUNT);

            for (i, desc) in tx_slice.iter_mut().enumerate() {
                *desc = DmaDescriptor::new();
                // Link descriptors in a ring
                if i == TX_DESC_COUNT - 1 {
                    desc.buffer2 = self.tx_descriptors_phys as u32; // Point to first
                } else {
                    desc.buffer2 = (self.tx_descriptors_phys
                        + (i + 1) * core::mem::size_of::<DmaDescriptor>())
                        as u32;
                }
            }
        }

        // Initialize RX descriptors and buffers
        unsafe {
            let rx_slice =
                core::slice::from_raw_parts_mut(self.rx_descriptors.as_ptr(), RX_DESC_COUNT);

            for (i, desc) in rx_slice.iter_mut().enumerate() {
                *desc = DmaDescriptor::new();

                // Allocate RX buffer
                let (buf_phys, buf_virt) = H::dma_alloc(MAX_FRAME_SIZE);
                if buf_phys == 0 {
                    return Err("Failed to allocate RX buffer");
                }

                let rx_buf = NetBufPtr::new(buf_virt, buf_virt, MAX_FRAME_SIZE);
                self.rx_buffers[i] = Some(rx_buf);

                desc.buffer1 = buf_phys as u32;
                desc.control = MAX_FRAME_SIZE as u32;
                desc.status = regs::dma::DESC_OWN; // Give to hardware

                // Link descriptors
                if i == RX_DESC_COUNT - 1 {
                    desc.buffer2 = self.rx_descriptors_phys as u32;
                } else {
                    desc.buffer2 = (self.rx_descriptors_phys
                        + (i + 1) * core::mem::size_of::<DmaDescriptor>())
                        as u32;
                }
            }
        }

        // Tell hardware about descriptor rings
        self.write_reg(
            regs::dma::TX_DESCRIPTOR_LIST,
            self.tx_descriptors_phys as u32,
        );
        self.write_reg(
            regs::dma::RX_DESCRIPTOR_LIST,
            self.rx_descriptors_phys as u32,
        );

        log::info!("✅ Descriptor rings ready");
        Ok(())
    }

    /// Configure MAC settings
    fn configure_mac(&self) -> Result<(), &'static str> {
        log::info!("🔧 Configuring MAC");

        self.write_reg(regs::mac::CONFIG, regs::mac::CONFIG_DEFAULT);

        let version = self.read_reg(regs::mac::VERSION);
        log::info!("   🔍 MAC version: {:#x}", version);

        self.setup_mac_address();
        let mac_addr: u64 = (self.read_reg(regs::mac::ADDRESS0_HIGH) as u64) << 32
            | (self.read_reg(regs::mac::ADDRESS0_LOW) as u64);
        log::info!("   🔍 MAC address: {:#x}", mac_addr);

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

    /// Configure DMA
    fn configure_dma(&self) -> Result<(), &'static str> {
        log::info!("🔧 Configuring DMA");

        // Basic DMA configuration
        let bus_mode = self.read_reg(regs::dma::BUS_MODE);
        self.write_reg(regs::dma::BUS_MODE, bus_mode | regs::dma::DMA_RESET);

        self.write_reg(regs::dma::BUS_MODE, DMA_RESET);

        self.write_reg(regs::dma::BUS_MODE, 0x0f0f_08f1);

        self.write_reg(regs::dma::CHAN_BASE_ADDR, 0x0000_0000);

        self.write_reg(regs::dma::CHAN_BASE_ADDR, 0x0010_0000);

        self.write_reg(
            regs::dma::CHAN_RX_BASE_ADDR,
            self.rx_descriptors_phys as u32,
        );

        self.write_reg(
            regs::dma::CHAN_RX_END_ADDR,
            (self.rx_descriptors_phys + core::mem::size_of::<DmaDescriptor>() * RX_DESC_COUNT)
                as u32,
        );

        self.write_reg(regs::dma::CHAN_TX_CTRL, 0x0010_0010);

        self.write_reg(
            regs::dma::CHAN_TX_BASE_ADDR,
            self.tx_descriptors_phys as u32,
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

        self.write_reg(regs::mac::CONFIG, regs::mac::CONFIG_2);

        self.write_reg(regs::dma::RX_POLL_DEMAND, 0x2);

        self.write_reg(regs::dma::MTL_CHAN_RX_OP_MODE, 0x700000);

        self.write_reg(regs::dma::CHAN_BASE_ADDR, 0x70018);

        self.write_reg(regs::dma::CHAN_TX_RING_LEN, 64);

        self.write_reg(regs::dma::CHAN_RX_RING_LEN, 64);

        let rx_ctrl = self.read_reg(regs::dma::CHAN_RX_CTRL);
        self.write_reg(regs::dma::CHAN_RX_CTRL, rx_ctrl | 0x1);

        let config = self.read_reg(regs::mac::CONFIG);
        self.write_reg(regs::mac::CONFIG, config | regs::mac::MacConfig::RE.bits());

        let tx_ctrl = self.read_reg(regs::dma::CHAN_TX_CTRL);
        self.write_reg(regs::dma::CHAN_TX_CTRL, tx_ctrl | 0x1);

        let config = self.read_reg(regs::mac::CONFIG);
        self.write_reg(regs::mac::CONFIG, config | regs::mac::MacConfig::TE.bits());

        unsafe { self.enable_dma_interrupts()? };

        self.write_reg(regs::mac::CONFIG, 0x8072203);
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

    /// Reclaim completed TX buffers
    fn reclaim_tx_buffers(&mut self) {
        let mut clean_idx = self.tx_clean.load(Ordering::Acquire);
        let next_idx = self.tx_next.load(Ordering::Acquire);

        while clean_idx != next_idx {
            unsafe {
                let desc = &mut *self.tx_descriptors.as_ptr().add(clean_idx);

                // Check if owned by hardware
                if desc.status & regs::dma::DESC_OWN != 0 {
                    break;
                }

                // Free the buffer
                if let Some(buf) = self.tx_buffers[clean_idx].take() {
                    drop(buf);
                }

                clean_idx = (clean_idx + 1) % TX_DESC_COUNT;
            }
        }

        self.tx_clean.store(clean_idx, Ordering::Release);
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
        let next_idx = self.tx_next.load(Ordering::Acquire);
        let clean_idx = self.tx_clean.load(Ordering::Acquire);
        let next_next = (next_idx + 1) % TX_DESC_COUNT;
        next_next != clean_idx
    }

    fn can_receive(&self) -> bool {
        let rx_idx = self.rx_next.load(Ordering::Acquire);
        unsafe {
            let desc = &*self.rx_descriptors.as_ptr().add(rx_idx);
            desc.status & regs::dma::DESC_OWN == 0
        }
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

        let tx_idx = self.tx_next.load(Ordering::Acquire);

        unsafe {
            let desc = &mut *self.tx_descriptors.as_ptr().add(tx_idx);

            // Setup buffer
            let buf_phys =
                H::mmio_virt_to_phys(NonNull::new(tx_buf.raw_ptr()).unwrap(), tx_buf.packet_len());

            desc.buffer1 = buf_phys as u32;
            desc.control = tx_buf.packet_len() as u32;
            desc.status = regs::dma::DESC_OWN | regs::dma::DESC_FIRST | regs::dma::DESC_LAST;

            // Store buffer
            self.tx_buffers[tx_idx] = Some(tx_buf);

            // Advance ring
            let next_tx = (tx_idx + 1) % TX_DESC_COUNT;
            self.tx_next.store(next_tx, Ordering::Release);
        }

        // Trigger transmission
        self.write_reg(regs::dma::TX_POLL_DEMAND, 1);

        Ok(())
    }

    fn receive(&mut self) -> DevResult<NetBufPtr> {
        if !self.can_receive() {
            return Err(DevError::Again);
        }

        let rx_idx = self.rx_next.load(Ordering::Acquire);

        unsafe {
            let desc = &*self.rx_descriptors.as_ptr().add(rx_idx);

            // Check for errors (simplified)
            if desc.status & (1 << 15) != 0 {
                log::warn!("RX error: {:#x}", desc.status);
                return Err(DevError::Again);
            }

            // Get frame length
            let frame_len = ((desc.status >> 16) & 0x3FFF) as usize;
            if frame_len < 4 {
                return Err(DevError::Again);
            }

            let packet_len = frame_len - 4; // Remove CRC

            // Take the buffer
            if let Some(rx_buf) = self.rx_buffers[rx_idx].take() {
                let new_rx_buf = NetBufPtr::new(
                    NonNull::new(rx_buf.raw_ptr()).unwrap(),
                    NonNull::new(rx_buf.raw_ptr()).unwrap(),
                    packet_len,
                );

                log::trace!("Received {} bytes", packet_len);
                Ok(new_rx_buf)
            } else {
                Err(DevError::BadState)
            }
        }
    }

    fn recycle_rx_buffer(&mut self, rx_buf: NetBufPtr) -> DevResult {
        let rx_idx = self.rx_next.load(Ordering::Acquire);

        unsafe {
            let desc = &mut *self.rx_descriptors.as_ptr().add(rx_idx);

            // Reset buffer
            let buf_phys =
                H::mmio_virt_to_phys(NonNull::new(rx_buf.raw_ptr()).unwrap(), MAX_FRAME_SIZE);

            desc.buffer1 = buf_phys as u32;
            desc.control = MAX_FRAME_SIZE as u32;
            desc.status = regs::dma::DESC_OWN;

            // Store buffer
            self.rx_buffers[rx_idx] = Some(rx_buf);

            // Advance ring
            let next_rx = (rx_idx + 1) % RX_DESC_COUNT;
            self.rx_next.store(next_rx, Ordering::Release);
        }

        // Resume RX if needed
        self.write_reg(regs::dma::RX_POLL_DEMAND, 1);

        Ok(())
    }

    fn recycle_tx_buffers(&mut self) -> DevResult {
        self.reclaim_tx_buffers();
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
