//! Synopsys DesignWare MAC (DWMAC) Ethernet driver.
//!
//! This driver supports the Synopsys DesignWare Ethernet MAC v5.10a
//! as found in the StarFive VisionFive 2 board.

use crate::{EthernetAddress, NetBufPtr, NetDriverOps};
use axdriver_base::{BaseDriverOps, DevError, DevResult, DeviceType};
use core::ptr::NonNull;

/// Hardware abstraction layer for DWMAC driver.
pub trait DwmacHal: Send + Sync {
    /// Allocate DMA-coherent memory.
    fn dma_alloc(size: usize) -> (PhysAddr, NonNull<u8>);

    /// Deallocate DMA-coherent memory.
    unsafe fn dma_dealloc(paddr: PhysAddr, vaddr: NonNull<u8>, size: usize) -> i32;

    /// Convert physical address to virtual address for MMIO.
    unsafe fn mmio_phys_to_virt(paddr: PhysAddr, size: usize) -> NonNull<u8>;

    /// Convert virtual address to physical address for MMIO.
    unsafe fn mmio_virt_to_phys(vaddr: NonNull<u8>, size: usize) -> PhysAddr;

    /// Wait for a specified duration.
    fn wait_until(duration: core::time::Duration) -> Result<(), &'static str>;
}

/// Physical address type.
pub type PhysAddr = usize;

/// DWMAC register offsets (Synopsys DesignWare MAC v5.10a).
mod regs {
    // MAC registers
    pub const MAC_CONFIG: usize = 0x0000;
    pub const MAC_FRAME_FILTER: usize = 0x0004;
    pub const MAC_HASH_TABLE_REG0: usize = 0x0008;
    pub const MAC_HASH_TABLE_REG1: usize = 0x000C;
    pub const MAC_GMII_ADDRESS: usize = 0x0010;
    pub const MAC_GMII_DATA: usize = 0x0014;
    pub const MAC_FLOW_CONTROL: usize = 0x0018;
    pub const MAC_VLAN_TAG: usize = 0x001C;
    pub const MAC_VERSION: usize = 0x0020;
    pub const MAC_DEBUG: usize = 0x0024;
    pub const MAC_REMOTE_WAKE_FILTER: usize = 0x0028;
    pub const MAC_PMT_CONTROL_STATUS: usize = 0x002C;
    pub const MAC_ADDRESS0_HIGH: usize = 0x0040;
    pub const MAC_ADDRESS0_LOW: usize = 0x0044;

    // DMA registers
    pub const DMA_BUS_MODE: usize = 0x1000;
    pub const DMA_TX_POLL_DEMAND: usize = 0x1004;
    pub const DMA_RX_POLL_DEMAND: usize = 0x1008;
    pub const DMA_RX_DESCRIPTOR_LIST: usize = 0x100C;
    pub const DMA_TX_DESCRIPTOR_LIST: usize = 0x1010;
    pub const DMA_STATUS: usize = 0x1014;
    pub const DMA_OPERATION_MODE: usize = 0x1018;
    pub const DMA_INTERRUPT_ENABLE: usize = 0x101C;
    pub const DMA_MISSED_FRAME_COUNTER: usize = 0x1020;
    pub const DMA_RX_INTERRUPT_WATCHDOG: usize = 0x1024;
    pub const DMA_AXI_BUS_MODE: usize = 0x1028;
    pub const DMA_AXI_STATUS: usize = 0x102C;
    pub const DMA_CURRENT_HOST_TX_DESCRIPTOR: usize = 0x1048;
    pub const DMA_CURRENT_HOST_RX_DESCRIPTOR: usize = 0x104C;
    pub const DMA_CURRENT_HOST_TX_BUFFER: usize = 0x1050;
    pub const DMA_CURRENT_HOST_RX_BUFFER: usize = 0x1054;
}

/// MAC configuration register bits.
mod mac_config {
    pub const WATCHDOG_DISABLE: u32 = 1 << 23;
    pub const JABBER_DISABLE: u32 = 1 << 22;
    pub const FRAME_BURST_ENABLE: u32 = 1 << 21;
    pub const JUMBO_FRAME_ENABLE: u32 = 1 << 20;
    pub const INTER_FRAME_GAP_96: u32 = 0 << 17;
    pub const INTER_FRAME_GAP_88: u32 = 1 << 17;
    pub const INTER_FRAME_GAP_80: u32 = 2 << 17;
    pub const INTER_FRAME_GAP_72: u32 = 3 << 17;
    pub const INTER_FRAME_GAP_64: u32 = 4 << 17;
    pub const INTER_FRAME_GAP_56: u32 = 5 << 17;
    pub const INTER_FRAME_GAP_48: u32 = 6 << 17;
    pub const INTER_FRAME_GAP_40: u32 = 7 << 17;
    pub const DISABLE_CRS: u32 = 1 << 16;
    pub const MII_GMII_SELECT: u32 = 1 << 15;
    pub const FAST_ETHERNET_SPEED: u32 = 1 << 14;
    pub const DISABLE_RX_OWN: u32 = 1 << 13;
    pub const LOOPBACK_MODE: u32 = 1 << 12;
    pub const FULL_DUPLEX_MODE: u32 = 1 << 11;
    pub const CHECKSUM_OFFLOAD: u32 = 1 << 10;
    pub const RETRY_DISABLE: u32 = 1 << 9;
    pub const AUTOMATIC_PAD_CRC_STRIPPING: u32 = 1 << 7;
    pub const BACKOFF_LIMIT_10: u32 = 0 << 5;
    pub const BACKOFF_LIMIT_8: u32 = 1 << 5;
    pub const BACKOFF_LIMIT_4: u32 = 2 << 5;
    pub const BACKOFF_LIMIT_1: u32 = 3 << 5;
    pub const DEFERRAL_CHECK: u32 = 1 << 4;
    pub const TX_ENABLE: u32 = 1 << 3;
    pub const RX_ENABLE: u32 = 1 << 2;
}

/// DMA bus mode register bits.
mod dma_bus_mode {
    pub const MIXED_BURST: u32 = 1 << 26;
    pub const ADDRESS_ALIGNED_BEATS: u32 = 1 << 25;
    pub const PBL_MODE_8X: u32 = 1 << 24;
    pub const USE_SEPARATE_PBL: u32 = 1 << 23;
    pub const RX_DMA_PBL_32: u32 = 32 << 17;
    pub const RX_DMA_PBL_16: u32 = 16 << 17;
    pub const RX_DMA_PBL_8: u32 = 8 << 17;
    pub const RX_DMA_PBL_4: u32 = 4 << 17;
    pub const RX_DMA_PBL_2: u32 = 2 << 17;
    pub const RX_DMA_PBL_1: u32 = 1 << 17;
    pub const FIXED_BURST: u32 = 1 << 16;
    pub const TX_PRIORITY_RATIO_1_1: u32 = 0 << 14;
    pub const TX_PRIORITY_RATIO_2_1: u32 = 1 << 14;
    pub const TX_PRIORITY_RATIO_3_1: u32 = 2 << 14;
    pub const TX_PRIORITY_RATIO_4_1: u32 = 3 << 14;
    pub const DMA_PBL_32: u32 = 32 << 8;
    pub const DMA_PBL_16: u32 = 16 << 8;
    pub const DMA_PBL_8: u32 = 8 << 8;
    pub const DMA_PBL_4: u32 = 4 << 8;
    pub const DMA_PBL_2: u32 = 2 << 8;
    pub const DMA_PBL_1: u32 = 1 << 8;
    pub const ENHANCED_DESCRIPTOR: u32 = 1 << 7;
    pub const DESCRIPTOR_SKIP_LENGTH: u32 = 0 << 2;
    pub const DMA_ARBITRATION_RR: u32 = 0 << 1;
    pub const DMA_ARBITRATION_WRR: u32 = 1 << 1;
    pub const SOFTWARE_RESET: u32 = 1 << 0;
}

/// DMA operation mode register bits.
mod dma_operation_mode {
    pub const DISABLE_DROP_TCP_IP_CHECKSUM_ERROR: u32 = 1 << 26;
    pub const RX_STORE_AND_FORWARD: u32 = 1 << 25;
    pub const DISABLE_FLUSHING_RX_FRAMES: u32 = 1 << 24;
    pub const TX_STORE_AND_FORWARD: u32 = 1 << 21;
    pub const FLUSH_TX_FIFO: u32 = 1 << 20;
    pub const TX_THRESHOLD_CONTROL_64: u32 = 0 << 14;
    pub const TX_THRESHOLD_CONTROL_128: u32 = 1 << 14;
    pub const TX_THRESHOLD_CONTROL_192: u32 = 2 << 14;
    pub const TX_THRESHOLD_CONTROL_256: u32 = 3 << 14;
    pub const TX_THRESHOLD_CONTROL_40: u32 = 4 << 14;
    pub const TX_THRESHOLD_CONTROL_32: u32 = 5 << 14;
    pub const TX_THRESHOLD_CONTROL_24: u32 = 6 << 14;
    pub const TX_THRESHOLD_CONTROL_16: u32 = 7 << 14;
    pub const START_STOP_TX: u32 = 1 << 13;
    pub const FORWARD_ERROR_FRAMES: u32 = 1 << 7;
    pub const FORWARD_UNDERSIZED_GOOD_FRAMES: u32 = 1 << 6;
    pub const RX_THRESHOLD_CONTROL_64: u32 = 0 << 3;
    pub const RX_THRESHOLD_CONTROL_32: u32 = 1 << 3;
    pub const RX_THRESHOLD_CONTROL_96: u32 = 2 << 3;
    pub const RX_THRESHOLD_CONTROL_128: u32 = 3 << 3;
    pub const OPERATE_ON_SECOND_FRAME: u32 = 1 << 2;
    pub const START_STOP_RX: u32 = 1 << 1;
}

/// DWMAC network interface controller.
pub struct DwmacNic<H: DwmacHal> {
    base_addr: NonNull<u8>,
    mac_addr: [u8; 6],
    _phantom: core::marker::PhantomData<H>,
}

// SAFETY: DwmacNic only contains MMIO addresses and configuration data
// The NonNull<u8> is used for MMIO access which is inherently thread-safe
// when properly synchronized at the hardware level
unsafe impl<H: DwmacHal> Send for DwmacNic<H> {}
unsafe impl<H: DwmacHal> Sync for DwmacNic<H> {}

impl<H: DwmacHal> DwmacNic<H> {
    /// Initialize the DWMAC device.
    pub fn init(base_addr: NonNull<u8>, _size: usize) -> Result<Self, &'static str> {
        let nic = Self {
            base_addr,
            mac_addr: [0x6c, 0xcf, 0x39, 0x00, 0x5d, 0x34], // Default MAC from VisionFive 2
            _phantom: core::marker::PhantomData,
        };

        // Initialize hardware
        nic.reset_device()?;
        nic.configure_mac()?;
        nic.configure_dma()?;
        nic.setup_mac_address();

        log::info!("DWMAC device initialized successfully");
        Ok(nic)
    }

    /// Reset the DWMAC device.
    fn reset_device(&self) -> Result<(), &'static str> {
        // Software reset
        self.write_reg(regs::DMA_BUS_MODE, dma_bus_mode::SOFTWARE_RESET);

        // Wait for reset to complete
        let start = core::time::Duration::from_millis(0);
        let timeout = core::time::Duration::from_millis(1000);

        loop {
            if (self.read_reg(regs::DMA_BUS_MODE) & dma_bus_mode::SOFTWARE_RESET) == 0 {
                break;
            }
            H::wait_until(core::time::Duration::from_millis(1))?;
            if start.as_millis() > timeout.as_millis() {
                return Err("DWMAC reset timeout");
            }
        }

        log::debug!("DWMAC device reset completed");
        Ok(())
    }

    /// Configure MAC settings.
    fn configure_mac(&self) -> Result<(), &'static str> {
        let mut config = mac_config::FULL_DUPLEX_MODE
            | mac_config::MII_GMII_SELECT
            | mac_config::INTER_FRAME_GAP_96
            | mac_config::CHECKSUM_OFFLOAD
            | mac_config::AUTOMATIC_PAD_CRC_STRIPPING;

        // Enable TX and RX
        config |= mac_config::TX_ENABLE | mac_config::RX_ENABLE;

        self.write_reg(regs::MAC_CONFIG, config);

        // Configure frame filter (accept all for now)
        self.write_reg(regs::MAC_FRAME_FILTER, 0);

        log::debug!("DWMAC MAC configured");
        Ok(())
    }

    /// Configure DMA settings.
    fn configure_dma(&self) -> Result<(), &'static str> {
        // Configure DMA bus mode
        let bus_mode = dma_bus_mode::FIXED_BURST
            | dma_bus_mode::DMA_PBL_16
            | dma_bus_mode::ENHANCED_DESCRIPTOR
            | dma_bus_mode::ADDRESS_ALIGNED_BEATS;

        self.write_reg(regs::DMA_BUS_MODE, bus_mode);

        // Configure DMA operation mode
        let op_mode = dma_operation_mode::TX_STORE_AND_FORWARD
            | dma_operation_mode::RX_STORE_AND_FORWARD
            | dma_operation_mode::FORWARD_ERROR_FRAMES
            | dma_operation_mode::FORWARD_UNDERSIZED_GOOD_FRAMES;

        self.write_reg(regs::DMA_OPERATION_MODE, op_mode);

        log::debug!("DWMAC DMA configured");
        Ok(())
    }

    /// Setup MAC address in hardware.
    fn setup_mac_address(&self) {
        let mac_high = ((self.mac_addr[5] as u32) << 8) | (self.mac_addr[4] as u32);
        let mac_low = ((self.mac_addr[3] as u32) << 24)
            | ((self.mac_addr[2] as u32) << 16)
            | ((self.mac_addr[1] as u32) << 8)
            | (self.mac_addr[0] as u32);

        self.write_reg(regs::MAC_ADDRESS0_HIGH, mac_high | (1 << 31)); // Address enable
        self.write_reg(regs::MAC_ADDRESS0_LOW, mac_low);

        log::debug!(
            "DWMAC MAC address set to {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
            self.mac_addr[0],
            self.mac_addr[1],
            self.mac_addr[2],
            self.mac_addr[3],
            self.mac_addr[4],
            self.mac_addr[5]
        );
    }

    /// Read a 32-bit register.
    fn read_reg(&self, offset: usize) -> u32 {
        unsafe {
            let addr = self.base_addr.as_ptr().add(offset) as *const u32;
            core::ptr::read_volatile(addr)
        }
    }

    /// Write a 32-bit register.
    fn write_reg(&self, offset: usize, value: u32) {
        unsafe {
            let addr = self.base_addr.as_ptr().add(offset) as *mut u32;
            core::ptr::write_volatile(addr, value);
        }
    }
}

impl<H: DwmacHal> BaseDriverOps for DwmacNic<H> {
    fn device_type(&self) -> DeviceType {
        DeviceType::Net
    }

    fn device_name(&self) -> &str {
        "dwmac-ethernet"
    }
}

impl<H: DwmacHal> NetDriverOps for DwmacNic<H> {
    fn mac_address(&self) -> EthernetAddress {
        EthernetAddress(self.mac_addr)
    }

    fn can_transmit(&self) -> bool {
        // For now, always return true (simplified implementation)
        true
    }

    fn can_receive(&self) -> bool {
        // For now, always return false (no packets available)
        false
    }

    fn rx_queue_size(&self) -> usize {
        256 // Configurable
    }

    fn tx_queue_size(&self) -> usize {
        256 // Configurable
    }

    fn recycle_rx_buffer(&mut self, _buf: NetBufPtr) -> DevResult {
        // TODO: Return RX buffer to hardware descriptor ring
        Ok(())
    }

    fn recycle_tx_buffers(&mut self) -> DevResult {
        // TODO: Reclaim completed TX buffers from descriptor ring
        Ok(())
    }

    fn transmit(&mut self, _buf: NetBufPtr) -> DevResult {
        // TODO: Submit packet for transmission via DMA descriptor ring
        log::warn!("DWMAC transmit not yet implemented");
        Err(DevError::Unsupported)
    }

    fn receive(&mut self) -> DevResult<NetBufPtr> {
        // TODO: Receive packet from hardware via DMA descriptor ring
        Err(DevError::Again)
    }

    fn alloc_tx_buffer(&mut self, _size: usize) -> DevResult<NetBufPtr> {
        // TODO: Implement proper buffer allocation with DMA-coherent memory
        // For now, return an error since we haven't implemented the full buffer management
        log::warn!("DWMAC alloc_tx_buffer not yet implemented");
        Err(DevError::Unsupported)
    }
}
