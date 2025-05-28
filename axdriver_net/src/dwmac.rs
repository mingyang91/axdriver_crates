//! Synopsys DesignWare MAC (DWMAC) Ethernet driver.
//!
//! This driver supports the Synopsys DesignWare Ethernet MAC v5.10a
//! as found in the StarFive VisionFive 2 board.

use crate::{EthernetAddress, NetBufPtr, NetDriverOps};
use axdriver_base::{BaseDriverOps, DevError, DevResult, DeviceType};
use core::ptr::NonNull;
use core::sync::atomic::{AtomicUsize, Ordering};

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

/// Number of TX descriptors in the ring.
const TX_DESC_COUNT: usize = 256;

/// Number of RX descriptors in the ring.
const RX_DESC_COUNT: usize = 256;

/// Maximum frame size (MTU + headers).
const MAX_FRAME_SIZE: usize = 1536;

/// DMA descriptor structure for enhanced descriptors.
#[repr(C, align(16))]
#[derive(Debug, Clone, Copy)]
struct DmaDescriptor {
    /// Status and control word.
    status: u32,
    /// Buffer size and control.
    control: u32,
    /// Buffer 1 address.
    buffer1: u32,
    /// Buffer 2 address or next descriptor.
    buffer2: u32,
}

impl DmaDescriptor {
    const fn new() -> Self {
        Self {
            status: 0,
            control: 0,
            buffer1: 0,
            buffer2: 0,
        }
    }
}

/// TX descriptor status bits.
mod tx_desc_status {
    pub const OWN: u32 = 1 << 31;
    pub const IC: u32 = 1 << 30; // Interrupt on completion
    pub const LS: u32 = 1 << 29; // Last segment
    pub const FS: u32 = 1 << 28; // First segment
    pub const DC: u32 = 1 << 27; // Disable CRC
    pub const DP: u32 = 1 << 26; // Disable padding
    pub const TTSE: u32 = 1 << 25; // Transmit timestamp enable
    pub const CIC_NONE: u32 = 0 << 22;
    pub const CIC_IP: u32 = 1 << 22;
    pub const CIC_IP_PAYLOAD: u32 = 2 << 22;
    pub const CIC_IP_PAYLOAD_PSEUDO: u32 = 3 << 22;
    pub const TER: u32 = 1 << 21; // Transmit end of ring
    pub const TCH: u32 = 1 << 20; // Second address chained
}

/// RX descriptor status bits.
mod rx_desc_status {
    pub const OWN: u32 = 1 << 31;
    pub const AFM: u32 = 1 << 30; // Destination address filter fail
    pub const FL_MASK: u32 = 0x3FFF << 16; // Frame length
    pub const FL_SHIFT: u32 = 16;
    pub const ES: u32 = 1 << 15; // Error summary
    pub const DE: u32 = 1 << 14; // Descriptor error
    pub const SAF: u32 = 1 << 13; // Source address filter fail
    pub const LE: u32 = 1 << 12; // Length error
    pub const OE: u32 = 1 << 11; // Overflow error
    pub const VLAN: u32 = 1 << 10; // VLAN tag
    pub const FS: u32 = 1 << 9; // First descriptor
    pub const LS: u32 = 1 << 8; // Last descriptor
    pub const IPC_GIANT: u32 = 1 << 7; // IPC giant frame
    pub const LC: u32 = 1 << 6; // Late collision
    pub const FT: u32 = 1 << 5; // Frame type
    pub const RWT: u32 = 1 << 4; // Receive watchdog timeout
    pub const RE: u32 = 1 << 3; // Receive error
    pub const DBE: u32 = 1 << 2; // Dribble bit error
    pub const CE: u32 = 1 << 1; // CRC error
    pub const PCE: u32 = 1 << 0; // Payload checksum error
}

/// RX descriptor control bits.
mod rx_desc_control {
    pub const RER: u32 = 1 << 25; // Receive end of ring
    pub const RCH: u32 = 1 << 24; // Second address chained
    pub const RBS2_MASK: u32 = 0x1FFF << 11; // Receive buffer 2 size
    pub const RBS1_MASK: u32 = 0x1FFF; // Receive buffer 1 size
}

/// DMA status register bits.
mod dma_status {
    pub const NIS: u32 = 1 << 16; // Normal interrupt summary
    pub const AIS: u32 = 1 << 15; // Abnormal interrupt summary
    pub const ERI: u32 = 1 << 14; // Early receive interrupt
    pub const FBI: u32 = 1 << 13; // Fatal bus error interrupt
    pub const ETI: u32 = 1 << 10; // Early transmit interrupt
    pub const RWT: u32 = 1 << 9; // Receive watchdog timeout
    pub const RPS: u32 = 1 << 8; // Receive process stopped
    pub const RU: u32 = 1 << 7; // Receive buffer unavailable
    pub const RI: u32 = 1 << 6; // Receive interrupt
    pub const UNF: u32 = 1 << 5; // Transmit underflow
    pub const OVF: u32 = 1 << 4; // Receive overflow
    pub const TJT: u32 = 1 << 3; // Transmit jabber timeout
    pub const TU: u32 = 1 << 2; // Transmit buffer unavailable
    pub const TPS: u32 = 1 << 1; // Transmit process stopped
    pub const TI: u32 = 1 << 0; // Transmit interrupt
}

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

    // TX descriptor ring
    tx_descriptors: NonNull<DmaDescriptor>,
    tx_descriptors_phys: PhysAddr,
    tx_buffers: [Option<NetBufPtr>; TX_DESC_COUNT],
    tx_next: AtomicUsize,
    tx_clean: AtomicUsize,

    // RX descriptor ring
    rx_descriptors: NonNull<DmaDescriptor>,
    rx_descriptors_phys: PhysAddr,
    rx_buffers: [Option<NetBufPtr>; RX_DESC_COUNT],
    rx_next: AtomicUsize,

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
        log::info!("DWMAC init: base_addr={:p}", base_addr.as_ptr());

        // Allocate DMA-coherent memory for descriptor rings
        let tx_desc_size = core::mem::size_of::<DmaDescriptor>() * TX_DESC_COUNT;
        let rx_desc_size = core::mem::size_of::<DmaDescriptor>() * RX_DESC_COUNT;

        log::debug!("Allocating TX descriptors: {} bytes", tx_desc_size);
        let (tx_descriptors_phys, tx_descriptors_virt) = H::dma_alloc(tx_desc_size);
        log::debug!("Allocating RX descriptors: {} bytes", rx_desc_size);
        let (rx_descriptors_phys, rx_descriptors_virt) = H::dma_alloc(rx_desc_size);

        if tx_descriptors_phys == 0 || rx_descriptors_phys == 0 {
            log::error!(
                "Failed to allocate DMA memory: tx_phys={:#x}, rx_phys={:#x}",
                tx_descriptors_phys,
                rx_descriptors_phys
            );
            return Err("Failed to allocate DMA memory for descriptors");
        }

        log::debug!(
            "TX descriptors: phys={:#x}, virt={:p}",
            tx_descriptors_phys,
            tx_descriptors_virt.as_ptr()
        );
        log::debug!(
            "RX descriptors: phys={:#x}, virt={:p}",
            rx_descriptors_phys,
            rx_descriptors_virt.as_ptr()
        );

        let tx_descriptors = NonNull::new(tx_descriptors_virt.as_ptr() as *mut DmaDescriptor)
            .ok_or("Invalid TX descriptor pointer")?;
        let rx_descriptors = NonNull::new(rx_descriptors_virt.as_ptr() as *mut DmaDescriptor)
            .ok_or("Invalid RX descriptor pointer")?;

        let mut nic = Self {
            base_addr,
            mac_addr: [0x6c, 0xcf, 0x39, 0x00, 0x5d, 0x34], // Default MAC from VisionFive 2

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
        log::debug!("Resetting DWMAC device...");
        nic.reset_device()?;
        log::debug!("Setting up descriptor rings...");
        nic.setup_descriptor_rings()?;
        log::debug!("Configuring MAC...");
        nic.configure_mac()?;
        log::debug!("Configuring DMA...");
        nic.configure_dma()?;
        log::debug!("Setting up MAC address...");
        nic.setup_mac_address();
        log::debug!("Starting DMA...");
        nic.start_dma()?;

        log::info!("DWMAC device initialized successfully");
        Ok(nic)
    }

    /// Reset the DWMAC device.
    fn reset_device(&self) -> Result<(), &'static str> {
        log::debug!("Starting DWMAC device reset");

        // Check if we can read from the device first
        let version = self.read_reg(regs::MAC_VERSION);
        log::debug!("DWMAC version register: {:#x}", version);

        // Perform software reset
        self.write_reg(regs::DMA_BUS_MODE, dma_bus_mode::SOFTWARE_RESET);

        // Wait for reset to complete with timeout (simple counter-based)
        let mut timeout_counter = 0;
        const MAX_TIMEOUT_ITERATIONS: u32 = 1000;

        loop {
            let bus_mode = self.read_reg(regs::DMA_BUS_MODE);
            if bus_mode & dma_bus_mode::SOFTWARE_RESET == 0 {
                log::debug!("DWMAC reset completed, bus_mode={:#x}", bus_mode);
                break;
            }

            timeout_counter += 1;
            if timeout_counter > MAX_TIMEOUT_ITERATIONS {
                log::error!("DWMAC reset timeout, bus_mode={:#x}", bus_mode);
                return Err("Device reset timeout");
            }

            // Small delay before checking again
            H::wait_until(core::time::Duration::from_millis(1))?;
        }

        log::debug!("DWMAC device reset completed successfully");
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

    /// Setup DMA descriptor rings.
    fn setup_descriptor_rings(&mut self) -> Result<(), &'static str> {
        log::debug!(
            "Setting up TX descriptor ring at phys={:#x}, virt={:p}",
            self.tx_descriptors_phys,
            self.tx_descriptors.as_ptr()
        );

        // Initialize TX descriptors
        unsafe {
            let tx_desc_slice =
                core::slice::from_raw_parts_mut(self.tx_descriptors.as_ptr(), TX_DESC_COUNT);

            for (i, desc) in tx_desc_slice.iter_mut().enumerate() {
                *desc = DmaDescriptor::new();
                // Set up chaining for ring buffer
                if i == TX_DESC_COUNT - 1 {
                    desc.control |= tx_desc_status::TER; // End of ring
                } else {
                    desc.buffer2 = (self.tx_descriptors_phys
                        + (i + 1) * core::mem::size_of::<DmaDescriptor>())
                        as u32;
                    desc.control |= tx_desc_status::TCH; // Second address chained
                }

                if i < 4 {
                    log::trace!(
                        "TX desc[{}]: status={:#x}, control={:#x}, buffer1={:#x}, buffer2={:#x}",
                        i,
                        desc.status,
                        desc.control,
                        desc.buffer1,
                        desc.buffer2
                    );
                }
            }
        }

        log::debug!(
            "Setting up RX descriptor ring at phys={:#x}, virt={:p}",
            self.rx_descriptors_phys,
            self.rx_descriptors.as_ptr()
        );

        // Initialize RX descriptors and allocate buffers
        unsafe {
            let rx_desc_slice =
                core::slice::from_raw_parts_mut(self.rx_descriptors.as_ptr(), RX_DESC_COUNT);

            for (i, desc) in rx_desc_slice.iter_mut().enumerate() {
                *desc = DmaDescriptor::new();

                // Allocate RX buffer
                log::trace!("Allocating RX buffer {} of size {}", i, MAX_FRAME_SIZE);
                let (buf_phys, buf_virt) = H::dma_alloc(MAX_FRAME_SIZE);
                if buf_phys == 0 {
                    log::error!("Failed to allocate RX buffer {}", i);
                    return Err("Failed to allocate RX buffer");
                }

                let rx_buf = NetBufPtr::new(buf_virt, buf_virt, MAX_FRAME_SIZE);
                self.rx_buffers[i] = Some(rx_buf);

                desc.buffer1 = buf_phys as u32;
                desc.control = MAX_FRAME_SIZE as u32 & rx_desc_control::RBS1_MASK;

                // Set up chaining for ring buffer
                if i == RX_DESC_COUNT - 1 {
                    desc.control |= rx_desc_control::RER; // End of ring
                } else {
                    desc.buffer2 = (self.rx_descriptors_phys
                        + (i + 1) * core::mem::size_of::<DmaDescriptor>())
                        as u32;
                    desc.control |= rx_desc_control::RCH; // Second address chained
                }

                // Give ownership to DMA
                desc.status = rx_desc_status::OWN;

                if i < 4 {
                    log::trace!(
                        "RX desc[{}]: status={:#x}, control={:#x}, buffer1={:#x}, buffer2={:#x}",
                        i,
                        desc.status,
                        desc.control,
                        desc.buffer1,
                        desc.buffer2
                    );
                }
            }
        }

        // Set descriptor list addresses
        log::debug!(
            "Setting TX descriptor list address: {:#x}",
            self.tx_descriptors_phys
        );
        self.write_reg(
            regs::DMA_TX_DESCRIPTOR_LIST,
            self.tx_descriptors_phys as u32,
        );

        log::debug!(
            "Setting RX descriptor list address: {:#x}",
            self.rx_descriptors_phys
        );
        self.write_reg(
            regs::DMA_RX_DESCRIPTOR_LIST,
            self.rx_descriptors_phys as u32,
        );

        log::debug!("DWMAC descriptor rings initialized");
        Ok(())
    }

    /// Start DMA operations.
    fn start_dma(&self) -> Result<(), &'static str> {
        // Enable DMA interrupts
        let int_enable = dma_status::NIS | dma_status::AIS | dma_status::RI | dma_status::TI;
        self.write_reg(regs::DMA_INTERRUPT_ENABLE, int_enable);

        // Start TX and RX DMA
        let mut op_mode = self.read_reg(regs::DMA_OPERATION_MODE);
        op_mode |= dma_operation_mode::START_STOP_TX | dma_operation_mode::START_STOP_RX;
        self.write_reg(regs::DMA_OPERATION_MODE, op_mode);

        log::debug!("DWMAC DMA started");
        Ok(())
    }

    /// Handle DMA interrupts.
    pub fn handle_interrupt(&mut self) {
        let status = self.read_reg(regs::DMA_STATUS);

        // Clear interrupts
        self.write_reg(regs::DMA_STATUS, status);

        if status & dma_status::TI != 0 {
            // TX interrupt - reclaim completed buffers
            self.reclaim_tx_buffers();
        }

        if status & dma_status::RI != 0 {
            // RX interrupt - packets available
            log::trace!("RX interrupt received");
        }

        if status & dma_status::AIS != 0 {
            log::warn!("DMA abnormal interrupt: {:#x}", status);
        }
    }

    /// Reclaim completed TX buffers.
    fn reclaim_tx_buffers(&mut self) {
        let mut clean_idx = self.tx_clean.load(Ordering::Acquire);
        let next_idx = self.tx_next.load(Ordering::Acquire);

        while clean_idx != next_idx {
            unsafe {
                let desc = &mut *self.tx_descriptors.as_ptr().add(clean_idx);

                // Check if descriptor is still owned by DMA
                if desc.status & tx_desc_status::OWN != 0 {
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

    /// Read a 32-bit register.
    fn read_reg(&self, offset: usize) -> u32 {
        unsafe {
            let addr = self.base_addr.as_ptr().add(offset) as *const u32;
            // Validate the address is within expected MMIO range
            let addr_val = addr as usize;
            if addr_val < 0x16030000 || addr_val >= 0x16040000 {
                log::warn!(
                    "DWMAC read_reg: suspicious address {:#x} (offset {:#x})",
                    addr_val,
                    offset
                );
            }
            let value = core::ptr::read_volatile(addr);
            log::trace!(
                "DWMAC read_reg: offset={:#x}, addr={:#x}, value={:#x}",
                offset,
                addr_val,
                value
            );
            value
        }
    }

    /// Write a 32-bit register.
    fn write_reg(&self, offset: usize, value: u32) {
        unsafe {
            let addr = self.base_addr.as_ptr().add(offset) as *mut u32;
            // Validate the address is within expected MMIO range
            let addr_val = addr as usize;
            if addr_val < 0x16030000 || addr_val >= 0x16040000 {
                log::warn!(
                    "DWMAC write_reg: suspicious address {:#x} (offset {:#x})",
                    addr_val,
                    offset
                );
            }
            log::trace!(
                "DWMAC write_reg: offset={:#x}, addr={:#x}, value={:#x}",
                offset,
                addr_val,
                value
            );
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
        let next_idx = self.tx_next.load(Ordering::Acquire);
        let clean_idx = self.tx_clean.load(Ordering::Acquire);

        // Check if there's space in the TX ring
        let next_next = (next_idx + 1) % TX_DESC_COUNT;
        next_next != clean_idx
    }

    fn can_receive(&self) -> bool {
        let rx_idx = self.rx_next.load(Ordering::Acquire);
        unsafe {
            let desc = &*self.rx_descriptors.as_ptr().add(rx_idx);
            // Check if descriptor is owned by CPU (packet received)
            desc.status & rx_desc_status::OWN == 0
        }
    }

    fn rx_queue_size(&self) -> usize {
        RX_DESC_COUNT
    }

    fn tx_queue_size(&self) -> usize {
        TX_DESC_COUNT
    }

    fn recycle_rx_buffer(&mut self, rx_buf: NetBufPtr) -> DevResult {
        let rx_idx = self.rx_next.load(Ordering::Acquire);

        unsafe {
            let desc = &mut *self.rx_descriptors.as_ptr().add(rx_idx);

            // Reset the buffer
            let buf_phys =
                H::mmio_virt_to_phys(NonNull::new(rx_buf.raw_ptr()).unwrap(), MAX_FRAME_SIZE);
            desc.buffer1 = buf_phys as u32;
            desc.control = (desc.control & !rx_desc_control::RBS1_MASK)
                | (MAX_FRAME_SIZE as u32 & rx_desc_control::RBS1_MASK);

            // Give ownership back to DMA
            desc.status = rx_desc_status::OWN;

            // Store the buffer
            self.rx_buffers[rx_idx] = Some(rx_buf);

            // Advance to next descriptor
            let next_rx = (rx_idx + 1) % RX_DESC_COUNT;
            self.rx_next.store(next_rx, Ordering::Release);
        }

        // Poll demand to resume RX if needed
        self.write_reg(regs::DMA_RX_POLL_DEMAND, 1);

        Ok(())
    }

    fn recycle_tx_buffers(&mut self) -> DevResult {
        self.reclaim_tx_buffers();
        Ok(())
    }

    fn transmit(&mut self, tx_buf: NetBufPtr) -> DevResult {
        if !self.can_transmit() {
            return Err(DevError::Again);
        }

        let tx_idx = self.tx_next.load(Ordering::Acquire);

        unsafe {
            let desc = &mut *self.tx_descriptors.as_ptr().add(tx_idx);

            // Set up buffer
            let buf_phys =
                H::mmio_virt_to_phys(NonNull::new(tx_buf.raw_ptr()).unwrap(), tx_buf.packet_len());
            desc.buffer1 = buf_phys as u32;
            desc.control = (tx_buf.packet_len() as u32 & 0x1FFF)
                | tx_desc_status::FS
                | tx_desc_status::LS
                | tx_desc_status::IC
                | tx_desc_status::CIC_IP_PAYLOAD_PSEUDO;

            // Store the buffer
            self.tx_buffers[tx_idx] = Some(tx_buf);

            // Give ownership to DMA
            desc.status = tx_desc_status::OWN;

            // Advance to next descriptor
            let next_tx = (tx_idx + 1) % TX_DESC_COUNT;
            self.tx_next.store(next_tx, Ordering::Release);
        }

        // Poll demand to start transmission
        self.write_reg(regs::DMA_TX_POLL_DEMAND, 1);

        log::trace!("Packet queued for transmission");
        Ok(())
    }

    fn receive(&mut self) -> DevResult<NetBufPtr> {
        if !self.can_receive() {
            return Err(DevError::Again);
        }

        let rx_idx = self.rx_next.load(Ordering::Acquire);

        unsafe {
            let desc = &*self.rx_descriptors.as_ptr().add(rx_idx);

            // Check for errors
            if desc.status & rx_desc_status::ES != 0 {
                log::warn!("RX error: {:#x}", desc.status);
                // Skip this packet and recycle the buffer
                let desc_mut = &mut *self.rx_descriptors.as_ptr().add(rx_idx);
                desc_mut.status = rx_desc_status::OWN;
                let next_rx = (rx_idx + 1) % RX_DESC_COUNT;
                self.rx_next.store(next_rx, Ordering::Release);
                return Err(DevError::Again);
            }

            // Extract frame length
            let frame_len =
                ((desc.status & rx_desc_status::FL_MASK) >> rx_desc_status::FL_SHIFT) as usize;
            if frame_len < 4 {
                log::warn!("Invalid frame length: {}", frame_len);
                return Err(DevError::Again);
            }

            // Remove CRC from length
            let packet_len = frame_len - 4;

            // Take the buffer
            if let Some(rx_buf) = self.rx_buffers[rx_idx].take() {
                // Create a new NetBufPtr with the correct packet length
                let new_rx_buf = NetBufPtr::new(
                    NonNull::new(rx_buf.raw_ptr()).unwrap(),
                    NonNull::new(rx_buf.raw_ptr()).unwrap(),
                    packet_len,
                );

                log::trace!("Received packet of {} bytes", packet_len);
                Ok(new_rx_buf)
            } else {
                log::error!("No RX buffer available at index {}", rx_idx);
                Err(DevError::BadState)
            }
        }
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
