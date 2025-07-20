//! Simple DWMAC Ethernet Driver Tutorial
//!
//! This is a simplified DWMAC driver designed for educational purposes.
//! It demonstrates the core concepts of ethernet driver development
//! without production-level complexity.

mod mdio;
mod mempool;
mod regs;

use crate::dwmac::mdio::{
    Yt8531cPhy, YT8531C_BMSR, YT8531C_EXT_CHIP_CONFIG, YT8531C_EXT_CLK_TX_INVERT,
    YT8531C_EXT_RGMII_CONFIG1, YT8531C_EXT_SYNCE_CFG,
};
use crate::dwmac::mempool::MemPool;
use crate::dwmac::regs::dma::{
    debug_chan_status, debug_tdes3_writeback, DESC_OWN, DMA_CHAN_STATUS_ERI, DMA_CHAN_STATUS_ETI,
    RDES3, TDES3WB,
};
use crate::dwmac::regs::mac::PACKET_FILTER_ALL;
use crate::dwmac::regs::mtl::debug_mtl_tx_fifo_read_controller_status;
use crate::{EthernetAddress, NetBufPtr, NetDriverOps};
use axdriver_base::{BaseDriverOps, DevError, DevResult, DeviceType};
use core::ptr::{read_volatile, write_volatile, NonNull};
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use core::{u8, usize};
use log::log_enabled;

extern crate alloc;

/// Hardware abstraction layer for DWMAC driver
pub trait DwmacHal: Send + Sync {
    /// Allocate DMA-coherent memory
    fn dma_alloc(size: usize, align: usize) -> (PhysAddr, NonNull<u8>);

    /// Deallocate DMA-coherent memory
    unsafe fn dma_dealloc(paddr: PhysAddr, vaddr: NonNull<u8>, size: usize, align: usize) -> i32;

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

    fn cache_flush_range(start: NonNull<u8>, end: NonNull<u8>);
}

/// Physical address type
pub type PhysAddr = usize;

/// Buffer sizes
const TX_DESC_COUNT: usize = 8;
const RX_DESC_COUNT: usize = 16;
const MAX_FRAME_SIZE: usize = 1600; // (1536 + 64 - 1) / 64 * 64;

#[repr(C, align(64))]
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

fn mb() {
    unsafe { core::arch::asm!("fence iorw, iorw") };
}

#[allow(dead_code)]
impl DmaDescriptor {
    fn des0(&self) -> u32 {
        unsafe { read_volatile(&self.des0) }
    }

    fn set_des0(&mut self, des0: u32) {
        unsafe { write_volatile(&mut self.des0, des0) };
    }

    fn des1(&self) -> u32 {
        unsafe { read_volatile(&self.des1) }
    }

    fn set_des1(&mut self, des1: u32) {
        unsafe { write_volatile(&mut self.des1, des1) };
    }

    fn des2(&self) -> u32 {
        unsafe { read_volatile(&self.des2) }
    }

    fn set_des2(&mut self, des2: u32) {
        unsafe { write_volatile(&mut self.des2, des2) };
    }

    fn des3(&self) -> u32 {
        mb();
        unsafe { read_volatile(&self.des3) }
    }

    fn set_des3(&mut self, des3: u32) {
        mb();
        unsafe { write_volatile(&mut self.des3, des3) };
    }

    /// Set the descriptor as owned by DMA hardware
    pub fn set_own(&mut self) {
        self.set_des3(self.des3() | DESC_OWN);
    }

    /// Check if descriptor is owned by DMA hardware
    pub fn basic_is_owned_by_dma(&self) -> bool {
        self.des3() & DESC_OWN != 0
    }
}

#[repr(C, align(64))]
#[derive(Debug, Copy, Clone)]
pub struct DmaExtendedDescriptor<H: DwmacHal> {
    basic: DmaDescriptor,
    // Extended fields for newer DWMAC versions
    // ext_status: u32,
    // reserved1: u32,
    // timestamp_low: u32,
    // timestamp_high: u32,
    _phantom: core::marker::PhantomData<H>,
}

impl<H: DwmacHal> DmaExtendedDescriptor<H> {
    fn zero_init(&mut self) {
        self.basic.des0 = 0;
        self.basic.des1 = 0;
        self.basic.des2 = 0;
        self.basic.des3 = 0;
        mb();
    }

    #[inline(always)]
    pub fn set_buffer_paddr(&mut self, addr: PhysAddr) {
        self.basic.set_des0(addr as u32);
        self.basic.set_des1((addr >> 32) as u32);
        mb();
    }

    #[inline(always)]
    pub fn is_owned_by_dma(&self) -> bool {
        self.cache_invalidate();
        self.basic.basic_is_owned_by_dma()
    }

    #[inline(always)]
    pub fn cache_invalidate(&self) {
        // jh7110 does not need to invalidate dcache
    }
}

const POOL_SIZE: usize = 4096;

pub struct DescriptorRing<const N: usize, H: DwmacHal> {
    descriptors: *const [DmaExtendedDescriptor<H>; N],
    phy_descriptors: PhysAddr,
    mem_pool: MemPool<H, POOL_SIZE, MAX_FRAME_SIZE>,
    head: AtomicUsize,
    tail: AtomicUsize,
    _phantom: core::marker::PhantomData<H>,
}

impl<const N: usize, H: DwmacHal> DescriptorRing<N, H> {
    fn new() -> Self {
        let (phy_descriptors, descriptors) =
            H::dma_alloc(N * size_of::<DmaExtendedDescriptor<H>>(), 64);
        log::info!(
            "🔍 Descriptor ring allocated at bus: 0x{:0>8x}, virt: {:p}",
            phy_descriptors,
            descriptors.as_ptr()
        );
        Self {
            descriptors: descriptors.as_ptr().cast(),
            phy_descriptors,
            mem_pool: MemPool::<H, POOL_SIZE, MAX_FRAME_SIZE>::new(),
            head: AtomicUsize::new(0),
            tail: AtomicUsize::new(0),
            _phantom: core::marker::PhantomData,
        }
    }

    fn descriptors(&self) -> &[DmaExtendedDescriptor<H>; N] {
        unsafe { &*self.descriptors }
    }

    fn descriptors_mut(&self) -> &mut [DmaExtendedDescriptor<H>; N] {
        unsafe { &mut *(self.descriptors as *mut _) }
    }

    fn flush_descriptors(&self) {
        unsafe {
            H::cache_flush_range(
                NonNull::new(self.descriptors as *mut u8).unwrap(),
                NonNull::new(self.descriptors.add(N - 1) as *mut u8).unwrap(),
            );
        }
        mb();
    }

    pub fn init_rx_ring(&mut self) -> Result<(), &'static str> {
        for i in 0..N {
            let buffer = self.mem_pool.alloc().expect("IMPOSSIBLE");
            let desc = &mut self.descriptors_mut()[i];
            desc.set_buffer_paddr(self.mem_pool.bus_addr(buffer));
            desc.basic.set_des2(0); // we don't need to set buffer size
            desc.basic
                .set_des3(regs::dma::DESC_OWN | RDES3::BUFFER1_VALID_ADDR.bits());
        }

        self.head.store(0, Ordering::Relaxed);
        self.tail.store(N - 1, Ordering::Relaxed);

        self.flush_descriptors();

        Ok(())
    }

    pub fn init_tx_ring(&mut self) -> Result<(), &'static str> {
        for desc in self.descriptors_mut().iter_mut() {
            desc.zero_init();
        }

        self.head.store(0, Ordering::Relaxed);
        self.tail.store(0, Ordering::Relaxed);

        self.flush_descriptors();

        Ok(())
    }

    pub fn get_descriptor_paddr(&self, index: usize) -> PhysAddr {
        self.phy_descriptors + index * size_of::<DmaExtendedDescriptor<H>>()
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

    pub fn next_tail(&self) -> usize {
        (self.tail.load(Ordering::Acquire) + 1) % N
    }

    pub fn advance_head(&self) {
        self.head.fetch_add(1, Ordering::Release);
    }

    pub fn advance_tail(&self) {
        self.tail.fetch_add(1, Ordering::Release);
    }

    pub fn get_next_tx_descriptor_mut(&mut self) -> Option<(usize, &mut DmaExtendedDescriptor<H>)> {
        let head = self.head();
        let next = self.next_tail();

        if next == head {
            return None;
        }

        let tail = self.tail();
        let current = &self.descriptors()[tail];
        if current.is_owned_by_dma() {
            log::error!("🔍 TX descriptor is owned by DMA");
            return None;
        }

        Some((tail, &mut self.descriptors_mut()[tail]))
    }

    pub fn get_completed_rx_descriptor(&mut self) -> Option<(usize, NonNull<u8>, usize)> {
        let head = self.head();
        let desc = &self.descriptors()[head];
        if desc.is_owned_by_dma() {
            return None;
        }
        self.advance_head();
        let desc = &mut self.descriptors_mut()[head];

        let result = if (desc.basic.des3() & RDES3::ERROR_SUMMARY.bits()) != 0 {
            log::error!("🔍 Error summary: {:#x}", desc.basic.des3());
            None
        } else {
            let packet_len = (desc.basic.des3() & 0x7fff) as usize;
            let bus_addr = desc.basic.des0() as usize | ((desc.basic.des1() as usize) << 32);
            let buffer_ptr = self.mem_pool.bus_addr_to_ptr(bus_addr);
            Some((head, buffer_ptr, packet_len))
        };

        result
    }

    pub fn advance_rx_tail(&self) {
        self.tail.fetch_add(1, Ordering::Release);
    }

    pub fn reclaim_tx_descriptors(&mut self) -> usize {
        let mut reclaimed = 0;
        let tail = self.tail();
        let head = self.head();

        let mut current = head;
        while current != tail {
            let desc = &mut self.descriptors_mut()[current];
            if desc.is_owned_by_dma() {
                break;
            }
            if (desc.basic.des3() & TDES3WB::ERROR_SUMMARY.bits()) != 0 {
                debug_tdes3_writeback(desc.basic.des3());
            }
            log::trace!("🔍 Reclaiming TX descriptor: {}", current);
            let bus_addr = desc.basic.des0() as usize | ((desc.basic.des1() as usize) << 32);
            // let len = desc.basic.des2() as usize;
            let ptr = self.mem_pool.bus_addr_to_ptr(bus_addr);
            self.mem_pool.free(ptr);

            desc.zero_init();

            current = (current + 1) % N;
            reclaimed += 1;
        }

        if reclaimed > 0 {
            log::trace!("tx reclaimed: {}", reclaimed);
            self.head.fetch_add(reclaimed, Ordering::Release);
        }

        reclaimed
    }

    pub fn recycle_rx_descriptor(&mut self, desc_index: usize) -> Result<(), &'static str> {
        let desc = &mut self.descriptors_mut()[desc_index];

        desc.basic
            .set_des3(regs::dma::DESC_OWN | RDES3::BUFFER1_VALID_ADDR.bits());

        Ok(())
    }

    pub fn is_full(&self) -> bool {
        self.next_tail() == self.head()
    }

    pub fn is_empty(&self) -> bool {
        self.head() == self.tail()
    }

    pub fn has_available_tx(&self) -> bool {
        !self.is_full()
    }

    pub fn has_completed_rx(&self) -> bool {
        !self.descriptors()[self.head()].is_owned_by_dma()
    }
}

/// Simple DWMAC network interface
pub struct DwmacNic<H: DwmacHal> {
    base_addr: NonNull<u8>,
    mac_addr: [u8; 6],
    link_up: AtomicBool,

    tx_ring: DescriptorRing<TX_DESC_COUNT, H>,
    rx_ring: DescriptorRing<RX_DESC_COUNT, H>,

    _phantom: core::marker::PhantomData<H>,
}

unsafe impl<H: DwmacHal> Send for DwmacNic<H> {}
unsafe impl<H: DwmacHal> Sync for DwmacNic<H> {}

impl<H: DwmacHal> DwmacNic<H> {
    /// Initialize the DWMAC device
    pub fn init(base_addr: NonNull<u8>, _size: usize) -> Result<Self, &'static str> {
        log::info!("🚀 Initializing DWMAC ethernet driver (tutorial version)");

        // eqos_start(dev=00000000ff728340):
        // jh7110_reset_trigger: deasserting reset 66 (reg=0x13020300, value=0xffe5efc8)
        // jh7110_reset_trigger: deasserting reset 67 (reg=0x13020300, value=0xffe5efc0)

        // Platform-specific setup
        H::configure_platform()?;

        let mut nic = Self {
            base_addr,
            mac_addr: [0x35, 0x5d, 0x00, 0x39, 0xcf, 0x6c], // Default MAC
            link_up: AtomicBool::new(true),

            tx_ring: DescriptorRing::<TX_DESC_COUNT, H>::new(),
            rx_ring: DescriptorRing::<RX_DESC_COUNT, H>::new(),

            _phantom: core::marker::PhantomData,
        };

        mb();
        nic.reset_dma()?;

        // Initialize PHYs
        mb();
        let _ = nic.init_phy(0).inspect_err(|e| {
            log::error!("PHY initialization failed: {:?}", e);
        });

        // clk_get_rate(clk=00000000ff7456e0)
        // clk_get_rate(clk=00000000ff72c1c0)
        // clk_get_parent_rate(clk=00000000ff72c1c0)
        // clk_get_parent(clk=00000000ff72c1c0)
        // debug_writel: 00000000160400dc = 0x0000007c
        nic.set_clock_freq(0x7c)?;

        // eqos_mdio_read(dev=00000000ff728340, addr=0, reg=1):
        // eqos_mdio_read: val=796d
        // eqos_mdio_read(dev=00000000ff728340, addr=0, reg=17):
        // eqos_mdio_read: val=7c00
        // eqos_mdio_write(dev=00000000ff728340, addr=0, reg=30, val=a003):
        // ytphy_modify_ext: regnum=0xa003, mask=0x4000, set=0x4000
        // eqos_mdio_read(dev=00000000ff728340, addr=0, reg=31):
        // eqos_mdio_read: val=48f0
        // eqos_mdio_write(dev=00000000ff728340, addr=0, reg=31, val=48f0):
        // eqos_adjust_link(dev=00000000ff728340):
        // eqos_set_full_duplex(dev=00000000ff728340):
        // eqos_set_mii_speed_100(dev=00000000ff728340):

        // debug_writel: 0000000016040d18 = 0x00000010

        // Initialize MTL
        nic.init_mtl()?;
        nic.flow_control()?;

        // debug_writel: 0000000016040300 = 0x0000355d
        // debug_writel: 0000000016040304 = 0x0039cf6c
        nic.setup_mac_address();

        nic.stop_dma();
        // debug_writel: 0000000016041004 = 0x0002080e
        nic.set_sys_bus_mode(0x0002080e); // dump from u-boot

        // debug_writel: 0000000016041110 = 0x00000000
        // debug_writel: 0000000016041114 = 0xff745840
        // debug_writel: 000000001604112c = 0x00000003
        // debug_writel: 0000000016041118 = 0x00000000
        // debug_writel: 000000001604111c = 0xff745980
        // debug_writel: 0000000016041130 = 0x00000003
        // debug_writel: 0000000016041128 = 0xff745a40
        nic.setup_descriptor_rings()?;
        // eqos_start: OK

        // eqos_send(dev=00000000ff728340, packet=00000000ff74ae80, length=350):
        // debug_writel: 0000000016041120 = 0xff745880
        // eqos_recv(dev=00000000ff728340, flags=1):
        // eqos_recv: *packetp=00000000ff746140, length=60
        // eqos_free_pkt(packet=00000000ff746140, length=60)
        // debug_writel: 0000000016041128 = 0xff745980
        // eqos_recv(dev=00000000ff728340, flags=0):
        // eqos_recv: *packetp=00000000ff746780, length=60
        // eqos_free_pkt(packet=00000000ff746780, length=60)
        // debug_writel: 0000000016041128 = 0xff7459c0
        // eqos_send(dev=00000000ff728340, packet=00000000ff74ae80, length=350):
        // debug_writel: 0000000016041120 = 0xff7458c0
        // eqos_send(dev=00000000ff728340, packet=00000000ff74ae80, length=350):
        // debug_writel: 0000000016041120 = 0xff745900
        // eqos_send(dev=00000000ff728340, packet=00000000ff74ae80, length=350):
        // debug_writel: 0000000016041120 = 0xff745840
        // eqos_recv(dev=00000000ff728340, flags=1):
        // eqos_recv: *packetp=00000000ff746dc0, length=342
        // eqos_send(dev=00000000ff728340, packet=00000000ff74b4c0, length=350):
        // debug_writel: 0000000016041120 = 0xff745880
        // eqos_free_pkt(packet=00000000ff746dc0, length=342)
        // debug_writel: 0000000016041128 = 0xff745a00
        // eqos_recv(dev=00000000ff728340, flags=0):
        // eqos_recv: *packetp=00000000ff747400, length=342
        // eqos_send(dev=00000000ff728340, packet=00000000ff74b380, length=42):
        // debug_writel: 0000000016041120 = 0xff7458c0
        // eqos_free_pkt(packet=00000000ff747400, length=342)
        // debug_writel: 0000000016041128 = 0xff745a40
        // eqos_recv(dev=00000000ff728340, flags=0):
        // eqos_recv: *packetp=00000000ff746140, length=62
        // eqos_free_pkt(packet=00000000ff746140, length=62)
        // debug_writel: 0000000016041128 = 0xff745980
        // DHCP client bound to address 192.168.1.47 (589 ms)
        // StarFive #

        nic.start_dma()?;
        nic.enable_dma_interrupts()?;
        nic.start_mac()?;

        nic.inspect_reg("DMA STATUS", regs::dma::DMA_STATUS);

        log::info!("✅ DWMAC initialization complete");
        Ok(nic)
    }

    /// Enable DMA interrupts
    fn enable_dma_interrupts(&self) -> Result<(), &'static str> {
        log::info!("🔧 Enabling DMA channel 0 interrupts...");
        self.write_reg(regs::mac::INTERRUPT_ENABLE, 0x0);

        self.inspect_reg("DMA CHAN_INTR_ENABLE", regs::dma::CHAN_INTR_ENABLE);
        self.write_reg(
            regs::dma::CHAN_INTR_ENABLE,
            regs::dma::InterruptMask::DEFAULT_MASK_4_10.bits(),
        );
        self.inspect_reg("DMA CHAN_INTR_ENABLE", regs::dma::CHAN_INTR_ENABLE);

        log::info!("🔧 Enabling GMAC interrupts...");

        self.inspect_reg("MAC INTERRUPT_STATUS", regs::mac::INTERRUPT_STATUS);
        self.write_reg(regs::mac::INTERRUPT_STATUS, 0x1);

        if self.read_reg(regs::mac::INTERRUPT_STATUS) != 0 {
            log::info!("🔧 Waiting for GMAC interrupt status to clear...");
            for _ in 0..1000 {
                if self.read_reg(regs::mac::INTERRUPT_STATUS) == 0 {
                    break;
                }
                H::wait_until(core::time::Duration::from_millis(1))?;
            }

            if self.read_reg(regs::mac::INTERRUPT_STATUS) != 0 {
                log::error!("GMAC interrupt status not cleared");
                self.inspect_reg("MAC INTERRUPT_STATUS", regs::mac::INTERRUPT_STATUS);
                self.inspect_reg("PHYIF_CONTROL_STATUS", regs::mac::PHYIF_CONTROL_STATUS);
                self.inspect_reg("DEBUG_STATUS0", regs::mac::DEBUG_STATUS);
            }
        }

        self.write_reg(
            regs::mac::INTERRUPT_ENABLE,
            regs::mac::INT_DEFAULT_ENABLE | regs::mac::MacInterruptEnable::RGMII.bits(),
        );
        self.inspect_reg("MAC INTERRUPT_ENABLE", regs::mac::INTERRUPT_ENABLE);

        self.inspect_reg("MAC INTERRUPT_STATUS", regs::mac::INTERRUPT_STATUS);

        Ok(())
    }

    /// Reset the hardware
    fn reset_dma(&self) -> Result<(), &'static str> {
        log::info!("🔄 Resetting DMA Mode");

        self.inspect_reg("DMA BUS_MODE", regs::dma::BUS_MODE);

        H::wait_until(core::time::Duration::from_millis(10))?;
        self.set_bits(regs::dma::BUS_MODE, regs::dma::DMA_RESET, 1);

        // Wait for reset to complete
        let mut timeout = 1000;
        while self.read_reg(regs::dma::BUS_MODE) & regs::dma::DMA_RESET != 0 {
            if timeout == 0 {
                log::error!("🔴 Hardware reset timeout");
                return Err("🔴 Hardware reset timeout");
            }
            timeout -= 1;
            H::wait_until(core::time::Duration::from_millis(1))?;
        }

        log::info!("✅ DMA Mode reset complete");
        Ok(())
    }

    fn set_clock_freq(&self, freq: u32) -> Result<(), &'static str> {
        log::info!("🔧 Setting clock frequency to {} MHz", freq);
        self.write_reg(regs::mac::US_TIC_COUNTER, freq);
        self.inspect_reg("MAC US_TIC_COUNTER", regs::mac::US_TIC_COUNTER);

        let phy = Yt8531cPhy::<H>::new(self.base_addr.as_ptr() as usize, 0);
        for _ in 0..100 {
            let bmsr = phy.read_reg(YT8531C_BMSR).map_err(|e| {
                log::warn!("⚠️  PHY not responding: {:?}", e);
                "PHY not responding"
            })?;

            // NOTE: 非常重要，PHY 稳定前配置 GMAC 会导致网卡无法正常工作。
            if bmsr == 0x796d {
                // 0x796d is target value
                log::info!("🔍 PHY BMSR: {:#x}", bmsr);
                break;
            }
            H::wait_until(core::time::Duration::from_millis(10))?;
        }

        phy.setbits_ext_reg(YT8531C_EXT_RGMII_CONFIG1, 0x4000, 0x4000)
            .map_err(|e| {
                log::error!("⚠️  PHY not responding: {:?}", e);
                "PHY not responding"
            })?;
        Ok(())
    }

    /// Setup DMA descriptor rings
    fn setup_descriptor_rings(&mut self) -> Result<(), &'static str> {
        log::info!("🔧 Setting up descriptor rings");

        self.rx_ring.init_rx_ring()?;
        self.tx_ring.init_tx_ring()?;

        // Set TX ring base address
        self.write_reg(
            regs::dma::CHAN_TX_BASE_ADDR_HI,
            (self.tx_ring.get_descriptor_paddr(0) >> 32) as u32,
        );
        self.write_reg(
            regs::dma::CHAN_TX_BASE_ADDR,
            self.tx_ring.get_descriptor_paddr(0) as u32,
        );
        self.inspect_reg("DMA CHAN_TX_BASE_ADDR", regs::dma::CHAN_TX_BASE_ADDR);
        // Set TX ring length
        self.write_reg(regs::dma::CHAN_TX_RING_LEN, (TX_DESC_COUNT - 1) as u32);

        // Set RX ring base address
        self.write_reg(
            regs::dma::CHAN_RX_BASE_ADDR_HI,
            (self.rx_ring.get_descriptor_paddr(0) >> 32) as u32,
        );
        self.write_reg(
            regs::dma::CHAN_RX_BASE_ADDR,
            self.rx_ring.get_descriptor_paddr(0) as u32,
        );
        self.inspect_reg("DMA CHAN_RX_BASE_ADDR", regs::dma::CHAN_RX_BASE_ADDR);
        // Set RX ring length
        self.write_reg(regs::dma::CHAN_RX_RING_LEN, (RX_DESC_COUNT - 1) as u32);

        // Set TX ring control
        self.write_reg(regs::dma::CHAN_TX_CTRL, 0x80011);
        self.inspect_reg("DMA CHAN_TX_CTRL", regs::dma::CHAN_TX_CTRL);

        // Set RX ring control
        self.write_reg(regs::dma::CHAN_RX_CTRL, 0x80c81);
        self.inspect_reg("DMA CHAN_RX_CTRL", regs::dma::CHAN_RX_CTRL);

        // Set TX ring end address
        // self.write_reg(
        //     regs::dma::CHAN_TX_END_ADDR,
        //     self.tx_ring.get_descriptor_paddr(TX_DESC_COUNT - 1) as u32,
        // );
        // self.inspect_reg("DMA CHAN_TX_END_ADDR", regs::dma::CHAN_TX_END_ADDR);

        log::info!("✅ Descriptor rings ready");
        Ok(())
    }

    fn update_rx_end_addr(&self, index: usize) {
        mb();
        let addr = self.rx_ring.get_descriptor_paddr(index);
        log::trace!("🔧 Updating RX end address to {:#x}", addr);
        self.write_reg(regs::dma::CHAN_RX_END_ADDR, addr as u32);
        self.inspect_reg("DMA CHAN_RX_END_ADDR", regs::dma::CHAN_RX_END_ADDR);
    }

    fn update_tx_end_addr(&self, index: usize) {
        mb();
        let addr = self.tx_ring.get_descriptor_paddr(index);
        log::trace!("🔧 Updating TX end address to {:#x}", addr);
        self.write_reg(regs::dma::CHAN_TX_END_ADDR, addr as u32);
        self.inspect_reg("DMA CHAN_TX_END_ADDR", regs::dma::CHAN_TX_END_ADDR);
    }

    /// Configure MAC settings
    fn start_mac(&self) -> Result<(), &'static str> {
        log::info!("🔧 Configuring MAC");

        self.write_reg(regs::mac::FRAME_FILTER, regs::mac::PacketFilter::PR.bits());
        self.inspect_reg("MAC FRAME_FILTER", regs::mac::FRAME_FILTER);

        // // setbits_le32 mac_regs->configuration(0000000016040000): 30e003
        self.write_reg(regs::mac::CONFIG, 0x30e003);
        self.inspect_reg("MAC CONFIG", regs::mac::CONFIG);

        // Set RX ring end address
        self.update_rx_end_addr(RX_DESC_COUNT - 1);
        self.inspect_reg("DMA CHAN_RX_END_ADDR", regs::dma::CHAN_RX_END_ADDR);

        // let status = self.read_reg(regs::dma::CHAN_STATUS);
        // log::info!("DMA CHAN_STATUS: {:#x}", status);
        // for _ in 0..10 {
        //     log::error!("DMA CHAN_STATUS is not 0: {:#x}", status);
        //     self.write_reg(regs::dma::CHAN_STATUS, status);
        //     self.inspect_reg("DMA CHAN_STATUS", regs::dma::CHAN_STATUS);
        //     if self.read_reg(regs::dma::CHAN_STATUS) == 0 {
        //         log::info!("DMA CHAN_STATUS resetted");
        //         break;
        //     }
        //     H::wait_until(core::time::Duration::from_millis(10))?;
        // }

        log::info!("🔧 MAC enabled");
        self.inspect_reg("MAC VERSION", regs::mac::VERSION);
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

    fn set_sys_bus_mode(&self, value: u32) {
        self.write_reg(regs::dma::SYS_BUS_MODE, value);
        self.inspect_reg("DMA SYS_BUS_MODE", regs::dma::SYS_BUS_MODE);
    }

    fn stop_dma(&self) {
        log::info!("🔧 Stopping DMA");
        // setbits_le32 dma_regs->ch0_tx_control(0000000016041104): 10
        self.write_reg(regs::dma::CHAN_TX_CTRL, 0x10);
        // clrsetbits_le32 dma_regs->ch0_rx_control(0000000016041108), mask: 7ffe, val: c80, final: c80
        self.set_bits(regs::dma::CHAN_RX_CTRL, 0x7ffe, 0xc80);
        // setbits_le32 dma_regs->ch0_control(0000000016041100): 190000
        self.write_reg(regs::dma::CHAN_BASE_ADDR, 0x190000);
        // clrsetbits_le32 dma_regs->ch0_tx_control(0000000016041104), mask: 3f0000, val: 80000, final: 80010
        self.set_bits(regs::dma::CHAN_TX_CTRL, 0x3f0000, 0x80000);
        // clrsetbits_le32 dma_regs->ch0_rx_control(0000000016041108), mask: 3f0000, val: 80000, final: 80c80
        self.set_bits(regs::dma::CHAN_RX_CTRL, 0x3f0000, 0x80000);

        self.inspect_reg("DMA CHAN_TX_CTRL", regs::dma::CHAN_TX_CTRL);
        self.inspect_reg("DMA CHAN_RX_CTRL", regs::dma::CHAN_RX_CTRL);
        self.inspect_reg("DMA CHAN_BASE_ADDR", regs::dma::CHAN_BASE_ADDR);
    }

    /// Start DMA operations
    fn start_dma(&self) -> Result<(), &'static str> {
        log::info!("🚀 Starting DMA");
        // setbits_le32 dma_regs->ch0_tx_control(0000000016041104): 80011
        self.write_reg(regs::dma::CHAN_TX_CTRL, 0x80011);
        // setbits_le32 dma_regs->ch0_rx_control(0000000016041108): 80c81
        self.write_reg(regs::dma::CHAN_RX_CTRL, 0x80c81);
        // debug_writel: 0000000016041128 = 0xff745a40
        Ok(())
    }

    fn init_mtl(&self) -> Result<(), &'static str> {
        log::info!("🔧 Initializing MTL");
        // setbits_le32 mtl_regs->txq0_operation_mode(0000000016040d00): 7000a
        self.write_reg(regs::mtl::BASE_ADDR, 0x7000a);
        self.inspect_reg("MTL BASE_ADDR", regs::mtl::BASE_ADDR);

        // clrsetbits_le32 mtl_regs->txq0_operation_mode(0000000016040d00): 7000a
        self.set_bits(regs::mtl::TXQ0_OPERATION_MODE, !0, 0x7000a);
        self.inspect_reg("MTL TXQ0_OPERATION_MODE", regs::mtl::TXQ0_OPERATION_MODE);

        // debug_writel: 0000000016040d18 = 0x00000010
        self.write_reg(regs::mtl::TXQ0_QUANTUM_WEIGHT, 0x10);
        self.inspect_reg("MTL TXQ0_QUANTUM_WEIGHT", regs::mtl::TXQ0_QUANTUM_WEIGHT);

        // setbits_le32 mtl_regs->rxq0_operation_mode(0000000016040d30): 700020
        self.write_reg(regs::mtl::RXQ0_OPERATION_MODE, 0x700020);
        self.inspect_reg("MTL RXQ0_OPERATION_MODE", regs::mtl::RXQ0_OPERATION_MODE);

        // clrsetbits_le32 mtl_regs->txq0_operation_mode(0000000016040d00), mask: 1ff0000, val: 70000, final: 7000a
        self.set_bits(regs::mtl::TXQ0_OPERATION_MODE, 0x1ff0000, 0x70000);
        self.inspect_reg("MTL TXQ0_OPERATION_MODE", regs::mtl::TXQ0_OPERATION_MODE);

        // clrsetbits_le32 mtl_regs->rxq0_operation_mode(0000000016040d30), mask: 3ff00000, val: 700000, final: 700020
        self.set_bits(regs::mtl::RXQ0_OPERATION_MODE, 0x3ff00000, 0x700000);
        self.inspect_reg("MTL RXQ0_OPERATION_MODE", regs::mtl::RXQ0_OPERATION_MODE);

        Ok(())
    }

    fn flow_control(&self) -> Result<(), &'static str> {
        log::info!("🔧 Configuring flow control");
        // clrsetbits_le32 mac_regs->rxq_ctrl0(00000000160400a0), mask: 3, val: 2, final: 0
        self.set_bits(regs::mac::RXQ_CTRL0, 0x3, 0x2);
        self.inspect_reg("MAC RXQ_CTRL0", regs::mac::RXQ_CTRL0);
        // setbits_le32 mac_regs->unused_0a4(00000000160400a4): 0
        self.write_reg(regs::mac::RXQ_CTRL1, 0);
        self.inspect_reg("MAC RXQ_CTRL1", regs::mac::RXQ_CTRL1);
        // setbits_le32 mac_regs->unused_004[1](0000000016040008): 1
        self.write_reg(regs::mac::FRAME_FILTER, PACKET_FILTER_ALL);
        self.inspect_reg("MAC FRAME_FILTER", regs::mac::FRAME_FILTER);
        // setbits_le32 mac_regs->q0_tx_flow_ctrl(0000000016040070): ffff0000
        self.write_reg(regs::mac::Q0_TX_FLOW_CTRL, 0xffff0000);
        self.inspect_reg("MAC Q0_TX_FLOW_CTRL", regs::mac::Q0_TX_FLOW_CTRL);
        // clrbits_le32 mac_regs->txq_prty_map0(0000000016040098), mask: ff, val: 0, final: 0
        self.set_bits(regs::mac::TXQ_PRTY_MAP0, 0xff, 0);
        self.inspect_reg("MAC TXQ_PRTY_MAP0", regs::mac::TXQ_PRTY_MAP0);
        // clrbits_le32 mac_regs->rxq_ctrl2(00000000160400a8), mask: ff, val: 0, final: 0
        self.set_bits(regs::mac::RXQ_CTRL2, 0xff, 0);
        self.inspect_reg("MAC RXQ_CTRL2", regs::mac::RXQ_CTRL2);
        // setbits_le32 mac_regs->q0_tx_flow_ctrl(0000000016040070): ffff0002
        self.write_reg(regs::mac::Q0_TX_FLOW_CTRL, 0xffff0002);
        self.inspect_reg("MAC Q0_TX_FLOW_CTRL", regs::mac::Q0_TX_FLOW_CTRL);
        // setbits_le32 mac_regs->rx_flow_ctrl(0000000016040090): 1
        self.write_reg(regs::mac::RX_FLOW_CTRL, 0x1);
        self.inspect_reg("MAC RX_FLOW_CTRL", regs::mac::RX_FLOW_CTRL);
        // clrsetbits_le32 mac_regs->configuration(0000000016040000), mask: 8b0000, val: 300000, final: 30e000
        self.set_bits(regs::mac::CONFIG, 0x8b0000, 0x300000);
        self.inspect_reg("MAC CONFIG", regs::mac::CONFIG);

        Ok(())
    }

    /// Initialize PHY (simplified)
    fn init_phy(&self, phy_addr: u8) -> Result<(), &'static str> {
        log::info!("🔧 Initializing PHY (basic)");

        let phy = Yt8531cPhy::<H>::new(self.base_addr.as_ptr() as usize, phy_addr);
        phy.soft_reset().map_err(|e| {
            log::warn!("⚠️  PHY not responding: {:?}", e);
            "PHY not responding"
        })?;

        let phy_id = phy.get_phy_id().map_err(|e| {
            log::warn!("⚠️  PHY not responding: {:?}", e);
            "PHY not responding"
        })?;

        log::info!("🔍 PHY ID: {:#x}", phy_id);
        if phy_id != 0x4f51e91b {
            log::error!("PHY ID mismatch: {:#x}", phy_id);
            return Err("PHY ID mismatch");
        }

        // eqos_mdio_write(dev=00000000ff728340, addr=0, reg=0, val=1200):
        // eqos_mdio_write(dev=00000000ff728340, addr=0, reg=30, val=a012):
        // ytphy_modify_ext: regnum=0xa012, mask=0x0040, set=0x0000
        phy.setbits_ext_reg(YT8531C_EXT_SYNCE_CFG, 0x40, 0)
            .map_err(|e| {
                log::warn!("⚠️  PHY not responding: {:?}", e);
                "PHY not responding"
            })?;

        // eqos_mdio_read(dev=00000000ff728340, addr=0, reg=31):
        // eqos_mdio_read: val=c8
        // eqos_mdio_write(dev=00000000ff728340, addr=0, reg=31, val=88):
        // eqos_mdio_write(dev=00000000ff728340, addr=0, reg=30, val=a001):
        // ytphy_modify_ext: regnum=0xa001, mask=0x0100, set=0x0000
        phy.setbits_ext_reg(YT8531C_EXT_CHIP_CONFIG, 0x0100, 0)
            .map_err(|e| {
                log::warn!("⚠️  PHY not responding: {:?}", e);
                "PHY not responding"
            })?;

        // eqos_mdio_read(dev=00000000ff728340, addr=0, reg=31):
        // eqos_mdio_read: val=8120
        // eqos_mdio_write(dev=00000000ff728340, addr=0, reg=31, val=8020):
        // eqos_mdio_write(dev=00000000ff728340, addr=0, reg=30, val=a003):
        // ytphy_modify_ext: regnum=0xa003, mask=0x3c0f, set=0x0800
        phy.setbits_ext_reg(YT8531C_EXT_RGMII_CONFIG1, 0x3c0f, 0x0800)
            .map_err(|e| {
                log::warn!("⚠️  PHY not responding: {:?}", e);
                "PHY not responding"
            })?;

        // eqos_mdio_read(dev=00000000ff728340, addr=0, reg=31):
        // eqos_mdio_read: val=f1
        // eqos_mdio_write(dev=00000000ff728340, addr=0, reg=31, val=8f0):
        // eqos_mdio_write(dev=00000000ff728340, addr=0, reg=30, val=a001):
        // eqos_mdio_read(dev=00000000ff728340, addr=0, reg=31):
        // eqos_mdio_read: val=8020
        // ytphy_read_ext: regnum=0xa001, ret=0x8020
        // eqos_mdio_write(dev=00000000ff728340, addr=0, reg=30, val=a010):
        // ytphy_modify_ext: regnum=0xa010, mask=0xe000, set=0xc000
        phy.setbits_ext_reg(YT8531C_EXT_CLK_TX_INVERT, 0xe000, 0xc000)
            .map_err(|e| {
                log::warn!("⚠️  PHY not responding: {:?}", e);
                "PHY not responding"
            })?;

        // eqos_mdio_read(dev=00000000ff728340, addr=0, reg=31):
        // eqos_mdio_read: val=6bff
        // eqos_mdio_write(dev=00000000ff728340, addr=0, reg=31, val=cbff):
        // eqos_mdio_write(dev=00000000ff728340, addr=0, reg=30, val=a001):
        // eqos_mdio_read(dev=00000000ff728340, addr=0, reg=31):
        // eqos_mdio_read: val=8020
        // ytphy_read_ext: regnum=0xa001, ret=0x8020
        // eqos_mdio_write(dev=00000000ff728340, addr=0, reg=30, val=a010):
        // ytphy_modify_ext: regnum=0xa010, mask=0x1030, set=0x0030
        phy.setbits_ext_reg(YT8531C_EXT_CLK_TX_INVERT, 0x1030, 0x0030)
            .map_err(|e| {
                log::warn!("⚠️  PHY not responding: {:?}", e);
                "PHY not responding"
            })?;

        for _ in 0..100 {
            let bmsr = phy.read_reg(YT8531C_BMSR).map_err(|e| {
                log::warn!("⚠️  PHY not responding: {:?}", e);
                "PHY not responding"
            })?;

            // NOTE: 非常重要，PHY 稳定前配置 GMAC 会导致网卡无法正常工作。
            if bmsr == 0x796d {
                // 0x796d is target value
                log::info!("🔍 PHY BMSR: {:#x}", bmsr);
                break;
            }
            H::wait_until(core::time::Duration::from_millis(10))?;
        }

        let config = phy.read_ext_reg(YT8531C_EXT_CHIP_CONFIG).map_err(|e| {
            log::warn!("⚠️  PHY not responding: {:?}", e);
            "PHY not responding"
        })?;
        log::info!("🔍 PHY EXT_CHIP_CONFIG: {:#x}", config);

        // phy.write_ext_reg(YT8531C_EXT_RGMII_CONFIG1, 0x850) // Magic number
        //     .map_err(|e| {
        //         log::warn!("⚠️  PHY not responding: {:?}", e);
        //         "PHY not responding"
        //     })?;

        // eqos_mdio_write(dev=00000000ff728340, addr=0, reg=30, val=a003):
        // ytphy_modify_ext: regnum=0xa003, mask=0x4000, set=0x4000
        phy.setbits_ext_reg(YT8531C_EXT_RGMII_CONFIG1, 0x4000, 0x4000)
            .map_err(|e| {
                log::warn!("⚠️  PHY not responding: {:?}", e);
                "PHY not responding"
            })?;

        // phy.write_ext_reg(YT8531C_EXT_CHIP_CONFIG, 0x7960) // Magic number
        //     .map_err(|e| {
        //         log::warn!("⚠️  PHY not responding: {:?}", e);
        //         "PHY not responding"
        //     })?;

        // phy.configure_rgmii_id().map_err(|e| {
        //     log::warn!("⚠️  PHY not responding: {:?}", e);
        //     "PHY not responding"
        // })?;

        // phy.set_phy_linus().map_err(|e| {
        //     log::warn!("⚠️  PHY not responding: {:?}", e);
        //     "PHY not responding"
        // })?;

        log::info!(
            "🔍 PHY EXT_RGMII_CONFIG1: {:#x}",
            phy.read_ext_reg(YT8531C_EXT_RGMII_CONFIG1).unwrap()
        );

        // let mut phyif_ctrl = self.read_reg(regs::mac::PHYIF_CONTROL_STATUS);
        // // 设置为RGMII模式 (bits [2:1] = 0b01)
        // phyif_ctrl &= !(0x3 << 1); // 清除PHY接口模式位
        // phyif_ctrl |= 0x1 << 1; // 设置为RGMII模式

        // // 启用链路状态检测
        // phyif_ctrl |= 1 << 16; // LNKMOD: 链路模式
        // phyif_ctrl |= 1 << 17; // LNKSTS: 链路状态

        // phyif_ctrl |= 0x000d0000; // dump from linux

        // self.write_reg(regs::mac::PHYIF_CONTROL_STATUS, phyif_ctrl);

        // H::wait_until(core::time::Duration::from_millis(100))?;
        // // let mut timeout = 1000;
        // // while self.read_reg(regs::mac::PHYIF_CONTROL_STATUS) != phyif_ctrl {
        // //     if timeout == 0 {
        // //         log::error!("PHYIF_CONTROL_STATUS timeout");
        // //         break;
        // //     }
        // //     timeout -= 1;
        // //     H::wait_until(core::time::Duration::from_millis(1))?;
        // // }

        // self.inspect_reg("PHYIF_CONTROL_STATUS", regs::mac::PHYIF_CONTROL_STATUS);

        Ok(())
    }

    fn set_qos(&self) -> Result<(), &'static str> {
        self.write_reg(
            regs::dma::GMAC_Q0_TX_FLOW_CTRL,
            regs::dma::GMAC_Q0_TX_FLOW_CTRL_TFE,
        );
        self.inspect_reg("DMA GMAC_Q0_TX_FLOW_CTRL", regs::dma::GMAC_Q0_TX_FLOW_CTRL);
        Ok(())
    }

    /// Read hardware register
    fn read_reg(&self, offset: usize) -> u32 {
        unsafe {
            let addr = self.base_addr.as_ptr().add(offset) as *const u32;
            let res = core::ptr::read_volatile(addr);
            mb();
            res
        }
    }

    /// Inspect hardware register
    #[inline(always)]
    fn inspect_reg(&self, name: &str, offset: usize) {
        if log_enabled!(log::Level::Trace) {
            log::trace!(
                "    🔍 {: >width$}(0x{:0>4x}): 0x{:0>8x}",
                name,
                offset,
                self.read_reg(offset),
                width = 32,
            );
        }
    }

    /// Write hardware register
    fn write_reg(&self, offset: usize, value: u32) {
        unsafe {
            let addr = self.base_addr.as_ptr().add(offset) as *mut u32;
            mb();
            core::ptr::write_volatile(addr, value);
        }
    }

    /// Set bits in a register
    fn set_bits(&self, offset: usize, mask: u32, value: u32) {
        self.write_reg(offset, (self.read_reg(offset) & !mask) | (value & mask));
    }

    // fn update_link_status(&self) {
    //     let status = self.read_reg(regs::mac::INTERRUPT_STATUS);
    //     if status & regs::mac::MacInterruptStatus::LINK_UP.bits() != 0 {
    //         self.link_up.store(true, Ordering::Release);
    //     } else {
    //         self.link_up.store(false, Ordering::Release);
    //     }
    // }

    fn inspect_mtl_regs(&self) {
        if COUNTER.load(Ordering::Acquire) % 1000000 != 0 {
            return;
        }
        self.inspect_reg("MTL TXQ0_OPERATION_MODE", regs::mtl::TXQ0_OPERATION_MODE);
        self.inspect_reg("MTL TXQ0_DEBUG", regs::mtl::TXQ0_DEBUG);
        self.inspect_reg("MTL TXQ0_QUANTUM_WEIGHT", regs::mtl::TXQ0_QUANTUM_WEIGHT);
        self.inspect_reg("MTL RXQ0_OPERATION_MODE", regs::mtl::RXQ0_OPERATION_MODE);
        self.inspect_reg("MTL RXQ0_DEBUG", regs::mtl::RXQ0_DEBUG);
    }

    fn inspect_dma_regs(&self) {
        COUNTER.fetch_add(1, Ordering::AcqRel);
        if COUNTER.load(Ordering::Acquire) % 1000000 != 0 {
            return;
        }
        log::debug!("--------------------------------");
        self.inspect_reg("MAC CONFIG", regs::mac::CONFIG);
        self.inspect_reg("PHYIF_CONTROL_STATUS", regs::mac::PHYIF_CONTROL_STATUS);
        self.inspect_reg("GMAC_DEBUG_STATUS", regs::mac::DEBUG_STATUS);
        self.inspect_reg("DMA SYS_BUS_MODE", regs::dma::SYS_BUS_MODE);
        self.inspect_reg("DMA_STATUS", regs::dma::DMA_STATUS);
        self.inspect_reg("DMA_DEBUG_STATUS0", regs::dma::DMA_DEBUG_STATUS0);
        self.inspect_reg("DMA_DEBUG_STATUS1", regs::dma::DMA_DEBUG_STATUS1);
        self.inspect_reg("DMA_DEBUG_STATUS2", regs::dma::DMA_DEBUG_STATUS2);
        self.inspect_reg("DMA_CHAN_CUR_TX_DESC", regs::dma::CHAN_CUR_TX_DESC);
        self.inspect_reg("DMA_CHAN_CUR_RX_DESC", regs::dma::CHAN_CUR_RX_DESC);
        self.inspect_reg("DMA_CHAN_STATUS", regs::dma::CHAN_STATUS);
        // let dma_intr_status = self.read_reg(0x1100 + 0x60);
        // if dma_intr_status != 0 {
        //     log::info!("DMA_INTR_STATUS: {:#x}", dma_intr_status);
        //     self.write_reg(0x1100 + 0x60, dma_intr_status);
        // }
        let dma_chan0_debug_status = self.read_reg(regs::dma::CHAN_STATUS);
        if dma_chan0_debug_status != 0 {
            regs::dma::debug_chan_status(dma_chan0_debug_status);
        }
        // let tx_fsm = dma_chan0_debug_status & 0x7;
        // let rx_fsm = (dma_chan0_debug_status >> 16) & 0x7;
        // if tx_fsm != 0 || rx_fsm != 0 {
        //     log::info!(
        //         "DMA_CHAN0_DEBUG_STATUS: {:#x}, tx_fsm: {:#x}, rx_fsm: {:#x}",
        //         dma_chan0_debug_status,
        //         tx_fsm,
        //         rx_fsm
        //     );
        // }
        self.inspect_reg("DMA_CHAN_RX_CTRL", regs::dma::CHAN_RX_CTRL);
    }

    fn scan_rx_ring(&self) {
        if COUNTER.load(Ordering::Acquire) % 1000000 != 0 {
            return;
        }
        for i in 0..RX_DESC_COUNT {
            let desc = &self.rx_ring.descriptors()[i];
            if !desc.is_owned_by_dma() {
                log::debug!("RX buffer owned by CPU, index: {}", i);
            }
            if (desc.basic.des3() & !RDES3::BUFFER1_VALID_ADDR.bits() & !DESC_OWN) != 0 {
                log::debug!("RX buffer status: {}", desc.basic.des3());
            }
            if desc.basic.des2() != 0 {
                log::debug!("RX buffer size: {}", desc.basic.des2());
            }
        }
    }

    fn clear_intr_status(&self) {
        let status = self.read_reg(regs::dma::CHAN_STATUS);
        if status & !DMA_CHAN_STATUS_ERI & !DMA_CHAN_STATUS_ETI != 0 {
            debug_chan_status(status);

            debug_mtl_tx_fifo_read_controller_status(self.read_reg(regs::mtl::TXQ0_DEBUG));

            self.write_reg(regs::dma::CHAN_STATUS, status);
        }
    }
}

static COUNTER: AtomicUsize = AtomicUsize::new(0);

// Implement network driver traits
impl<H: DwmacHal> BaseDriverOps for DwmacNic<H> {
    fn device_type(&self) -> DeviceType {
        DeviceType::Net
    }

    fn device_name(&self) -> &str {
        "dwmac-5.2"
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
        log::trace!("transmit");
        if !self.can_transmit() {
            return Err(DevError::Again);
        }

        let bus_addr = self.tx_ring.mem_pool.bus_addr(tx_buf.buf_ptr);

        let (index, desc) = self
            .tx_ring
            .get_next_tx_descriptor_mut()
            .ok_or(DevError::NoMemory)?;

        desc.set_buffer_paddr(bus_addr);
        desc.basic.set_des2(tx_buf.packet_len() as u32);
        desc.basic.set_des3(
            regs::dma::DESC_OWN
                | regs::dma::TDES3::FIRST_DESCRIPTOR.bits()
                | regs::dma::TDES3::LAST_DESCRIPTOR.bits()
                | (tx_buf.packet_len() as u32),
        );

        self.tx_ring.advance_tail();

        self.update_tx_end_addr(self.tx_ring.tail());

        log::trace!("Packet transmitted, TX index: {}", index);

        Ok(())
    }

    fn receive(&mut self) -> DevResult<NetBufPtr> {
        // self.inspect_dma_regs();
        // self.inspect_mtl_regs();
        // self.scan_rx_ring();
        if !self.can_receive() {
            return Err(DevError::Again);
        }

        let (head, buffer_ptr, packet_len) = self
            .rx_ring
            .get_completed_rx_descriptor()
            .ok_or(DevError::Again)?;

        let net_buf = NetBufPtr::new(self.rx_ring.mem_pool.base_ptr(), buffer_ptr, packet_len);

        let desc = &mut self.rx_ring.descriptors_mut()[head];

        let buf_ptr = self.rx_ring.mem_pool.alloc().ok_or(DevError::NoMemory)?;
        let bus_addr = self.rx_ring.mem_pool.bus_addr(buf_ptr);

        desc.set_buffer_paddr(bus_addr);
        desc.basic.set_des2(0);
        desc.basic
            .set_des3(regs::dma::DESC_OWN | RDES3::BUFFER1_VALID_ADDR.bits());

        log::trace!("RX buffer recycled, RX index: {}", head);

        self.update_rx_end_addr(head);

        self.clear_intr_status();

        log::trace!(
            "Packet received, length: {}, RX index: {}",
            packet_len,
            self.rx_ring.head()
        );

        Ok(net_buf)
    }

    fn recycle_rx_buffer(&mut self, rx_buf: NetBufPtr) -> DevResult {
        self.rx_ring.mem_pool.free(rx_buf.buf_ptr);

        Ok(())
    }

    fn recycle_tx_buffers(&mut self) -> DevResult {
        let _ = self.tx_ring.reclaim_tx_descriptors();
        Ok(())
    }

    fn alloc_tx_buffer(&mut self, size: usize) -> DevResult<NetBufPtr> {
        if size > MAX_FRAME_SIZE {
            return Err(DevError::InvalidParam);
        }

        let buf_ptr = self.tx_ring.mem_pool.alloc().ok_or(DevError::NoMemory)?;

        Ok(NetBufPtr::new(
            self.tx_ring.mem_pool.base_ptr(),
            buf_ptr,
            size,
        ))
    }
}
