//! Simple DWMAC Ethernet Driver Tutorial
//!
//! This is a simplified DWMAC driver designed for educational purposes.
//! It demonstrates the core concepts of ethernet driver development
//! without production-level complexity.

mod mdio;
mod regs;
use crate::dwmac::mdio::{
    Yt8531cPhy, YT8531C_BMSR, YT8531C_EXT_CHIP_CONFIG, YT8531C_EXT_CLK_TX_INVERT,
    YT8531C_EXT_RGMII_CONFIG1, YT8531C_EXT_SYNCE_CFG,
};
use crate::dwmac::regs::dma::{DESC_OWN, RDES3};
use crate::dwmac::regs::mac::PACKET_FILTER_ALL;
use crate::{EthernetAddress, NetBufPtr, NetDriverOps};
use axdriver_base::{BaseDriverOps, DevError, DevResult, DeviceType};
use core::ptr::{read_volatile, write_volatile, NonNull};
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use core::{u8, usize};

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
        mb();
    }

    /// Set the descriptor as owned by DMA hardware
    pub fn set_own(&mut self) {
        self.set_des3(self.des3() | DESC_OWN);
    }

    /// Clear the DMA ownership bit
    pub fn clear_own(&mut self) {
        self.set_des3(self.des3() & !DESC_OWN);
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
            _phantom: core::marker::PhantomData,
        }
    }

    pub fn set_buffer_vaddr(&mut self, addr: *mut u8, size: usize) {
        unsafe {
            let phys_addr = H::mmio_virt_to_phys(NonNull::new(addr).unwrap(), size);
            self.set_buffer_paddr(phys_addr);
        }
    }

    pub fn set_buffer_paddr(&mut self, addr: PhysAddr) {
        self.basic.set_des0(addr as u32);
        self.basic.set_des1((addr >> 32) as u32);
        mb();
    }

    pub fn is_owned_by_dma(&self) -> bool {
        self.cache_invalidate();
        self.basic.basic_is_owned_by_dma()
    }

    pub fn cache_invalidate(&self) {
        // jh7110 does not need to invalidate dcache
        // unsafe {
        // CBO.INVAL
        // cbo.inval offset(base)
        // let start: usize = self as *const _ as u32 as usize;
        // let offset = size_of::<Self>();
        // asm!("cbo.inval ({base})", base = in(reg) start);
        // asm!("flushd ({base})", base = in(reg) self as *const _ as u32 as usize);
        // H::cache_flush_range(start, end);
        // }
    }
}

pub struct DescriptorRing<const N: usize, H: DwmacHal> {
    descriptors: *const [DmaExtendedDescriptor<H>; N],
    phy_descriptors: PhysAddr,
    buffers: [*mut u8; N],
    buffer_size: usize,
    head: AtomicUsize,
    tail: AtomicUsize,
    recycle_index: AtomicUsize,
    _phantom: core::marker::PhantomData<H>,
}

impl<const N: usize, H: DwmacHal> DescriptorRing<N, H> {
    fn new(buffer_size: usize) -> Self {
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
            buffers: [core::ptr::null_mut(); N],
            buffer_size,
            head: AtomicUsize::new(0),
            tail: AtomicUsize::new(0),
            recycle_index: AtomicUsize::new(0),
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
            let desc = &mut self.descriptors_mut()[i];
            let (buffer_paddr, buffer_vaddr) = H::dma_alloc(self.buffer_size, 64);
            desc.set_buffer_paddr(buffer_paddr);
            desc.basic.set_des2(0); // we don't need to set buffer size
            desc.basic
                .set_des3(regs::dma::DESC_OWN | RDES3::BUFFER1_VALID_ADDR.bits());
            self.buffers[i] = buffer_vaddr.as_ptr();
        }

        self.head.store(0, Ordering::Relaxed);
        self.tail.store(N - 1, Ordering::Relaxed);
        self.recycle_index.store(0, Ordering::Relaxed);

        self.flush_descriptors();

        Ok(())
    }

    pub fn init_tx_ring(&mut self) -> Result<(), &'static str> {
        for desc in self.descriptors_mut().iter_mut() {
            desc.basic.des0 = 0x0;
            desc.basic.des1 = 0x0;
            desc.basic.des2 = 0x0;
            desc.basic.des3 = 0x0;
        }

        self.head.store(0, Ordering::Relaxed);
        self.tail.store(0, Ordering::Relaxed);
        for addr in self.buffers.iter_mut() {
            *addr = core::ptr::null_mut();
        }

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

    pub fn advance_head(&self) {
        self.head.fetch_add(1, Ordering::Release);
    }

    pub fn get_next_tx_descriptor(&self) -> Option<&DmaExtendedDescriptor<H>> {
        let head = self.head();
        let tail = self.tail();
        let next = self.next_head();

        if next == tail {
            return None;
        }

        let current = &self.descriptors()[head];
        if current.is_owned_by_dma() {
            return None;
        }

        Some(&current)
    }

    pub fn get_next_tx_descriptor_mut(&mut self) -> Option<&mut DmaExtendedDescriptor<H>> {
        let head = self.head();
        let tail = self.tail();
        let next = self.next_head();

        if next == tail {
            return None;
        }

        let current = &self.descriptors()[head];
        if current.is_owned_by_dma() {
            return None;
        }

        Some(&mut self.descriptors_mut()[head])
    }

    pub fn get_completed_rx_descriptor(&mut self) -> Option<(usize, *mut u8)> {
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
            let buffer_ptr = self.buffers[head];
            Some((packet_len, buffer_ptr))
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

        let mut current = tail;
        while current != head {
            let desc = &mut self.descriptors_mut()[current];
            if desc.is_owned_by_dma() {
                break;
            }

            desc.basic.clear_own();
            desc.basic.set_des0(0x0);
            desc.basic.set_des1(0x0);
            desc.basic.set_des2(0x0);
            desc.basic.set_des3(0x0);

            self.buffers[current] = core::ptr::null_mut();

            current = (current + 1) % N;
            reclaimed += 1;
        }

        if reclaimed > 0 {
            log::trace!("tx reclaimed: {}", reclaimed);
            self.tail.fetch_add(reclaimed, Ordering::Release);
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
        self.next_head() == self.tail()
    }

    pub fn is_empty(&self) -> bool {
        self.head() == self.tail()
    }

    pub fn has_available_tx(&self) -> bool {
        !self.is_full() && !self.descriptors()[self.head()].is_owned_by_dma()
    }

    pub fn has_completed_rx(&self) -> bool {
        !self.descriptors()[self.head()].is_owned_by_dma()
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

        // eqos_start(dev=00000000ff728340):
        // jh7110_reset_trigger: deasserting reset 66 (reg=0x13020300, value=0xffe5efc8)
        // jh7110_reset_trigger: deasserting reset 67 (reg=0x13020300, value=0xffe5efc0)

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

        mb();
        nic.reset_dma()?;

        // clk_get_rate(clk=00000000ff7456e0)
        // clk_get_rate(clk=00000000ff72c1c0)
        // clk_get_parent_rate(clk=00000000ff72c1c0)
        // clk_get_parent(clk=00000000ff72c1c0)
        // debug_writel: 00000000160400dc = 0x0000007c
        nic.set_clock_freq(0x7c);

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

        // Initialize PHYs
        mb();
        let _ = nic.init_phy(0).inspect_err(|e| {
            log::error!("PHY initialization failed: {:?}", e);
        });

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
        // nic.enable_dma_interrupts()?;
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

    fn set_clock_freq(&self, freq: u32) {
        log::info!("🔧 Setting clock frequency to {} MHz", freq);
        self.write_reg(regs::mac::US_TIC_COUNTER, freq);
        self.inspect_reg("MAC US_TIC_COUNTER", regs::mac::US_TIC_COUNTER);
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

    fn update_rx_end_addr(&self, addr: u32) {
        log::info!("🔧 Updating RX end address to {:#x}", addr);
        mb();
        self.write_reg(regs::dma::CHAN_RX_END_ADDR, addr as u32);
    }

    /// Configure MAC settings
    fn start_mac(&self) -> Result<(), &'static str> {
        log::info!("🔧 Configuring MAC");

        // self.write_reg(regs::mac::FRAME_FILTER, regs::mac::PACKET_FILTER_ALL);
        // self.inspect_reg("MAC FRAME_FILTER", regs::mac::FRAME_FILTER);

        // // setbits_le32 mac_regs->configuration(0000000016040000): 30e003
        self.write_reg(regs::mac::CONFIG, 0x30e003);
        self.inspect_reg("MAC CONFIG", regs::mac::CONFIG);

        // Set RX ring end address
        self.update_rx_end_addr(self.rx_ring.get_descriptor_paddr(RX_DESC_COUNT - 1) as u32);
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

        let mut phy = Yt8531cPhy::<H>::new(self.base_addr.as_ptr() as usize, phy_addr);
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

            if bmsr != 0x7949 {
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
    fn inspect_reg(&self, name: &str, offset: usize) {
        log::trace!(
            "    🔍 {: >width$}(0x{:0>4x}): 0x{:0>8x}",
            name,
            offset,
            self.read_reg(offset),
            width = 32,
        );
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

    /// 写入TX DMA轮询请求寄存器
    fn start_tx_dma(&self) {
        self.set_bits(regs::dma::CHAN_TX_CTRL, 1, 1);
    }

    fn start_rx_dma(&self) {
        self.set_bits(regs::dma::CHAN_RX_CTRL, 1, 1);
    }

    fn write_rx_tail_id(&self) {
        mb();
        self.write_reg(
            regs::dma::CHAN_RX_END_ADDR,
            self.rx_ring.get_descriptor_paddr(self.rx_ring.tail()) as u32,
        );
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
        if status != 0 {
            self.write_reg(regs::dma::CHAN_STATUS, status);
        }
    }

    fn start_dma_rx(&self) {
        self.set_bits(regs::dma::CHAN_RX_CTRL, 1, 1);
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

        let desc = self
            .tx_ring
            .get_next_tx_descriptor_mut()
            .ok_or(DevError::NoMemory)?;

        desc.set_buffer_vaddr(tx_buf.raw_ptr::<u8>(), tx_buf.packet_len());
        desc.basic.set_des2(tx_buf.packet_len() as u32);
        desc.basic.set_des3(
            regs::dma::DESC_OWN
                | regs::dma::TDES3::FIRST_DESCRIPTOR.bits()
                | regs::dma::TDES3::LAST_DESCRIPTOR.bits()
                | (tx_buf.packet_len() as u32),
        );

        self.tx_ring.buffers[self.tx_ring.head()] = tx_buf.raw_ptr() as *mut u8;

        self.tx_ring.advance_head();
        self.tx_dirty.fetch_add(1, Ordering::AcqRel);

        self.start_tx_dma();

        log::trace!("Packet transmitted, TX index: {}", self.tx_ring.head());

        Ok(())
    }

    fn receive(&mut self) -> DevResult<NetBufPtr> {
        mb();
        self.inspect_dma_regs();
        self.inspect_mtl_regs();
        self.scan_rx_ring();
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

        log::trace!(
            "Packet received, length: {}, RX index: {}",
            packet_len,
            self.rx_ring.head()
        );

        Ok(net_buf)
    }

    fn recycle_rx_buffer(&mut self, rx_buf: NetBufPtr) -> DevResult {
        let recycle_index = self.rx_ring.get_recycle_index();
        let desc = &mut self.rx_ring.descriptors_mut()[recycle_index];

        desc.set_buffer_vaddr(rx_buf.raw_ptr::<u8>(), self.rx_ring.buffer_size);
        desc.basic.set_des2(0);
        desc.basic.set_des3(
            regs::dma::DESC_OWN | RDES3::BUFFER1_VALID_ADDR.bits(),
            // | RDES3::INT_ON_COMPLETION_EN.bits(),
        );
        // desc.basic.set_own();

        self.rx_ring.buffers[recycle_index] = rx_buf.raw_ptr() as *mut u8;
        self.rx_ring.advance_recycle_index();
        log::trace!("RX buffer recycled, RX index: {}", recycle_index);

        mb();
        self.update_rx_end_addr(self.rx_ring.get_descriptor_paddr(recycle_index) as u32);

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

        let (buf_phys, buf_virt) = H::dma_alloc(size, 64);
        if buf_phys == 0 {
            return Err(DevError::NoMemory);
        }

        Ok(NetBufPtr::new(buf_virt, buf_virt, size))
    }
}
