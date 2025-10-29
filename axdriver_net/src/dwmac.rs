use crate::{
    BaseDriverOps, DevError, DevResult, DeviceType, EthernetAddress, NetBufPtr, NetDriverOps,
};
use core::ptr::NonNull;
use core::sync::atomic::Ordering;
pub use dwmac_rs::DwmacHal;
use dwmac_rs::{
    DwmacNic as DwmacNicImpl, DESC_OWN, MAX_FRAME_SIZE, RDES3, RX_DESC_COUNT, TDES3, TX_DESC_COUNT,
};

pub type PhysAddr = usize;

pub struct DwmacNic<H: DwmacHal> {
    inner: DwmacNicImpl<H>,
}

impl<H: DwmacHal> DwmacNic<H> {
    pub fn init(base_addr: NonNull<u8>, _size: usize) -> Result<Self, &'static str> {
        let inner = DwmacNicImpl::<H>::init(base_addr, _size)?;
        Ok(Self { inner })
    }
}

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
        EthernetAddress(self.inner.mac_addr)
    }

    fn can_transmit(&self) -> bool {
        // self.inspect_dma_regs();
        // self.inspect_mtl_regs();
        self.inner.link_up.load(Ordering::Acquire) && self.inner.tx_ring.has_available_tx()
    }

    fn can_receive(&self) -> bool {
        self.inner.link_up.load(Ordering::Acquire) && self.inner.rx_ring.has_completed_rx()
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

        let bus_addr = self.inner.tx_ring.mem_pool.bus_addr(tx_buf.buf_ptr);

        let (index, desc) = self
            .inner
            .tx_ring
            .get_next_tx_descriptor_mut()
            .ok_or(DevError::NoMemory)?;

        desc.set_buffer_paddr(bus_addr);
        desc.basic.set_des2(tx_buf.packet_len() as u32);
        desc.basic.set_des3(
            DESC_OWN
                | TDES3::FIRST_DESCRIPTOR.bits()
                | TDES3::LAST_DESCRIPTOR.bits()
                | (tx_buf.packet_len() as u32),
        );

        self.inner.tx_ring.advance_tail();

        self.inner.update_tx_end_addr(self.inner.tx_ring.tail());

        log::trace!("Packet transmitted, TX index: {}", index);

        Ok(())
    }

    fn clear_intr_status(&mut self) -> bool {
        self.inner.read_mac_intr_status();
        self.inner.clear_dma_intr_status()
    }

    fn receive(&mut self) -> DevResult<NetBufPtr> {
        // self.inspect_dma_regs();
        // self.scan_rx_ring();
        if !self.can_receive() {
            return Err(DevError::Again);
        }

        let (head, buffer_ptr, packet_len) = self
            .inner
            .rx_ring
            .get_completed_rx_descriptor()
            .ok_or(DevError::Again)?;

        let net_buf = NetBufPtr::new(
            self.inner.rx_ring.mem_pool.base_ptr(),
            buffer_ptr,
            packet_len,
        );

        let desc = &mut self.inner.rx_ring.descriptors_mut()[head];

        let buf_ptr = self
            .inner
            .rx_ring
            .mem_pool
            .alloc()
            .ok_or(DevError::NoMemory)?;
        let bus_addr = self.inner.rx_ring.mem_pool.bus_addr(buf_ptr);

        desc.set_buffer_paddr(bus_addr);
        desc.basic.set_des2(0);
        desc.basic.set_des3(
            DESC_OWN | RDES3::BUFFER1_VALID_ADDR.bits() | RDES3::INT_ON_COMPLETION_EN.bits(),
        );

        log::trace!("RX buffer recycled, RX index: {}", head);

        self.inner.update_rx_end_addr(head);

        log::trace!(
            "Packet received, length: {}, RX index: {}",
            packet_len,
            self.inner.rx_ring.head()
        );

        Ok(net_buf)
    }

    fn recycle_rx_buffer(&mut self, rx_buf: NetBufPtr) -> DevResult {
        self.inner.rx_ring.mem_pool.free(rx_buf.buf_ptr);

        Ok(())
    }

    fn recycle_tx_buffers(&mut self) -> DevResult {
        self.inner.tx_ring.reclaim_tx_descriptors();
        Ok(())
    }

    fn alloc_tx_buffer(&mut self, size: usize) -> DevResult<NetBufPtr> {
        if size > MAX_FRAME_SIZE {
            return Err(DevError::InvalidParam);
        }

        let buf_ptr = self
            .inner
            .tx_ring
            .mem_pool
            .alloc()
            .ok_or(DevError::NoMemory)?;

        Ok(NetBufPtr::new(
            self.inner.tx_ring.mem_pool.base_ptr(),
            buf_ptr,
            size,
        ))
    }
}
