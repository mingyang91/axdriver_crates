extern crate alloc;

use core::{cell::RefCell, marker::PhantomData, ptr::NonNull};

use alloc::collections::VecDeque;

use crate::dwmac::DwmacHal;

pub struct MemPool<H: DwmacHal, const N: usize, const PS: usize> {
    bus_addr: usize,
    ptr: NonNull<u8>,
    free: RefCell<VecDeque<NonNull<u8>>>,
    _marker: PhantomData<H>,
}

impl<H: DwmacHal, const N: usize, const PS: usize> MemPool<H, N, PS> {
    pub fn new() -> Self {
        let (bus_addr, ptr) = H::dma_alloc(PS * N, 64);
        let mut free_buffers = VecDeque::new();
        for i in 0..N {
            free_buffers.push_back(unsafe { ptr.add(i * PS) });
        }
        Self {
            bus_addr,
            ptr,
            free: RefCell::new(free_buffers),
            _marker: PhantomData,
        }
    }

    pub fn base_ptr(&self) -> NonNull<u8> {
        self.ptr
    }

    pub fn alloc(&self) -> Option<NonNull<u8>> {
        self.free.borrow_mut().pop_front()
    }

    pub fn free(&self, ptr: NonNull<u8>) {
        self.free.borrow_mut().push_back(ptr);
    }

    pub fn bus_addr(&self, ptr: NonNull<u8>) -> usize {
        self.bus_addr + (ptr.as_ptr() as usize - self.ptr.as_ptr() as usize)
    }

    pub fn bus_addr_to_ptr(&self, bus_addr: usize) -> NonNull<u8> {
        unsafe { NonNull::new_unchecked(self.ptr.as_ptr().add(bus_addr - self.bus_addr)) }
    }
}
