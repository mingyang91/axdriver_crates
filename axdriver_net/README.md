# DWMAC Ethernet Driver for StarFive VisionFive 2

## Overview

This directory contains the DWMAC (DesignWare MAC) Ethernet driver implementation for the StarFive VisionFive 2 development board. The driver has been developed through comprehensive debugging of MDIO hardware issues and provides graceful degradation when hardware limitations are encountered.

## Features

- **Hardware Abstraction Layer (HAL)**: Clean separation between hardware-specific operations and driver logic
- **Graceful Degradation**: Continues to operate even when MDIO controller hardware is non-functional
- **Network Stack Integration**: Full compatibility with ArceOS network stack
- **Comprehensive Debugging**: Based on extensive hardware analysis and systematic debugging
- **Memory Safety**: Rust implementation with proper buffer management

## Architecture

### Driver Components

1. **Core Driver** (`dwmac.rs`): Main driver implementation with hardware initialization and network operations
2. **HAL Trait**: Platform abstraction for memory mapping and timing operations
3. **Integration Layer** (`modules/axdriver/src/dwmac.rs`): ArceOS-specific HAL implementation
4. **Driver Registration** (`modules/axdriver/src/drivers.rs`): Device discovery and initialization

### Hardware Support

- **Target Platform**: StarFive VisionFive 2 (JH7110 SoC)
- **Ethernet Controller**: Synopsys DesignWare MAC v5.20
- **PHY**: YT8531C Gigabit Ethernet PHY
- **Interface**: RGMII (Reduced Gigabit Media Independent Interface)

## Technical Background

### MDIO Hardware Issues

Through systematic debugging, we identified fundamental MDIO controller issues:

1. **Stuck Busy Bit**: MDIO controller's busy bit (0x00000001) never clears
2. **Hardware-Level Failure**: Issue appears to be at the silicon level
3. **Clock Dependencies**: External clock dependencies from PHY required for MDIO operation
4. **Register Accessibility**: Some MAC registers are accessible while MDIO functionality is not

### Solution Approach

The driver implements several strategies to handle the hardware limitations:

1. **Hardware Detection**: Probes MAC_VERSION register to determine hardware accessibility
2. **Software-Only Mode**: Operates without PHY configuration when MDIO is non-functional
3. **Bootloader Assumption**: Assumes PHY is pre-configured by bootloader (U-Boot)
4. **Network Layer Support**: Maintains full network stack compatibility for testing

## Usage

### Enabling the Driver

Add the DWMAC feature to your ArceOS configuration:

```toml
[features]
dwmac = ["axdriver/dwmac"]
```

### Hardware Addresses

The driver supports both GMAC instances on the StarFive VisionFive 2:

- **GMAC0**: 0x16030000 (PHY address 0)
- **GMAC1**: 0x16040000 (PHY address 1)

### Integration Example

```rust
use axdriver_net::dwmac::{DwmacHal, DwmacNic};

// Implement HAL for your platform
struct MyHal;

impl DwmacHal for MyHal {
    unsafe fn mmio_phys_to_virt(paddr: usize, size: usize) -> NonNull<u8> {
        // Platform-specific memory mapping
    }
    
    fn wait_for(duration: core::time::Duration) -> Result<(), &'static str> {
        // Platform-specific delay implementation
    }
}

// Initialize driver
let nic = DwmacNic::<MyHal>::init(0x16040000)?;
```

## Network Operations

The driver implements the standard ArceOS `NetDriverOps` trait:

- **Transmission**: `transmit()` - Send network packets
- **Reception**: `receive()` - Receive network packets  
- **Buffer Management**: `alloc_tx_buffer()`, `recycle_rx_buffer()`
- **Status Queries**: `can_transmit()`, `can_receive()`

### Current Limitations

Due to the MDIO hardware issues:

1. **PHY Configuration**: Cannot dynamically configure PHY settings
2. **Link Detection**: Cannot detect cable insertion/removal
3. **Speed Negotiation**: Cannot perform auto-negotiation
4. **Hardware TX/RX**: Actual packet transmission/reception needs completion

## Testing

The driver includes simulation capabilities for testing the network stack:

- **Simulated Packets**: Generates ARP request packets for network layer testing
- **Buffer Management**: Tests allocation and deallocation paths
- **Integration Testing**: Validates compatibility with upper network layers

## Hardware Debugging History

Our development process included:

1. **Systematic MDIO Analysis**: 8-step debugging framework
2. **Clock System Investigation**: Comprehensive clock dependency analysis
3. **Alternative Approaches**: GPIO bit-banging MDIO implementation attempts
4. **U-Boot Driver Analysis**: Integration of proven U-Boot patterns
5. **Recovery Testing**: Multiple hardware reset and recovery strategies

## Future Improvements

1. **Complete Hardware TX/RX**: Implement full DMA descriptor management
2. **Interrupt Support**: Add interrupt-driven packet processing
3. **PHY Alternative**: Implement GPIO-based MDIO when hardware permits
4. **Power Management**: Add sleep/wake functionality
5. **Statistics**: Implement network statistics collection

## Debugging Support

The driver includes comprehensive logging:

```
🚀 StarFive DWMAC driver initializing
   Base address: 0x16040000
🔧 Starting hardware initialization
   🔍 Checking hardware accessibility...
   MAC_VERSION: 0x5020
   ✅ Hardware is accessible (MAC_VERSION=0x5020)
✅ StarFive DWMAC driver initialized
   Hardware accessible: true
   PHY detected: true
   MAC address: 6c:cf:39:00:5d:34
```

## Contributing

When contributing to this driver:

1. Maintain the HAL abstraction for portability
2. Add comprehensive logging for debugging
3. Test with both GMAC0 and GMAC1 instances
4. Document any hardware-specific workarounds
5. Follow Rust safety guidelines for hardware access

## License

This driver is part of the ArceOS project and follows the same license terms. 

# DWMAC Ethernet Driver Tutorial

This directory contains a **simplified DWMAC ethernet driver** designed for educational purposes. It demonstrates the core concepts of ethernet driver development without production-level complexity.

## 🎯 Tutorial Goals

This tutorial teaches:
- **Ethernet driver architecture** - How network drivers work
- **DMA descriptor rings** - TX/RX buffer management
- **Hardware abstraction** - Platform-independent driver design
- **MDIO communication** - PHY management basics
- **Register programming** - Hardware configuration

## 📁 File Structure

```
src/
├── dwmac.rs          # Core DWMAC driver (tutorial version)
├── lib.rs            # Library exports
└── README.md         # This file
```

## 🔧 Key Components

### 1. **DwmacHal Trait** - Hardware Abstraction Layer
```rust
pub trait DwmacHal: Send + Sync {
    fn dma_alloc(size: usize) -> (PhysAddr, NonNull<u8>);
    fn wait_until(duration: core::time::Duration) -> Result<(), &'static str>;
    fn configure_platform() -> Result<(), &'static str>;
    // ... other HAL functions
}
```

### 2. **DwmacNic Struct** - Main Driver
```rust
pub struct DwmacNic<H: DwmacHal> {
    base_addr: NonNull<u8>,
    mac_addr: [u8; 6],
    
    // TX ring
    tx_descriptors: NonNull<DmaDescriptor>,
    tx_buffers: [Option<NetBufPtr>; TX_DESC_COUNT],
    
    // RX ring  
    rx_descriptors: NonNull<DmaDescriptor>,
    rx_buffers: [Option<NetBufPtr>; RX_DESC_COUNT],
}
```

### 3. **DMA Descriptors** - Hardware Interface
```rust
#[repr(C, align(16))]
struct DmaDescriptor {
    status: u32,    // Ownership and status flags
    control: u32,   // Buffer size and control
    buffer1: u32,   // Physical buffer address
    buffer2: u32,   // Next descriptor or second buffer
}
```

## 🚀 Initialization Flow

1. **Platform Setup** - Configure clocks, resets, pins
2. **Hardware Reset** - Apply software reset via DMA_BUS_MODE
3. **Descriptor Rings** - Allocate and setup TX/RX rings
4. **MAC Configuration** - Set speed, duplex, filtering
5. **DMA Configuration** - Enable store-and-forward mode
6. **PHY Initialization** - Basic MDIO communication
7. **Start Operations** - Enable TX/RX DMA

## 📡 MDIO Communication

Simple PHY management:
```rust
// Read PHY register
let bmsr = self.mdio_read(phy_addr, 1)?;

// Write PHY register  
self.mdio_write(phy_addr, 0, 0x1200)?; // Enable auto-negotiation
```

## 🔄 Packet Flow

### **Transmit Path:**
1. Check if TX ring has space (`can_transmit()`)
2. Setup DMA descriptor with packet buffer
3. Set `DESC_OWN` bit to give ownership to hardware
4. Trigger transmission via `DMA_TX_POLL_DEMAND`

### **Receive Path:**
1. Check if RX descriptor is ready (`can_receive()`)
2. Hardware fills buffer and clears `DESC_OWN` bit
3. Extract packet length from descriptor status
4. Return buffer to software, allocate new RX buffer

## 🎓 Learning Outcomes

After studying this driver, you'll understand:

- **Ring Buffer Management** - How circular DMA rings work
- **Memory Coherency** - DMA-coherent memory allocation
- **Hardware Synchronization** - Ownership bits and polling
- **Network Stack Integration** - Driver trait implementation
- **Error Handling** - Timeout and error recovery

## 🔍 What's Simplified

This tutorial version omits:
- ❌ **QoS features** - Advanced traffic management
- ❌ **Power management** - Sleep/wake functionality  
- ❌ **Advanced PHY features** - EEE, WoL, etc.
- ❌ **Interrupt handling** - Uses polling instead
- ❌ **Performance optimization** - Focus on clarity
- ❌ **Production error handling** - Simplified error paths

## 🛠️ Platform Integration

The driver uses a HAL (Hardware Abstraction Layer) pattern:

```rust
// Platform-specific implementation
impl DwmacHal for DwmacHalImpl {
    fn configure_platform() -> Result<(), &'static str> {
        // Configure clocks, resets, pin mux
        Ok(())
    }
}

// Generic driver
let nic = DwmacNic::<DwmacHalImpl>::init(base_addr, size)?;
```

## 📚 Next Steps

To extend this tutorial:
1. **Add interrupt support** - Replace polling with IRQ handling
2. **Implement advanced PHY features** - Link detection, speed negotiation
3. **Add power management** - Suspend/resume functionality
4. **Optimize performance** - Batch processing, NAPI-style polling
5. **Add platform-specific features** - Clock management, pin configuration

## 🎉 Success Criteria

A working implementation should:
- ✅ **Initialize hardware** - Reset and configure DWMAC
- ✅ **Detect PHY** - Basic MDIO communication
- ✅ **Transmit packets** - Send ethernet frames
- ✅ **Receive packets** - Process incoming frames
- ✅ **Handle errors gracefully** - Timeout and recovery

This tutorial provides a solid foundation for understanding ethernet driver development! 

```
# uEnv.txt
loadkernel=fatload mmc 1:3 0x40200000 arceos-starfive.bin
bootcmd=run loadkernel; go 0x40200000
```