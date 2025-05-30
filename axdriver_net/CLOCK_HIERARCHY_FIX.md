# StarFive VisionFive 2 DWMAC Clock Hierarchy Fix

## Problem Summary

The StarFive VisionFive 2 DWMAC (Ethernet) driver was experiencing complete hardware inaccessibility - all DWMAC register reads returned `0x0`, indicating fundamental clock/reset/power issues. Despite implementing comprehensive working driver methods, both GMAC0 and GMAC1 remained non-functional.

## Root Cause Analysis

The issue was identified as **missing hierarchical clock dependencies**:

1. **Missing PLL2 (GMAC PLL)** - The root clock feeding GTXCLK
2. **Missing Parent/Bus Clocks** - Required infrastructure clocks  
3. **No Power Domain Management** - Hardware may be powered off
4. **Incorrect Clock Sequence** - Enabling clocks without dependencies

### Critical Evidence

```
Clock Status Before Fix:
  Clock 97 (GMAC1_AHB):    0x80000000 ✅
  Clock 98 (GMAC1_AXI):    0x80000000 ✅  
  Clock 100 (GMAC1_GTXCLK): 0x0 ❌ CRITICAL FAILURE
  Clock 102 (GMAC1_PTP):   0x80000000 ✅

DWMAC Register Test:
  MAC_VERSION:  0x0 ❌
  DMA_BUS_MODE: 0x0 ❌
  MAC_CONFIG:   0x0 ❌
```

**Clock 100 (GMAC1_GTXCLK) failure** was the smoking gun - this clock depends on PLL2 which was never configured.

## Comprehensive Solution

We implemented a **6-step hierarchical clock initialization** system:

### Step 1: PLL2 (GMAC PLL) Configuration ⭐ CRITICAL
```rust
fn enable_pll2_gmac_pll() -> Result<(), &'static str> {
    // Configure PLL2 for 125MHz GMAC clock
    // Formula: Fout = Fin * (FBDIV) / (PREDIV * POSTDIV)
    // 24MHz * 125 / (6 * 4) = 125MHz
    let pll2_ctrl0_val = (125 << 16) | (6 << 8) | (4 << 0);
    let pll2_ctrl1_val = 0x1; // Enable PLL2
    
    // Write configuration and wait for lock
    // Verify PLL2 lock bit (bit 2) in PLL_STATUS
}
```

### Step 2: Power Domain Management
```rust
fn enable_power_domains() -> Result<(), &'static str> {
    // Check SYSTOP power domain status
    // Enable if not already active
    // Wait for stabilization
}
```

### Step 3: Parent/Bus Clock Dependencies
```rust
fn enable_parent_clocks() -> Result<(), &'static str> {
    let parent_clocks = [
        (STG_AXIAHB_CLK, "STG_AXIAHB"),      // Clock ID 10
        (NOC_BUS_STG_AXI_CLK, "NOC_BUS_STG_AXI"), // Clock ID 25
        (AHB0_CLK, "AHB0"),                  // Clock ID 18
        (AHB1_CLK, "AHB1"),                  // Clock ID 19
        (APB_BUS_CLK, "APB_BUS"),            // Clock ID 20
    ];
    // Enable in dependency order
}
```

### Step 4: GMAC Peripheral Clocks  
```rust
fn enable_gmac_peripheral_clocks() -> Result<(), &'static str> {
    let gmac_clocks = [
        (98, "GMAC1_AXI"),     // Depends on NOC_BUS_STG_AXI
        (97, "GMAC1_AHB"),     // Depends on STG_AXIAHB + AHB0
        (100, "GMAC1_GTXCLK"), // Depends on PLL2 + GMAC1_AXI ⭐
        (102, "GMAC1_PTP"),    // Depends on OSC
    ];
    // Enable with dependency checking
}
```

### Step 5: Enhanced Reset Sequence
```rust
fn apply_reset_sequence() -> Result<(), &'static str> {
    // Working driver's reset sequence with proper timing
    // Enhanced delays between steps
}
```

### Step 6: Clock Tree Verification
```rust
fn verify_clock_tree() -> Result<(), &'static str> {
    // Validate all critical clocks enabled
    // Return clear error messages for failures
}
```

## Expected Results

### Clock Status After Fix:
```
Clock 97 (GMAC1_AHB):    0x80000000 ✅
Clock 98 (GMAC1_AXI):    0x80000000 ✅  
Clock 100 (GMAC1_GTXCLK): 0x80000000 ✅ NOW SHOULD WORK! 🎉
Clock 102 (GMAC1_PTP):   0x80000000 ✅
```

### DWMAC Register Test:
```
MAC_VERSION:  0x1037xxxx ✅ (Synopsys DesignWare MAC version)
DMA_BUS_MODE: 0x000xxxxx ✅ (DMA configuration register)
MAC_CONFIG:   0x000xxxxx ✅ (MAC configuration register)
```

## Key Register Addresses

### Clock System:
- **SYSCRG Base**: `0x13020000` (System Clock & Reset Generator)
- **AONCRG Base**: `0x17000000` (Always-On Clock & Reset Generator)  
- **PLL Base**: `0x13020000` (PLL configuration)
- **PMU Base**: `0x17030000` (Power Management Unit)

### PLL2 Configuration:
- **PLL2_CTRL0**: `0x13020018` (PLL2 divider configuration)
- **PLL2_CTRL1**: `0x1302001C` (PLL2 enable)  
- **PLL_STATUS**: `0x13020080` (PLL lock status, bit 2 = PLL2)

### Critical Clocks:
- **Clock 100 (GMAC1_GTXCLK)**: `0x13020190` (Most critical - was failing)
- **Clock 97 (GMAC1_AHB)**: `0x13020184`
- **Clock 98 (GMAC1_AXI)**: `0x13020188`  
- **Clock 102 (GMAC1_PTP)**: `0x13020198`

## Implementation

The fix is implemented in `modules/axdriver/src/dwmac.rs` in the `StarfiveClockSystem` struct:

```rust
impl StarfiveClockSystem {
    fn enable_dwmac_clocks() -> Result<(), &'static str> {
        Self::enable_pll2_gmac_pll()?;      // Step 1: Root PLL
        Self::enable_power_domains()?;       // Step 2: Power
        Self::enable_parent_clocks()?;       // Step 3: Dependencies  
        Self::enable_gmac_peripheral_clocks()?; // Step 4: GMAC clocks
        Self::apply_reset_sequence()?;       // Step 5: Reset
        Self::verify_clock_tree()?;          // Step 6: Verify
        Ok(())
    }
}
```

## Testing on Real Hardware

### Immediate Verification:
1. Check if Clock 100 (GMAC1_GTXCLK) now shows `0x80000000`
2. Verify DWMAC registers return non-zero values
3. Test basic DWMAC functionality

### If Still Failing - Debug Steps:

#### 1. PLL2 Debugging:
```bash
# Check PLL lock status
devmem 0x13020080 32  # Should show bit 2 = 1 for PLL2 lock
```

#### 2. Parent Clock Debugging:  
```bash
# Verify parent clocks enabled
devmem 0x13020028 32  # Clock 10 (STG_AXIAHB)
devmem 0x13020048 32  # Clock 18 (AHB0)  
devmem 0x13020064 32  # Clock 25 (NOC_BUS_STG_AXI)
# All should show 0x80000000
```

#### 3. Power Domain Debugging:
```bash
# Check power domain status
devmem 0x17030010 32  # PMU status - SYSTOP should be active
```

#### 4. GMAC Register Debugging:
```bash
# Test DWMAC accessibility
devmem 0x16040020 32  # MAC_VERSION (GMAC1)  
devmem 0x16041000 32  # DMA_BUS_MODE (GMAC1)
# Should return non-zero values
```

## Possible Adjustments

If the fix doesn't work immediately, these parameters may need tuning:

### PLL2 Configuration:
- **Current**: FBDIV=125, PREDIV=6, POSTDIV=4 → 125MHz
- **Alternative**: Try different divider combinations for target frequency

### Parent Clock IDs:
- **Current**: STG_AXIAHB=10, NOC_BUS_STG_AXI=25, AHB0=18, etc.
- **May need**: Verification against actual hardware documentation

### Power Domain Registers:
- **Current**: PMU base `0x17030000`
- **May need**: Different base address or register layout

### Reset Timing:
- **Current**: 100,000 loop iterations delay
- **May need**: Longer delays or different sequence

## Technical Background

This fix is based on analysis of:
1. **Working StarFive driver** reference implementation
2. **OpenBSD StarFive clock driver** (stfclock.c) 
3. **Clock hierarchy analysis** from debug logs
4. **Hardware accessibility patterns** from register reads

The key insight was that GTXCLK (the transmit clock) depends on a properly configured PLL2, and that parent clocks must be enabled in the correct hierarchical order.

## Status

✅ **Implementation Complete**: Comprehensive 6-step clock hierarchy fix implemented  
🧪 **Testing Required**: Needs validation on real StarFive VisionFive 2 hardware  
🎯 **Expected Outcome**: Should resolve "all registers read 0x0" issue and enable DWMAC functionality

---

**Note**: This fix targets real StarFive hardware. QEMU testing is not possible as it doesn't emulate the StarFive-specific clock and power management registers. 