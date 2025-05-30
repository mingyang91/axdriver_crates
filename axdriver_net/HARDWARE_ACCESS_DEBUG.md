# StarFive DWMAC Hardware Accessibility Debug Guide

## 🎯 Current Status: Clocks Fixed, Hardware Access Issue

**MAJOR SUCCESS**: All clock issues have been resolved!
- ✅ PLL2 (GMAC PLL): Working
- ✅ GMAC1_AXI, GMAC1_AHB, GMAC1_PTP: All enabled  
- ✅ GTXCLK: Found working at offset 0x198

**REMAINING ISSUE**: Hardware registers still read 0x0
- ❌ MAC_VERSION register: 0x0 (should be ~0x1037xxxx)
- ❌ DMA_BUS_MODE register: 0x0 (should have default values)

## 🔍 What Our Latest Fix Addresses

### 1. **STG Power Domain Control**
```
🔋 Enabling STG power domain (critical for GMAC hardware access)
STG power status before/after logging
```

### 2. **Power Isolation Removal**  
```
🔓 Removing power isolation for GMAC hardware access
Power isolation before/after logging
```

### 3. **Bus Interconnect Reset**
```
🔄 Resetting and enabling bus interconnects  
Bus reset before/after logging
```

### 4. **Enhanced Reset Sequence**
```
🔧 Applying enhanced StarFive reset sequence
🚀 Applying hardware wake-up sequence
```

### 5. **Hardware Accessibility Test**
```
🔬 TESTING HARDWARE ACCESSIBILITY
Testing GMAC0/GMAC1 registers directly
Register write/read tests
```

## 📊 Expected Test Results

### ✅ **SUCCESS Case** (What we want to see):
```
✅ GMAC1 HARDWARE IS ACCESSIBLE! (MAC_VERSION=0x1037xxxx)
✅ GMAC1 register write/read working!
🎉 HARDWARE ACCESSIBILITY TEST PASSED!
DWMAC device initialized successfully
number of NICs: 1
```

### ❌ **Still Failing** (Need more investigation):
```
⚠️ GMAC1 hardware still not accessible (MAC_VERSION=0x0)
⚠️ GMAC1 register write/read failed  
🚨 HARDWARE STILL NOT ACCESSIBLE
Device reset timeout
number of NICs: 0
```

## 🔧 Additional Areas to Investigate (if still failing)

### 1. **Pin Muxing**
- GMAC pins may need specific muxing configuration
- Check if GMAC pins are properly routed to the controller

### 2. **External PHY Power**
- PHY chip may need separate power enable
- Check if PHY reset is properly sequenced

### 3. **Clock Domain Crossings**
- May need specific timing for clock domain crossings
- Some clocks may need phase alignment

### 4. **Hardware Errata**
- Specific StarFive JH7110 hardware workarounds
- Vendor-specific initialization sequences

### 5. **Memory Mapping**
- Physical addresses may be different than documented
- May need different memory mapping attributes

## 📝 Next Debug Steps (if still failing)

1. **Check power domain logs** for any failures
2. **Look for bus/interconnect errors** in power isolation
3. **Verify all reset sequences** complete successfully
4. **Test alternative GMAC base addresses**
5. **Check for any missing StarFive-specific controls**

The comprehensive fix should resolve the hardware accessibility issue. If not, the detailed logging will help identify what additional StarFive-specific controls are needed. 