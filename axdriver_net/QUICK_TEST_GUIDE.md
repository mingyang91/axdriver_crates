# Quick Test Guide - StarFive DWMAC Fix Validation

## 🎯 Goal: Verify Clock 100 (GMAC1_GTXCLK) now works

### ⚡ Quick Validation (30 seconds)

**Boot ArceOS and look for:**
```
✅ SUCCESS: 
GMAC1_GTXCLK enabled successfully: 0x80000000
DWMAC device initialized successfully

❌ FAILURE:
GMAC1_GTXCLK failed to enable (parent dependency issue?)
Failed to initialize DWMAC device
```

### 🔍 Manual Register Check
```bash
# Critical clock that was failing
devmem 0x13020190 32  # Clock 100 (GMAC1_GTXCLK)
# Expected: 0x80000000 (was 0x0 before)

# DWMAC hardware accessibility  
devmem 0x16040020 32  # MAC_VERSION (GMAC1)
# Expected: 0x1037xxxx (was 0x0 before)
```

### 📊 Success Criteria

| Test         | Before Fix | After Fix      | Status        |
| ------------ | ---------- | -------------- | ------------- |
| Clock 100    | `0x0` ❌    | `0x80000000` ✅ | **PRIMARY**   |
| MAC_VERSION  | `0x0` ❌    | `0x1037xxxx` ✅ | **SECONDARY** |
| Network Init | Panic ❌    | Success ✅      | **ULTIMATE**  |

### 🚨 If Still Failing

**Check PLL2 Status:**
```bash
devmem 0x13020080 32  # Should show lock bits set
```

**Check Parent Clocks:**
```bash
devmem 0x13020028 32  # STG_AXIAHB (should be 0x80000000)
devmem 0x13020064 32  # NOC_BUS_STG_AXI (should be 0x80000000)
```

**Boot Logs to Check:**
- Look for "PLL2 lock status: bit2=X, bit31=Y, bit1=Z"
- Any errors in STEP 1-6 indicate specific issues

---
**Expected Result**: Clock 100 finally shows `0x80000000` and DWMAC becomes accessible! 🎉 