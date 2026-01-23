# Force Plate Calibration System - Technical Deep Dive

This document explains the calibration architecture for the ADS1256-based force plate system, including tare, calibration slopes, precision scaling, and geometry-based corrections.

---

## Table of Contents

1. [System Overview](#system-overview)
2. [The Data Pipeline](#the-data-pipeline)
3. [Tare (Zero Offset)](#tare-zero-offset)
4. [Polarity Correction](#polarity-correction)
5. [Calibration Slopes](#calibration-slopes)
6. [Precision and Scaling](#precision-and-scaling)
7. [Geometry-Based Calibration](#geometry-based-calibration)
8. [Mathematical Formulas](#mathematical-formulas)
9. [Calibration Workflow](#calibration-workflow)
10. [Troubleshooting](#troubleshooting)

---

## System Overview

### Hardware Components

```
Load Cell → ADS1256 ADC → Teensy 4.1 → BLE → ESP32 → GUI
   ↓           ↓             ↓
 Analog    24-bit raw    Calibrated
 Signal    counts        10g units
```

### The Challenge

Raw ADC readings are meaningless numbers (e.g., `8,388,608`). We need to convert them to meaningful weight units while:

1. Handling the **zero offset** (load cell reads non-zero with no load)
2. Correcting for **polarity** (some cells wired in reverse)
3. Applying individual **sensitivity slopes** (each cell has different gain)
4. Fitting values into **16-bit integers** for efficient BLE transmission
5. Ensuring the **sum is accurate** regardless of weight position

---

## The Data Pipeline

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        DATA TRANSFORMATION PIPELINE                      │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  RAW ADC      FILTERED      TARE           POLARITY      CALIBRATED     │
│  (24-bit) →   (noise     →  (subtract  →   (multiply  →  (multiply      │
│              reduction)     offset)        by ±1)        by slope)      │
│                                                                          │
│  Example for 15 kg on one load cell:                                     │
│                                                                          │
│  8,500,000 →  8,498,234  →  112,345    →   112,345    →  1500           │
│  counts       counts        counts         counts        (10g units)    │
│                             (after         (positive     = 15.00 kg     │
│                             -8,385,889)    polarity)                    │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## Tare (Zero Offset)

### What is Tare?

Tare compensates for the fact that load cells output a non-zero signal even with no load applied. This is due to:

- Internal strain gauge bias
- Mounting stress
- Temperature drift
- Electronic offset

### How Tare Works

```cpp
// During CAL_TARE (no load on plate):
g_cal.offsets[ch] = current_reading;  // Store the "zero" point

// During streaming:
tared_value = raw_reading - g_cal.offsets[ch];
```

### Example

```
Load Cell 1 with NO LOAD:
  Raw ADC reading:     8,385,889 counts
  Stored offset:       8,385,889 counts
  
Load Cell 1 with 10 kg:
  Raw ADC reading:     8,498,234 counts
  After tare:          8,498,234 - 8,385,889 = 112,345 counts
```

### Key Points

- **CAL_TARE must be done with EMPTY plate**
- Offsets are stored in EEPROM (persist across power cycles)
- Each channel has its own independent offset
- Temperature changes may require re-tare

---

## Polarity Correction

### The Problem

Load cells have differential outputs (SIG+ and SIG-). If wired in reverse:

```
Normal wiring:    Force applied → Positive count change
Reverse wiring:   Force applied → Negative count change
```

### Automatic Detection

During `CAL_ADD`, the system detects polarity automatically:

```cpp
double delta = current_mean - stored_offset;

if (delta < 0) {
    g_cal.polarity[ch] = -1;  // Inverted
    Serial.println("LC polarity auto-detected: INVERTED");
} else {
    g_cal.polarity[ch] = +1;  // Normal
    Serial.println("LC polarity auto-detected: NORMAL");
}
```

### Application

```cpp
// During streaming:
corrected_value = tared_value * g_cal.polarity[ch];
// Now all channels produce positive values for positive force
```

---

## Calibration Slopes

### What is a Slope?

The slope converts ADC counts to physical units:

```
weight_in_10g_units = counts × slope
```

Where:
- `counts` = tare-corrected, polarity-corrected ADC reading
- `slope` = calibration coefficient (10g per count)

### Why Per-Channel Slopes?

Each load cell has slightly different sensitivity due to:

- Manufacturing variations
- Mounting conditions
- Wire resistance
- ADC channel gain differences

### Slope Calculation

For a known weight `W` applied:

```
slope = W_in_10g_units / delta_counts
```

Example:
```
Known weight:     15 kg = 1500 (10g units)
Delta counts:     112,345
Slope:            1500 / 112,345 = 0.01335 (10g per count)
```

---

## Precision and Scaling

### The int16 Constraint

BLE packets use `int16` (16-bit signed integer) for efficiency:

```
int16 range: -32,768 to +32,767
```

### Why 10g Units?

We need to represent weights up to ~100 kg per channel while fitting in int16:

| Unit System | 100 kg Value | Fits in int16? |
|-------------|--------------|----------------|
| Grams       | 100,000      | ❌ No (overflow) |
| 10g units   | 10,000       | ✅ Yes |
| 100g units  | 1,000        | ✅ Yes (less precision) |

**10g units** provide a good balance:
- Max value: 327.67 kg per channel ✅
- Resolution: 10 grams (0.01 kg) ✅

### Conversion Table

| 10g Units | Kilograms | Grams |
|-----------|-----------|-------|
| 1         | 0.01 kg   | 10 g  |
| 100       | 1.00 kg   | 1000 g |
| 1500      | 15.00 kg  | 15000 g |
| 6000      | 60.00 kg  | 60000 g |
| 32767     | 327.67 kg | 327670 g |

### In the GUI

```python
# Convert 10g units to kg for display:
kg = value_10g / 100.0
```

---

## Geometry-Based Calibration

### The Problem: Sum Changes with Position

With simple calibration, moving weight on the plate changes the sum:

```
Weight at CENTER:     Sum = 60.0 kg ✓
Weight near LC1:      Sum = 58.5 kg ✗
Weight near LC3:      Sum = 61.2 kg ✗
```

This happens because individual load cells have different sensitivities.

### The Solution: Equal Distribution Assumption

When weight is placed at the **exact center** of a rectangular plate with 4 corner load cells, physics dictates that each cell carries **exactly 25%** of the total weight.

```
┌─────────────────────────────────────┐
│                                     │
│   LC1 (25%)  ←───22cm───→  LC2 (25%)│
│       ↑                       ↑     │
│       │                       │     │
│      35cm       CENTER       35cm   │
│       │          ⬤           │     │
│       ↓         (weight)      ↓     │
│   LC4 (25%)  ←───22cm───→  LC3 (25%)│
│                                     │
└─────────────────────────────────────┘
```

### Old Method (Proportional - WRONG)

```cpp
// Distributes weight based on counts (all cells get SAME slope)
y10g_ch = total_y10g * (delta[ch] / sum_delta);

// Results:
// slope1 = slope2 = slope3 = slope4 = total_weight / sum_counts
```

**Problem**: If LC1 is 10% more sensitive than LC2, this isn't accounted for.

### New Method (Geometry-Based - CORRECT)

```cpp
// Each cell carries 25% of total weight (geometry assumption)
weight_per_cell = total_y10g / 4;  // = 0.25 × total

// Each cell gets its OWN slope:
// slope1 = weight_per_cell / counts1
// slope2 = weight_per_cell / counts2
// etc.
```

### Example Calculation

Calibrating with 60 kg at center:

| Load Cell | Counts | Weight Share | Slope |
|-----------|--------|--------------|-------|
| LC1 | 100,000 | 15 kg (1500) | 1500/100000 = **0.01500** |
| LC2 | 85,000 | 15 kg (1500) | 1500/85000 = **0.01765** |
| LC3 | 92,000 | 15 kg (1500) | 1500/92000 = **0.01630** |
| LC4 | 88,000 | 15 kg (1500) | 1500/88000 = **0.01705** |

**Notice**: Each cell has a **different** slope that compensates for its individual sensitivity.

### Why This Works

When weight moves on the plate:

```
Weight shifts toward LC1:
  - LC1 counts increase (e.g., +20%)
  - LC2 counts decrease (e.g., -10%)
  - LC3 counts decrease (e.g., -5%)
  - LC4 counts decrease (e.g., -5%)
  
With individual slopes:
  - LC1 contribution: (counts × slope1) = slightly higher
  - LC2 contribution: (counts × slope2) = slightly lower
  - BUT the slopes are tuned so the SUM remains constant!
```

---

## Mathematical Formulas

### Complete Transformation

For each channel `ch`:

```
calibrated_value = |raw - offset| × polarity × slope
```

Where:
- `raw` = filtered ADC reading (24-bit signed)
- `offset` = stored tare value (from CAL_TARE)
- `polarity` = +1 or -1 (from auto-detection)
- `slope` = 10g per count (from CAL_ADD)

### Total Force

```
F_total = Σ calibrated_value[ch]  for ch = 0 to 3
        = (raw0 - off0) × pol0 × slope0
        + (raw1 - off1) × pol1 × slope1
        + (raw2 - off2) × pol2 × slope2
        + (raw3 - off3) × pol3 × slope3
```

### Center of Pressure (CoP)

Given calibrated forces F1, F2, F3, F4 at corners:

```
           LC1 ─────────── LC2
            │               │
            │    (CoPx,     │
            │     CoPy)     │
            │       ⬤       │
           LC4 ─────────── LC3

CoP_X = (Width/2) × [(F2 + F3) - (F1 + F4)] / F_total

CoP_Y = (Length/2) × [(F1 + F2) - (F4 + F3)] / F_total
```

Origin (0,0) is at plate center.

---

## Calibration Workflow

### Step-by-Step Procedure

```
┌────────────────────────────────────────────────────────────────────┐
│                     CALIBRATION PROCEDURE                           │
├────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  1. CAL_CLEAR                                                       │
│     └─ Erase all stored calibration data                           │
│                                                                     │
│  2. CAL_TARE  (plate EMPTY)                                        │
│     └─ Capture zero-offset for all channels                        │
│     └─ Store in g_cal.offsets[]                                    │
│                                                                     │
│  3. CAL_ADD_60  (place 60 kg at CENTER)                            │
│     └─ Capture loaded readings                                     │
│     └─ Auto-detect polarity                                        │
│     └─ Calculate individual slopes (25% each)                      │
│     └─ Store in g_cal.ch_a_10g_per_count[]                         │
│                                                                     │
│  4. CAL_STATUS                                                      │
│     └─ Verify different slopes per channel                         │
│     └─ Check slope spread percentage                               │
│                                                                     │
│  5. START                                                           │
│     └─ Begin streaming calibrated values                           │
│     └─ Test by moving weight - sum should stay constant            │
│                                                                     │
└────────────────────────────────────────────────────────────────────┘
```

### Verification

After calibration, check with `CAL_STATUS`:

```
[CAL] ===== Calibration v4 (geometry-based) =====
[CAL] Geometry: 22cm x 35cm plate, 4 corner load cells
[CAL] Each cell carries 25% of total weight when centered

[CAL] LC1: offset=8385889, slope=0.015000000, pts=1, pol=NRM
[CAL] LC2: offset=8401234, slope=0.017647059, pts=1, pol=NRM
[CAL] LC3: offset=8378456, slope=0.016304348, pts=1, pol=INV
[CAL] LC4: offset=8390123, slope=0.017045455, pts=1, pol=NRM

[CAL] Slope stats: avg=0.016499216, min=0.015000000, max=0.017647059
[CAL] Slope spread: 16.05% (individual cell sensitivity differences)
```

**Key indicators**:
- Each channel should have a **different slope**
- Slope spread of 5-20% is normal
- Polarities should match wiring

---

## Troubleshooting

### Problem: Values don't zero after tare

**Symptoms**: Channels show ~32,000 instead of ~0 with no load

**Cause**: Tare offset not being applied

**Solution**: Check `get_filtered_load_cell_reading()` subtracts offset

### Problem: Some channels go negative when force applied

**Symptoms**: Force increases but value decreases (goes negative)

**Cause**: Polarity not detected or not applied

**Solution**: 
1. Ensure CAL_ADD is run with weight
2. Check polarity detection in CAL_ADD
3. Verify polarity multiplication in streaming

### Problem: Sum changes when moving on plate

**Symptoms**: 60 kg shows as 58-62 kg depending on position

**Cause**: Using same slope for all channels

**Solution**: Use geometry-based calibration (25% per cell)

### Problem: Values clamp to 32767

**Symptoms**: Stepping on plate maxes out all channels

**Cause**: Slope too high or wrong scaling

**Solution**: 
1. Re-calibrate with known weight
2. Check slope values are reasonable (0.001-0.1 range)
3. Verify using 10g units not grams

### Problem: Noisy readings

**Symptoms**: Values fluctuate ±50-100 units constantly

**Cause**: Insufficient filtering or electrical noise

**Solution**:
1. Increase filter strength (EMA alpha, median window)
2. Check grounding and shielding
3. Use averaging during calibration capture

---

## Data Structures

### CalBlob (stored in EEPROM)

```cpp
struct CalBlob {
    uint16_t    version;                              // CAL_VERSION
    int32_t     offsets[4];                           // Tare values
    float       ch_a_10g_per_count[4];                // Slopes
    int8_t      polarity[4];                          // +1 or -1
    CalPoint    points_ch[4][CAL_MAX_POINTS];         // Raw data points
    uint8_t     points_ch_n[4];                       // Point counts
    uint32_t    checksum;                             // CRC32
};
```

### Data Flow Diagram

```
EEPROM
  │
  ├─→ offsets[]     ─→ Subtracted from raw in streaming
  ├─→ polarity[]    ─→ Multiplied after tare
  └─→ slopes[]      ─→ Multiplied after polarity
                          │
                          ↓
                    Calibrated int16 value
                          │
                          ↓
                    BLE packet (16 bytes per sample)
                          │
                          ↓
                    GUI displays in kg (value/100)
```

---

## Summary

| Component | Purpose | Storage | Applied When |
|-----------|---------|---------|--------------|
| **Tare Offset** | Zero compensation | EEPROM | Streaming |
| **Polarity** | Sign correction | EEPROM | Streaming |
| **Slope** | Counts → 10g units | EEPROM | Streaming |
| **10g Units** | Fit in int16 | N/A | Transmission |
| **Geometry** | Equal distribution | Calibration | CAL_ADD only |

The key insight is that **geometry-based calibration** assumes equal weight distribution at center, which allows computing individual slopes that keep the total sum accurate regardless of where force is applied on the plate.

---

*Document Version: 1.0*  
*Last Updated: January 2026*
