/*
 * BQ25792 Battery Management System - Standalone Test Code
 * 
 * This code tests:
 * - I2C communication with BQ25792
 * - VBAT reading (voltage)
 * - IBAT reading (current)
 * - Charging detection
 * - SOC calculation
 * 
 * Upload this to either ESP32 (rx_radio or spi_slave) to test battery functionality
 */

#include <Wire.h>

// ============================================================================
// CONFIGURATION
// ============================================================================
static const uint8_t BQ_ADDR = 0x6B;
static const int I2C_SDA = 8;
static const int I2C_SCL = 9;
static const uint32_t I2C_HZ = 400000;

// VBAT calibration (adjust if needed)
static const float VBAT_GAIN     = 1.0000f;
static const float VBAT_OFFSET_V = -0.111f;

// ============================================================================
// BQ25792 REGISTER MAP
// ============================================================================
static const uint8_t REG00_VSYSMIN  = 0x00;  // Minimum System Voltage (8-bit)
static const uint8_t REG01_VREG     = 0x01;  // Charge Voltage Limit (16-bit, LSB-first)
static const uint8_t REG03_ICHG     = 0x03;  // Charge Current Limit (16-bit, LSB-first)
static const uint8_t REG06_IINDPM   = 0x06;  // Input Current Limit (16-bit, LSB-first)
static const uint8_t REG09_ITERM    = 0x09;  // Termination Control (ITERM)
static const uint8_t REG0A_CELL     = 0x0A;  // Recharge / Cell config (CELL[1:0])
static const uint8_t REG0F_CTRL0    = 0x0F;  // Charger Control 0 (EN_CHG bit5)
static const uint8_t REG10_CTRL1    = 0x10;  // Charger Control 1 (WATCHDOG bits[2:0])
static const uint8_t REG14_CTRL5    = 0x14;  // Charger Control 5 (EN_IBAT bit5)
static const uint8_t REG2E_ADC_CTRL = 0x2E;  // ADC Control (ADC_EN bit7, ADC_RATE bit6)
static const uint8_t REG2F_ADC_DIS0  = 0x2F;  // ADC Function Disable 0
static const uint8_t REG3B_VBAT_ADC = 0x3B;  // VBAT ADC (16-bit, MSB-first, mV)
static const uint8_t REG32_IBAT     = 0x32;  // IBAT ADC (16-bit, LSB-first, signed, mA) - Try this first
static const uint8_t REG33_IBAT     = 0x33;  // IBAT ADC (16-bit, LSB-first, signed, mA) - BQ25792 spec

// ============================================================================
// I2C HELPER FUNCTIONS
// ============================================================================

static bool bqProbe() {
  Wire.beginTransmission(BQ_ADDR);
  return (Wire.endTransmission() == 0);
}

static bool bqWrite8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(BQ_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

static bool bqRead8(uint8_t reg, uint8_t &val) {
  Wire.beginTransmission(BQ_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)BQ_ADDR, (uint8_t)1) != 1) return false;
  val = Wire.read();
  return true;
}

static bool bqRead16_MSB(uint8_t reg, uint16_t &val) {
  Wire.beginTransmission(BQ_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)BQ_ADDR, (uint8_t)2) != 2) return false;
  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  val = ((uint16_t)msb << 8) | (uint16_t)lsb;
  return true;
}

static bool bqRead16_LSB(uint8_t reg, uint16_t &val) {
  Wire.beginTransmission(BQ_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)BQ_ADDR, (uint8_t)2) != 2) return false;
  uint8_t lsb = Wire.read();
  uint8_t msb = Wire.read();
  val = ((uint16_t)lsb | ((uint16_t)msb << 8));
  return true;
}

static bool bqWrite16_LSB(uint8_t reg, uint16_t val) {
  Wire.beginTransmission(BQ_ADDR);
  Wire.write(reg);
  Wire.write((uint8_t)(val & 0xFF));        // LSB first
  Wire.write((uint8_t)((val >> 8) & 0xFF)); // MSB second
  return (Wire.endTransmission() == 0);
}

// ============================================================================
// BQ25792 INITIALIZATION
// ============================================================================

static bool bqDisableWatchdog() {
  uint8_t r10 = 0;
  if (!bqRead8(REG10_CTRL1, r10)) return false;
  r10 &= ~0x07;  // WD=0
  return bqWrite8(REG10_CTRL1, r10);
}

static bool bqEnableIBAT() {
  uint8_t r14 = 0;
  if (!bqRead8(REG14_CTRL5, r14)) {
    Serial.println("[INIT] ⚠ Failed to read REG14_CTRL5");
    return false;
  }
  uint8_t r14_before = r14;
  r14 |= (1 << 5);  // EN_IBAT = 1
  if (!bqWrite8(REG14_CTRL5, r14)) {
    Serial.println("[INIT] ⚠ Failed to write REG14_CTRL5");
    return false;
  }
  
  // Verify write
  uint8_t r14_verify = 0;
  if (bqRead8(REG14_CTRL5, r14_verify)) {
    if ((r14_verify & (1 << 5)) == 0) {
      Serial.printf("[INIT] ⚠ EN_IBAT not set! REG14: 0x%02X (was 0x%02X)\n", r14_verify, r14_before);
      return false;
    }
    Serial.printf("[INIT] ✓ EN_IBAT enabled (REG14: 0x%02X)\n", r14_verify);
  }
  return true;
}

static void bqAdcEnable() {
  uint8_t r2e = 0, r2f = 0;

  // Enable ADC (REG2E, bit7)
  if (bqRead8(REG2E_ADC_CTRL, r2e)) {
    uint8_t r2e_before = r2e;
    r2e |= (1 << 7);   // ADC_EN = 1
    r2e &= ~(1 << 6);  // ADC_RATE = 0 (continuous)
    if (bqWrite8(REG2E_ADC_CTRL, r2e)) {
      Serial.printf("[INIT] ✓ ADC enabled (REG2E: 0x%02X -> 0x%02X)\n", r2e_before, r2e);
    } else {
      Serial.println("[INIT] ⚠ Failed to write REG2E_ADC_CTRL");
    }
  } else {
    Serial.println("[INIT] ⚠ Failed to read REG2E_ADC_CTRL");
  }

  // Clear ADC disable bits (REG2F)
  if (bqRead8(REG2F_ADC_DIS0, r2f)) {
    uint8_t r2f_before = r2f;
    r2f &= ~(1 << 4);  // Clear VBAT disable bit (if applicable)
    if (bqWrite8(REG2F_ADC_DIS0, r2f)) {
      Serial.printf("[INIT] ✓ ADC disable bits cleared (REG2F: 0x%02X -> 0x%02X)\n", r2f_before, r2f);
    } else {
      Serial.println("[INIT] ⚠ Failed to write REG2F_ADC_DIS0");
    }
  } else {
    Serial.println("[INIT] ⚠ Failed to read REG2F_ADC_DIS0");
  }
  
  // Enable IBAT sensing
  if (!bqEnableIBAT()) {
    Serial.println("[INIT] ⚠ Failed to enable IBAT sensing");
  }

  delay(20);  // Allow ADC to settle
}

static bool bqInit2S_LiFePO4() {
  Serial.println("[INIT] Initializing BQ25792 for 2S LiFePO4...");
  
  // 1. Disable watchdog
  if (!bqDisableWatchdog()) {
    Serial.println("[INIT] ⚠ Failed to disable watchdog");
    return false;
  }
  Serial.println("[INIT] ✓ Watchdog disabled");
  
  // 2. Set cell count to 2S (REG0A, CELL[1:0] = 1)
  uint8_t r0a = 0;
  if (!bqRead8(REG0A_CELL, r0a)) {
    Serial.println("[INIT] ⚠ Failed to read REG0A_CELL");
    return false;
  }
  uint8_t r0a_before = r0a;
  r0a &= ~0x03;  // Clear CELL[1:0]
  r0a |= 0x01;   // Set CELL = 1 (2-series)
  if (!bqWrite8(REG0A_CELL, r0a)) {
    Serial.println("[INIT] ⚠ Failed to write REG0A_CELL");
    return false;
  }
  Serial.printf("[INIT] ✓ Cell count set to 2S (REG0A: 0x%02X -> 0x%02X)\n", r0a_before, r0a);
  
  // 3. Set VSYSMIN (REG00): 6000 mV = (6000-2500)/250 = 14 = 0x0E
  uint8_t vsysmin_code = (6000 - 2500) / 250;  // 14
  if (!bqWrite8(REG00_VSYSMIN, vsysmin_code & 0x3F)) {
    Serial.println("[INIT] ⚠ Failed to write REG00_VSYSMIN");
    return false;
  }
  Serial.printf("[INIT] ✓ VSYSMIN set to 6000mV (code: 0x%02X)\n", vsysmin_code);
  
  // 4. Set VREG (REG01): 7200 mV = 720 = 0x02D0 (LSB-first: 0xD0, 0x02)
  uint16_t vreg_code = 7200 / 10;  // 720
  if (!bqWrite16_LSB(REG01_VREG, vreg_code)) {
    Serial.println("[INIT] ⚠ Failed to write REG01_VREG");
    return false;
  }
  Serial.printf("[INIT] ✓ VREG set to 7200mV (code: 0x%04X)\n", vreg_code);
  
  // 5. Set ICHG (REG03): 2500 mA = 250 = 0x00FA (LSB-first: 0xFA, 0x00)
  uint16_t ichg_code = 2500 / 10;  // 250
  if (!bqWrite16_LSB(REG03_ICHG, ichg_code)) {
    Serial.println("[INIT] ⚠ Failed to write REG03_ICHG");
    return false;
  }
  Serial.printf("[INIT] ✓ ICHG set to 2500mA (code: 0x%04X)\n", ichg_code);
  
  // 6. Set IINDPM (REG06): 3000 mA = 300 = 0x012C (LSB-first: 0x2C, 0x01)
  uint16_t iindpm_code = 3000 / 10;  // 300
  if (!bqWrite16_LSB(REG06_IINDPM, iindpm_code)) {
    Serial.println("[INIT] ⚠ Failed to write REG06_IINDPM");
    return false;
  }
  Serial.printf("[INIT] ✓ IINDPM set to 3000mA (code: 0x%04X)\n", iindpm_code);
  
  // 7. Set ITERM (REG09): 200 mA = 200/40 = 5
  uint8_t iterm_code = 200 / 40;  // 5
  if (!bqRead8(REG09_ITERM, r0a)) {
    Serial.println("[INIT] ⚠ Failed to read REG09_ITERM");
    return false;
  }
  uint8_t iterm_before = r0a;
  r0a &= ~0x1F;  // Clear ITERM bits (assuming bits[4:0])
  r0a |= (iterm_code & 0x1F);
  if (!bqWrite8(REG09_ITERM, r0a)) {
    Serial.println("[INIT] ⚠ Failed to write REG09_ITERM");
    return false;
  }
  Serial.printf("[INIT] ✓ ITERM set to 200mA (REG09: 0x%02X -> 0x%02X)\n", iterm_before, r0a);
  
  // 8. Enable ADC and IBAT sensing
  bqAdcEnable();
  
  // 9. Start with charging disabled (EN_CHG = 0)
  uint8_t r0f = 1;
  if (!bqRead8(REG0F_CTRL0, r0f)) {
    Serial.println("[INIT] ⚠ Failed to read REG0F_CTRL0");
    return false;
  }
  uint8_t r0f_before = r0f;
  r0f &= ~(1 << 5);  // EN_CHG = 0 (disabled)
  if (!bqWrite8(REG0F_CTRL0, r0f)) {
    Serial.println("[INIT] ⚠ Failed to write REG0F_CTRL0");
    return false;
  }
  Serial.printf("[INIT] ✓ Charging disabled (REG0F: 0x%02X -> 0x%02X)\n", r0f_before, r0f);
  
  Serial.println("[INIT] ✓ BQ25792 initialization complete!\n");
  return true;
}

// ============================================================================
// BATTERY READING FUNCTIONS
// ============================================================================

static float applyVbatCal(float vraw) {
  return vraw * VBAT_GAIN + VBAT_OFFSET_V;
}

static bool readVBATraw(float &vbatV) {
  // Try both MSB-first and LSB-first to find correct format
  static uint8_t vbat_format = 0;  // 0=unknown, 1=MSB, 2=LSB
  static uint32_t last_format_test = 0;
  
  uint16_t raw_msb = 0, raw_lsb = 0;
  bool msb_ok = bqRead16_MSB(REG3B_VBAT_ADC, raw_msb);
  bool lsb_ok = bqRead16_LSB(REG3B_VBAT_ADC, raw_lsb);
  
  // Test format on first call or every 30 seconds
  if (vbat_format == 0 || (millis() - last_format_test > 30000)) {
    if (msb_ok && lsb_ok) {
      // Both readable - choose the one that makes sense (should be 5000-7200 mV for 2S LiFePO4)
      float v_msb = raw_msb / 1000.0f;
      float v_lsb = raw_lsb / 1000.0f;
      
      if (v_msb >= 5.0f && v_msb <= 7.5f) {
        vbat_format = 1;  // MSB format is correct
        Serial.printf("[VBAT] Using MSB format (MSB=%.3fV, LSB=%.3fV)\n", v_msb, v_lsb);
      } else if (v_lsb >= 5.0f && v_lsb <= 7.5f) {
        vbat_format = 2;  // LSB format is correct
        Serial.printf("[VBAT] Using LSB format (MSB=%.3fV, LSB=%.3fV)\n", v_msb, v_lsb);
      } else {
        // Neither makes sense - prefer MSB (datasheet says MSB-first)
        vbat_format = 1;
        Serial.printf("[VBAT] Neither format makes sense, using MSB (MSB=%.3fV, LSB=%.3fV)\n", v_msb, v_lsb);
      }
      last_format_test = millis();
    } else if (msb_ok) {
      vbat_format = 1;
    } else if (lsb_ok) {
      vbat_format = 2;
    } else {
      return false;
    }
  }
  
  // Use the determined format
  uint16_t raw = 0;
  if (vbat_format == 1) {
    if (!msb_ok) return false;
    raw = raw_msb;
  } else if (vbat_format == 2) {
    if (!lsb_ok) return false;
    raw = raw_lsb;
  } else {
    return false;
  }
  
  vbatV = raw / 1000.0f;  // Convert mV to V
  return true;
}

static bool readIBAT(float &ibatA, int16_t &raw_mA, uint8_t &working_reg) {
  // Try both REG32 and REG33
  static uint8_t cached_working_reg = 0;
  static uint32_t last_test_ms = 0;
  
  uint16_t u = 0;
  bool success = false;
  
  // Test both registers on first call or every 30 seconds
  if (cached_working_reg == 0 || (millis() - last_test_ms > 30000)) {
    uint16_t u32 = 0, u33 = 0;
    bool r32_ok = bqRead16_LSB(REG32_IBAT, u32);
    bool r33_ok = bqRead16_LSB(REG33_IBAT, u33);
    
    if (r32_ok || r33_ok) {
      if (r32_ok && r33_ok) {
        // Both work - prefer REG32 (reference implementation)
        cached_working_reg = REG32_IBAT;
        u = u32;
        success = true;
        Serial.printf("[IBAT] Both REG32 and REG33 readable. Using REG32 (0x%04X vs 0x%04X)\n", u32, u33);
      } else if (r32_ok) {
        cached_working_reg = REG32_IBAT;
        u = u32;
        success = true;
        Serial.printf("[IBAT] Using REG32 (0x%04X)\n", u32);
      } else {
        cached_working_reg = REG33_IBAT;
        u = u33;
        success = true;
        Serial.printf("[IBAT] Using REG33 (0x%04X)\n", u33);
      }
      last_test_ms = millis();
    }
  } else {
    // Use the cached working register
    if (bqRead16_LSB(cached_working_reg, u)) {
      success = true;
    }
  }
  
  if (!success) {
    return false;
  }
  
  int16_t s = (int16_t)u;  // Signed 16-bit value
  raw_mA = s;
  ibatA = (float)s / 1000.0f;  // Convert from mA to A
  working_reg = cached_working_reg;
  return true;
}

// Simple SOC lookup table (2S LiFePO4)
static float socFromVoltage(float v) {
  // 2S LiFePO4: 5.0V (0%) to 7.2V (100%)
  if (v <= 5.0f) return 0.0f;
  if (v >= 7.2f) return 100.0f;
  return ((v - 5.0f) / 2.2f) * 100.0f;
}

// ============================================================================
// MAIN SETUP AND LOOP
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n========================================");
  Serial.println("BQ25792 Battery Test - Standalone");
  Serial.println("========================================\n");
  
  // Initialize I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_HZ);
  delay(100);
  
  // Probe BQ25792
  if (!bqProbe()) {
    Serial.println("[ERROR] BQ25792 not found at address 0x6B!");
    Serial.println("Check I2C connections:");
    Serial.printf("  SDA: GPIO %d\n", I2C_SDA);
    Serial.printf("  SCL: GPIO %d\n", I2C_SCL);
    Serial.println("\nHalting...");
    while (1) delay(1000);
  }
  Serial.println("[INIT] ✓ BQ25792 found at address 0x6B");
  
  // Initialize BQ25792
  if (!bqInit2S_LiFePO4()) {
    Serial.println("[ERROR] Failed to initialize BQ25792!");
    Serial.println("Halting...");
    while (1) delay(1000);
  }
  
  Serial.println("\n========================================");
  Serial.println("Starting battery readings...");
  Serial.println("========================================\n");
  Serial.println("Format: VBAT | IBAT | SOC | Charging | Register");
  Serial.println("------------------------------------------------");
}

void loop() {
  static uint32_t lastRead = 0;
  const uint32_t READ_INTERVAL = 1000;  // Read every 1 second
  
  if (millis() - lastRead < READ_INTERVAL) {
    delay(10);
    return;
  }
  lastRead = millis();
  
  // Read VBAT
  float vbat_raw = 0;
  float vbat_cal = 0;
  bool vbat_ok = readVBATraw(vbat_raw);
  if (vbat_ok) {
    vbat_cal = applyVbatCal(vbat_raw);
  }
  
  // Read IBAT
  float ibat = 0;
  int16_t raw_mA = 0;
  uint8_t working_reg = 0;
  bool ibat_ok = readIBAT(ibat, raw_mA, working_reg);
  
  // Calculate SOC
  float soc = vbat_ok ? socFromVoltage(vbat_cal) : 0.0f;
  
  // Detect charging (positive current > 50mA threshold)
  bool is_charging = ibat_ok && (ibat > 0.05f);
  
  // Print results
  Serial.printf("VBAT: ");
  if (vbat_ok) {
    Serial.printf("%.3fV (raw: %.3fV)", vbat_cal, vbat_raw);
  } else {
    Serial.print("ERROR");
  }
  
  Serial.print(" | IBAT: ");
  if (ibat_ok) {
    Serial.printf("%.3fA (raw: %d mA)", ibat, raw_mA);
  } else {
    Serial.print("ERROR");
  }
  
  Serial.printf(" | SOC: %.1f%%", soc);
  Serial.printf(" | Charging: %s", is_charging ? "YES" : "NO");
  
  if (ibat_ok && working_reg > 0) {
    Serial.printf(" | Reg: 0x%02X", working_reg);
  }
  
  Serial.println();
  
  // Print detailed debug every 5 seconds
  static uint32_t lastDebug = 0;
  if (millis() - lastDebug > 5000) {
    lastDebug = millis();
    Serial.println("\n--- DEBUG INFO ---");
    
    // Read and print register values
    uint8_t r0f = 0, r14 = 0, r2e = 0, r2f = 0;
    if (bqRead8(REG0F_CTRL0, r0f)) {
      Serial.printf("REG0F (CTRL0): 0x%02X (EN_CHG=%d)\n", r0f, (r0f >> 5) & 1);
    }
    if (bqRead8(REG14_CTRL5, r14)) {
      Serial.printf("REG14 (CTRL5): 0x%02X (EN_IBAT=%d)\n", r14, (r14 >> 5) & 1);
    }
    if (bqRead8(REG2E_ADC_CTRL, r2e)) {
      Serial.printf("REG2E (ADC_CTRL): 0x%02X (ADC_EN=%d)\n", r2e, (r2e >> 7) & 1);
    }
    if (bqRead8(REG2F_ADC_DIS0, r2f)) {
      Serial.printf("REG2F (ADC_DIS0): 0x%02X\n", r2f);
    }
    
    // Try reading both IBAT registers (read multiple times to catch actual value)
    uint16_t u32_1 = 0, u32_2 = 0, u33_1 = 0, u33_2 = 0;
    bool r32_ok = bqRead16_LSB(REG32_IBAT, u32_1);
    delay(10);
    if (r32_ok) bqRead16_LSB(REG32_IBAT, u32_2);
    bool r33_ok = bqRead16_LSB(REG33_IBAT, u33_1);
    delay(10);
    if (r33_ok) bqRead16_LSB(REG33_IBAT, u33_2);
    
    Serial.printf("REG32_IBAT: %s (0x%04X, 0x%04X = %d, %d mA)\n", 
                  r32_ok ? "OK" : "FAIL", u32_1, u32_2, (int16_t)u32_1, (int16_t)u32_2);
    Serial.printf("REG33_IBAT: %s (0x%04X, 0x%04X = %d, %d mA)\n", 
                  r33_ok ? "OK" : "FAIL", u33_1, u33_2, (int16_t)u33_1, (int16_t)u33_2);
    
    // Also try reading VBAT in both formats for debug
    uint16_t vbat_msb = 0, vbat_lsb = 0;
    bool vbat_msb_ok = bqRead16_MSB(REG3B_VBAT_ADC, vbat_msb);
    bool vbat_lsb_ok = bqRead16_LSB(REG3B_VBAT_ADC, vbat_lsb);
    Serial.printf("REG3B_VBAT: MSB=%s (0x%04X=%u mV), LSB=%s (0x%04X=%u mV)\n",
                  vbat_msb_ok ? "OK" : "FAIL", vbat_msb, vbat_msb,
                  vbat_lsb_ok ? "OK" : "FAIL", vbat_lsb, vbat_lsb);
    
    Serial.println("------------------\n");
  }
}
