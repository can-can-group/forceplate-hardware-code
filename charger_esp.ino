#include <Wire.h>


// ===================== BQ25792 =====================
static const uint8_t BQ_ADDR = 0x6B;

// ---- Config / control registers (per your table) ----
static const uint8_t REG00_VSYSMIN  = 0x00; // Minimal system voltage
static const uint8_t REG01_VREG     = 0x01; // Charge voltage limit
static const uint8_t REG03_ICHG     = 0x03; // Charge current limit
static const uint8_t REG05_VINDPM   = 0x05; // Input voltage limit
static const uint8_t REG06_IINDPM   = 0x06; // Input current limit
static const uint8_t REG0F_CTRL0    = 0x0F; // Charger control 0
static const uint8_t REG10_CTRL1    = 0x10; // Charger control 1
static const uint8_t REG18_NTC1     = 0x18; // NTC control 1
static const uint8_t REG2E_ADC_CTRL = 0x2E; // ADC control
static const uint8_t REG2F_ADC_DIS0 = 0x2F; // ADC function disable 0
static const uint8_t REG30_ADC_DIS1 = 0x30; // ADC function disable 1

// ---- Status / fault ----
static const uint8_t REG1B_STAT0 = 0x1B;
static const uint8_t REG1C_STAT1 = 0x1C;
static const uint8_t REG1D_STAT2 = 0x1D;
static const uint8_t REG1E_STAT3 = 0x1E;
static const uint8_t REG20_FAULT0 = 0x20;
static const uint8_t REG21_FAULT1 = 0x21;

// ---- ADC readouts (16-bit, per your table) ----
static const uint8_t REG31_IBUS_ADC = 0x31;
static const uint8_t REG33_IBAT_ADC = 0x33;
static const uint8_t REG35_VBUS_ADC = 0x35;
static const uint8_t REG37_VAC1_ADC = 0x37;
static const uint8_t REG39_VAC2_ADC = 0x39;
static const uint8_t REG3B_VBAT_ADC = 0x3B;
static const uint8_t REG3D_VSYS_ADC = 0x3D;

// ===================== User targets (2S LiFePO4) =====================
static const float VBAT_MIN_V = 6.00f;
static const float VBAT_MAX_V = 7.30f;

// ===================== AUTO IINDPM control =====================
static uint16_t iindpm_mA = 1200;          // start-mid
static uint16_t iindpm_min_mA  = 300;
static uint16_t iindpm_max_mA  = 2500;
static uint16_t iindpm_step_mA = 50;

static float vbus_target_V   = 4.80f;      // droop target
static float vbus_deadband_V = 0.05f;
static float vbus_panic_V    = 4.55f;

static bool auto_mode = true;

// Timing
static uint32_t last_ctrl_ms   = 0;
static const uint32_t CTRL_PERIOD_MS = 300;

static uint32_t last_serial_ms = 0;
static const uint32_t SERIAL_PERIOD_MS = 1000;

static uint32_t last_ble_ms = 0;
static const uint32_t BLE_PERIOD_MS = 500; // 2 Hz BLE

// ===================== VBAT moving average =====================
static const int VBAT_AVG_N = 32; // 32-sample moving average
static float vbat_buf[VBAT_AVG_N];
static int vbat_idx = 0;
static bool vbat_filled = false;

static float vbat_avg_V = NAN;

// ===================== VBAT "display" offset compensation =====================
// Calibrate these:
static float R_int_ohm      = 0.125f; // start guess (adjust with your logs)
static float V_surface_V    = 0.040f; // 40mV start guess
static float ibat_charge_th_A = 0.08f; // above this => "charging"
static float vbat_offset_filt_V = 0.0f;
static float vbat_disp_V = NAN;

// Smoothing for offset (0.03..0.10 typical)
static float offset_alpha = 0.06f;

// ===================== Charge hysteresis (100/95) =====================
static bool chg_enable = true;     // internal state (what we believe chip is set to)
static bool holdoff = false;       // true => we stopped at 100, wait until <=95
static bool last_pg = false;       // to detect plug-in transitions

// Manual lock for chg=off
static bool chg_manual_lock = false;   // if true, we force charger state and bypass hysteresis
static bool chg_manual_state = false;  // forced state when locked (false=OFF)

// ===================== I2C helpers =====================
bool write8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(BQ_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

bool read8(uint8_t reg, uint8_t &out) {
  Wire.beginTransmission(BQ_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  int n = Wire.requestFrom((int)BQ_ADDR, 1);
  if (n != 1) return false;
  out = Wire.read();
  return true;
}

// 16-bit registers MSB-first
bool write16_MSB(uint8_t reg, uint16_t val) {
  Wire.beginTransmission(BQ_ADDR);
  Wire.write(reg);
  Wire.write((uint8_t)((val >> 8) & 0xFF));
  Wire.write((uint8_t)(val & 0xFF));
  return (Wire.endTransmission() == 0);
}

bool read16_MSB(uint8_t reg, uint16_t &out) {
  Wire.beginTransmission(BQ_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  int n = Wire.requestFrom((int)BQ_ADDR, 2);
  if (n != 2) return false;
  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  out = ((uint16_t)msb << 8) | lsb;
  return true;
}

// ===================== Scales / encode-decode =====================
// ADC voltages: 1mV/bit
static float adcVoltage_V(uint16_t raw_mV) { return (float)raw_mV / 1000.0f; }
// ADC currents: 1mA/bit
static float adcCurrent_A(uint16_t raw_mA) { return (float)raw_mA / 1000.0f; }

// IINDPM: 10mA/LSB
static uint16_t enc_IINDPM_mA(uint16_t mA) {
  if (mA < 100) mA = 100;
  if (mA > 3300) mA = 3300;
  return (uint16_t)(mA / 10);
}
static uint16_t dec_IINDPM_mA(uint16_t raw) { return (uint16_t)(raw * 10); }

// ICHG: 10mA/LSB
static uint16_t enc_ICHG_mA(uint16_t mA) { return (uint16_t)(mA / 10); }
static uint16_t dec_ICHG_mA(uint16_t raw) { return (uint16_t)(raw * 10); }

// VREG: 10mV/LSB
static uint16_t enc_VREG_mV(uint16_t mV) { return (uint16_t)(mV / 10); }
static uint16_t dec_VREG_mV(uint16_t raw) { return (uint16_t)(raw * 10); }

// VINDPM: 64mV/LSB
static uint16_t enc_VINDPM_mV(uint16_t mV) { return (uint16_t)(mV / 64); }
static uint16_t dec_VINDPM_mV(uint16_t raw) { return (uint16_t)(raw * 64); }

// ===================== ADC reads =====================
static bool readADC16(uint8_t reg, uint16_t &raw) {
  return read16_MSB(reg, raw);
}

static float readVoltageFromADC(uint8_t reg, uint16_t &raw_mV) {
  raw_mV = 0;
  if (!readADC16(reg, raw_mV)) return NAN;
  return adcVoltage_V(raw_mV);
}

static float readCurrentFromADC(uint8_t reg, uint16_t &raw_mA) {
  raw_mA = 0;
  if (!readADC16(reg, raw_mA)) return NAN;
  return adcCurrent_A(raw_mA);
}

// ===================== VBAT moving average =====================
static void vbatAvgPush(float v) {
  if (!isfinite(v)) return;

  vbat_buf[vbat_idx] = v;
  vbat_idx++;
  if (vbat_idx >= VBAT_AVG_N) {
    vbat_idx = 0;
    vbat_filled = true;
  }

  int n = vbat_filled ? VBAT_AVG_N : vbat_idx;
  if (n <= 0) return;

  float sum = 0.0f;
  for (int i = 0; i < n; i++) sum += vbat_buf[i];
  vbat_avg_V = sum / (float)n;
}

// ===================== SOC (simple linear) =====================
static float calcSOC_pct(float vbat_for_soc) {
  if (!isfinite(vbat_for_soc)) return NAN;
  float x = (vbat_for_soc - VBAT_MIN_V) / (VBAT_MAX_V - VBAT_MIN_V);
  if (x < 0) x = 0;
  if (x > 1) x = 1;
  return x * 100.0f;
}


// ===================== Status helpers =====================
static bool isPowerGood() {
  uint8_t s0 = 0;
  if (!read8(REG1B_STAT0, s0)) return false;
  return ((s0 >> 3) & 1) ? true : false;
}

static void readStatus(uint8_t &s0, uint8_t &s1, uint8_t &s2, uint8_t &s3) {
  read8(REG1B_STAT0, s0);
  read8(REG1C_STAT1, s1);
  read8(REG1D_STAT2, s2);
  read8(REG1E_STAT3, s3);
}

static void readFault(uint8_t &f0, uint8_t &f1) {
  read8(REG20_FAULT0, f0);
  read8(REG21_FAULT1, f1);
}

// ===================== Charger control (REG0F) =====================
// We only touch EN_CHG at bit5 (your working assumption).
// Also keep a local "chg_enable" state updated on success.
bool setChargeEnable(bool en)
{
  uint8_t v = 0;
  if (!read8(REG0F_CTRL0, v)) return false;

  if (en) v |=  (1u << 5);   // EN_CHG = 1
  else    v &= ~(1u << 5);   // EN_CHG = 0

  if (!write8(REG0F_CTRL0, v)) return false;

  // Readback verify
  uint8_t rb = 0;
  if (!read8(REG0F_CTRL0, rb)) return false;

  bool en_rb = (rb >> 5) & 1;
  if (en_rb == en) {
    chg_enable = en;
    return true;
  }
  return false;
}

// ===================== Set limits =====================
static void setInputCurrentLimit_mA(uint16_t mA) {
  iindpm_mA = mA;
  write16_MSB(REG06_IINDPM, enc_IINDPM_mA(mA));
}

static void setChargeCurrent_mA(uint16_t mA) {
  write16_MSB(REG03_ICHG, enc_ICHG_mA(mA));
}

static void setChargeVoltage_mV(uint16_t mV) {
  write16_MSB(REG01_VREG, enc_VREG_mV(mV));
}

static void setVindpm_mV(uint16_t mV) {
  write16_MSB(REG05_VINDPM, enc_VINDPM_mV(mV));
}

// ===================== Configure BQ25792 =====================
// Safest: write config at every boot.
static void configureBQ() {
  // Disable watchdog / default control for your use
  write8(REG10_CTRL1, 0x00);

  // NTC ignore / safe config
  write8(REG18_NTC1, 0x55);

  // Enable ADC continuous (your working value)
  write8(REG2E_ADC_CTRL, 0x8C);

  // Ensure ADC channels enabled
  write8(REG2F_ADC_DIS0, 0x00);
  write8(REG30_ADC_DIS1, 0x00);

  // Set LiFePO4 targets
  setChargeVoltage_mV(7300);  // 7.30V
  setChargeCurrent_mA(2500);  // 2.5A
  setVindpm_mV(4400);         // 4.4V

  // Start mid IINDPM
  setInputCurrentLimit_mA(iindpm_mA);

  // Enable charging initially
  setChargeEnable(true);

  // Clear holdoff at boot (so it can charge to 100 if plugged)
  holdoff = false;

  // Clear manual lock at boot
  chg_manual_lock = false;
  chg_manual_state = false;

  // Offset filter reset (optional)
  vbat_offset_filt_V = 0.0f;
}

// ===================== AUTO IINDPM controller =====================
static void autoControlStep(float vbus) {
  if (!isfinite(vbus)) return;

  // If charger disabled, don't push IINDPM up
  if (!chg_enable) return;

  // Panic droop: back off fast
  if (vbus < vbus_panic_V) {
    uint16_t dec = iindpm_step_mA * 3;
    uint16_t new_mA = (iindpm_mA > dec) ? (iindpm_mA - dec) : iindpm_min_mA;
    if (new_mA < iindpm_min_mA) new_mA = iindpm_min_mA;
    if (new_mA != iindpm_mA) setInputCurrentLimit_mA(new_mA);
    return;
  }

  float err = vbus - vbus_target_V;

  // deadband
  if (fabs(err) <= vbus_deadband_V) return;

  if (err > 0) {
    // increase slowly
    uint16_t new_mA = iindpm_mA + iindpm_step_mA;
    if (new_mA > iindpm_max_mA) new_mA = iindpm_max_mA;
    if (new_mA != iindpm_mA) setInputCurrentLimit_mA(new_mA);
  } else {
    // decrease faster
    uint16_t dec = iindpm_step_mA * 2;
    uint16_t new_mA = (iindpm_mA > dec) ? (iindpm_mA - dec) : iindpm_min_mA;
    if (new_mA < iindpm_min_mA) new_mA = iindpm_min_mA;
    if (new_mA != iindpm_mA) setInputCurrentLimit_mA(new_mA);
  }
}

// ===================== Charge hysteresis (100/95) =====================
static void chargeHysteresisUpdate(bool pg, float soc) {
  // Plug-in detection: PG rising edge clears holdoff and enables charge-to-100
  if (pg && !last_pg) {
    holdoff = false;
    setChargeEnable(true);
  }
  last_pg = pg;

  // If no input, do nothing
  if (!pg) return;
  if (!isfinite(soc)) return;

  // If we already stopped at 100, wait for <=95 to resume
  if (holdoff) {
    if (soc <= 95.0f) {
      holdoff = false;
      setChargeEnable(true);
    } else {
      setChargeEnable(false);
    }
    return;
  }

  // Normal: charge until 100, then stop and enter holdoff
  if (soc >= 100.0f) {
    holdoff = true;
    setChargeEnable(false);
  } else {
    setChargeEnable(true);
  }
}

// ===================== VBAT offset compensation =====================
static void updateVbatDisplayComp(bool pg, float ibat_A) {
  float offset_target = 0.0f;

  // "charging" detection: PG + charger enabled + IBAT > threshold
  bool charging_now = (pg && chg_enable && isfinite(ibat_A) && (ibat_A > ibat_charge_th_A));

  if (charging_now) {
    offset_target = (ibat_A * R_int_ohm) + V_surface_V;
  } else {
    offset_target = 0.0f;
  }

  // Smooth it
  vbat_offset_filt_V += offset_alpha * (offset_target - vbat_offset_filt_V);

  // Display VBAT for SOC & UI
  if (isfinite(vbat_avg_V)) {
    vbat_disp_V = vbat_avg_V - vbat_offset_filt_V;
  } else {
    vbat_disp_V = NAN;
  }
}

// ===================== Commands =====================
static void printHelp() {
  Serial.println("Commands:");
  Serial.println("  help");
  Serial.println("  reinit");
  Serial.println("  dump");
  Serial.println("  auto=on | auto=off");
  Serial.println("  target=4.80");
  Serial.println("  iindpm=1200 (forces AUTO off)");
  Serial.println("  chg=on | chg=off");
  Serial.println("  rint=0.095   (ohm, VBAT display comp)");
  Serial.println("  vsurf=0.070  (V, VBAT display comp)");
}

static void applyCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();
  if (cmd.length() == 0) return;

  if (cmd == "help") { printHelp(); return; }

  if (cmd == "reinit") {
    Serial.println("reinit...");
    configureBQ();
    return;
  }

  if (cmd == "dump") {
    uint8_t s0=0,s1=0,s2=0,s3=0,f0=0,f1=0;
    readStatus(s0,s1,s2,s3);
    readFault(f0,f1);

    uint8_t r0f=0, r10=0;
    read8(REG0F_CTRL0, r0f);
    read8(REG10_CTRL1, r10);

    uint16_t vreg_raw=0, ichg_raw=0, iind_raw=0, vind_raw=0;
    read16_MSB(REG01_VREG, vreg_raw);
    read16_MSB(REG03_ICHG, ichg_raw);
    read16_MSB(REG06_IINDPM, iind_raw);
    read16_MSB(REG05_VINDPM, vind_raw);

    Serial.print("REG0F=0x"); Serial.print(r0f, HEX);
    Serial.print(" REG10=0x"); Serial.print(r10, HEX);
    Serial.print(" | STAT: s0=0x"); Serial.print(s0, HEX);
    Serial.print(" s1=0x"); Serial.print(s1, HEX);
    Serial.print(" s2=0x"); Serial.print(s2, HEX);
    Serial.print(" s3=0x"); Serial.print(s3, HEX);
    Serial.print(" | FAULT: f0=0x"); Serial.print(f0, HEX);
    Serial.print(" f1=0x"); Serial.println(f1, HEX);

    Serial.print("READBACK: VREG="); Serial.print(dec_VREG_mV(vreg_raw)); Serial.print("mV ");
    Serial.print("ICHG="); Serial.print(dec_ICHG_mA(ichg_raw)); Serial.print("mA ");
    Serial.print("IINDPM="); Serial.print(dec_IINDPM_mA(iind_raw)); Serial.print("mA ");
    Serial.print("VINDPM="); Serial.print(dec_VINDPM_mV(vind_raw)); Serial.println("mV");
    return;
  }

  if (cmd == "auto=on")  { auto_mode = true;  Serial.println("AUTO=ON");  return; }
  if (cmd == "auto=off") { auto_mode = false; Serial.println("AUTO=OFF"); return; }

  if (cmd.startsWith("target=")) {
    float v = cmd.substring(7).toFloat();
    if (v > 4.2f && v < 5.2f) {
      vbus_target_V = v;
      Serial.print("target="); Serial.println(vbus_target_V, 3);
    }
    return;
  }

  if (cmd.startsWith("iindpm=")) {
    int v = cmd.substring(7).toInt();
    if (v > 0) {
      auto_mode = false;
      if (v < (int)iindpm_min_mA) v = iindpm_min_mA;
      if (v > (int)iindpm_max_mA) v = iindpm_max_mA;
      setInputCurrentLimit_mA((uint16_t)v);
      Serial.print("AUTO=OFF IINDPM="); Serial.print(iindpm_mA); Serial.println("mA");
    }
    return;
  }

  if (cmd.startsWith("rint=")) {
    float v = cmd.substring(5).toFloat();
    if (v >= 0.0f && v <= 1.0f) {
      R_int_ohm = v;
      Serial.print("R_int="); Serial.print(R_int_ohm, 4); Serial.println(" ohm");
    }
    return;
  }

  if (cmd.startsWith("vsurf=")) {
    float v = cmd.substring(6).toFloat();
    if (v >= 0.0f && v <= 0.2f) {
      V_surface_V = v;
      Serial.print("V_surface="); Serial.print(V_surface_V, 4); Serial.println(" V");
    }
    return;
  }

  if (cmd == "chg=off") {
    // Manual OFF lock: stop charging and prevent hysteresis from turning it back on
    chg_manual_lock = true;
    chg_manual_state = false;

    // Disable AUTO as requested
    auto_mode = false;

    bool ok = setChargeEnable(false);
    uint8_t r0f=0; read8(REG0F_CTRL0, r0f);
    Serial.print(ok ? "CHG=OFF OK " : "CHG=OFF failed ");
    Serial.print("REG0F=0x"); Serial.print(r0f, HEX);
    Serial.print(" EN_CHG="); Serial.println((r0f >> 5) & 1);
    return;
  }

  if (cmd == "chg=on") {
    // Release lock and return to normal behavior
    chg_manual_lock = false;

    // Enable AUTO again as requested
    auto_mode = true;

    // Clear holdoff so it can charge to 100 if needed
    holdoff = false;

    bool ok = setChargeEnable(true);
    uint8_t r0f=0; read8(REG0F_CTRL0, r0f);
    Serial.print(ok ? "CHG=ON OK " : "CHG=ON failed ");
    Serial.print("REG0F=0x"); Serial.print(r0f, HEX);
    Serial.print(" EN_CHG="); Serial.println((r0f >> 5) & 1);
    return;
  }

  Serial.println("Unknown cmd. Type help");
}



// ===================== Setup / loop =====================
void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin();
  Wire.setClock(400000);

  Serial.println("BQ25792 init...");
  configureBQ();

  Serial.println("Commands: help | reinit | dump | auto=on/off | target=4.80 | iindpm=900 | chg=on/off | rint= | vsurf=");
}

void loop() {
  // Serial commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    applyCommand(cmd);
  }

  // ---- Read telemetry ----
  uint16_t raw_vbus=0, raw_ibus=0, raw_vbat=0, raw_ibat=0, raw_vsys=0, raw_vac1=0, raw_vac2=0;
  float vbus = readVoltageFromADC(REG35_VBUS_ADC, raw_vbus);
  float vac1 = readVoltageFromADC(REG37_VAC1_ADC, raw_vac1);
  float vac2 = readVoltageFromADC(REG39_VAC2_ADC, raw_vac2);
  float vbat = readVoltageFromADC(REG3B_VBAT_ADC, raw_vbat);
  float vsys = readVoltageFromADC(REG3D_VSYS_ADC, raw_vsys);
  float ibus = readCurrentFromADC(REG31_IBUS_ADC, raw_ibus);
  float ibat = readCurrentFromADC(REG33_IBAT_ADC, raw_ibat);

  // VBAT software moving average
  vbatAvgPush(vbat);

  // Power-good
  bool pg = isPowerGood();

  // Update "display" VBAT compensation (used for SOC/UI)
  updateVbatDisplayComp(pg, ibat);

  // SOC from compensated VBAT (prevents bounce)
  float soc = calcSOC_pct(vbat_disp_V);

  // ---- Charger control path ----
  if (chg_manual_lock) {
    // Forced state: keep charger fixed, skip hysteresis
    setChargeEnable(chg_manual_state);
    // also keep holdoff cleared (manual rules)
    holdoff = false;
  } else {
    // Normal hysteresis
    chargeHysteresisUpdate(pg, soc);
  }

  uint32_t now = millis();

  // ---- AUTO IINDPM ----
  if (!chg_manual_lock && auto_mode && (now - last_ctrl_ms >= CTRL_PERIOD_MS)) {
    last_ctrl_ms = now;
    if (pg) autoControlStep(vbus);
  }

  // ---- Serial telemetry 1 Hz ----
  if (now - last_serial_ms >= SERIAL_PERIOD_MS) {
    last_serial_ms = now;

    uint8_t s0=0,s1=0,s2=0,s3=0,f0=0,f1=0;
    readStatus(s0,s1,s2,s3);
    readFault(f0,f1);

    uint8_t r0f=0; read8(REG0F_CTRL0, r0f);

    Serial.print("IBUS="); Serial.print(ibus, 3); Serial.print("A(raw="); Serial.print(raw_ibus); Serial.print(") | ");
    
    Serial.print("VBAT(disp)="); Serial.print(vbat_disp_V, 3);

    Serial.print("SOC="); Serial.print(soc, 1); Serial.print("% ");
    Serial.print("CHG_EN="); Serial.print(chg_enable ? 1 : 0);

    Serial.print(" | IINDPM="); Serial.print(iindpm_mA); Serial.print("mA AUTO="); Serial.print(auto_mode ? 1 : 0);
    Serial.println();
  }
}
