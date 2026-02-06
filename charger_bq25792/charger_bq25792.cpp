/*
 * charger_bq25792.cpp - Read-only BQ25792 module (no EN_CHG hysteresis / auto IINDPM in Update).
 * Extracted from charger_esp.ino for use by esp32_spi_slave and esp32_rx_radio.
 */

#include "charger_bq25792.h"
#include <Wire.h>
#include <math.h>

// ===================== BQ25792 =====================
static const uint8_t BQ_ADDR = 0x6B;

static const uint8_t REG00_VSYSMIN  = 0x00;
static const uint8_t REG01_VREG     = 0x01;
static const uint8_t REG03_ICHG     = 0x03;
static const uint8_t REG05_VINDPM   = 0x05;
static const uint8_t REG06_IINDPM   = 0x06;
static const uint8_t REG0F_CTRL0    = 0x0F;
static const uint8_t REG10_CTRL1    = 0x10;
static const uint8_t REG18_NTC1     = 0x18;
static const uint8_t REG2E_ADC_CTRL = 0x2E;
static const uint8_t REG2F_ADC_DIS0 = 0x2F;
static const uint8_t REG30_ADC_DIS1 = 0x30;
static const uint8_t REG1B_STAT0    = 0x1B;
static const uint8_t REG31_IBUS_ADC = 0x31;
static const uint8_t REG33_IBAT_ADC = 0x33;
static const uint8_t REG35_VBUS_ADC = 0x35;
static const uint8_t REG3B_VBAT_ADC = 0x3B;
static const uint8_t REG3D_VSYS_ADC = 0x3D;

static const float VBAT_MIN_V = 6.00f;
static const float VBAT_MAX_V = 7.30f;

static const int VBAT_AVG_N = 32;
static float vbat_buf[VBAT_AVG_N];
static int vbat_idx = 0;
static bool vbat_filled = false;
static float vbat_avg_V = NAN;

static float R_int_ohm      = 0.125f;
static float V_surface_V    = 0.040f;
static float ibat_charge_th_A = 0.08f;
static float vbat_offset_filt_V = 0.0f;
static float vbat_disp_V = NAN;
static float offset_alpha = 0.06f;

static uint16_t iindpm_mA = 1200;

static bool charger_ok = false;
static bool last_pg = false;
static float last_soc = NAN;
static bool last_is_charging = false;

// I2C helpers
static bool write8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(BQ_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

static bool read8(uint8_t reg, uint8_t &out) {
  Wire.beginTransmission(BQ_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  int n = Wire.requestFrom((int)BQ_ADDR, 1);
  if (n != 1) return false;
  out = Wire.read();
  return true;
}

static bool write16_MSB(uint8_t reg, uint16_t val) {
  Wire.beginTransmission(BQ_ADDR);
  Wire.write(reg);
  Wire.write((uint8_t)((val >> 8) & 0xFF));
  Wire.write((uint8_t)(val & 0xFF));
  return (Wire.endTransmission() == 0);
}

static bool read16_MSB(uint8_t reg, uint16_t &out) {
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

static float adcVoltage_V(uint16_t raw_mV) { return (float)raw_mV / 1000.0f; }
static float adcCurrent_A(uint16_t raw_mA) { return (float)raw_mA / 1000.0f; }

static uint16_t enc_IINDPM_mA(uint16_t mA) {
  if (mA < 100) mA = 100;
  if (mA > 3300) mA = 3300;
  return (uint16_t)(mA / 10);
}
static uint16_t enc_ICHG_mA(uint16_t mA) { return (uint16_t)(mA / 10); }
static uint16_t enc_VREG_mV(uint16_t mV) { return (uint16_t)(mV / 10); }
static uint16_t enc_VINDPM_mV(uint16_t mV) { return (uint16_t)(mV / 64); }

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

static float calcSOC_pct(float vbat_for_soc) {
  if (!isfinite(vbat_for_soc)) return NAN;
  float x = (vbat_for_soc - VBAT_MIN_V) / (VBAT_MAX_V - VBAT_MIN_V);
  if (x < 0) x = 0;
  if (x > 1) x = 1;
  return x * 100.0f;
}

static bool isPowerGood() {
  uint8_t s0 = 0;
  if (!read8(REG1B_STAT0, s0)) return false;
  return ((s0 >> 3) & 1) ? true : false;
}

static bool setChargeEnable(bool en) {
  uint8_t v = 0;
  if (!read8(REG0F_CTRL0, v)) return false;
  if (en) v |=  (1u << 5);
  else    v &= ~(1u << 5);
  if (!write8(REG0F_CTRL0, v)) return false;
  uint8_t rb = 0;
  if (!read8(REG0F_CTRL0, rb)) return false;
  return ((rb >> 5) & 1) == en;
}

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

static void configureBQ() {
  write8(REG10_CTRL1, 0x00);
  write8(REG18_NTC1, 0x55);
  write8(REG2E_ADC_CTRL, 0x8C);
  write8(REG2F_ADC_DIS0, 0x00);
  write8(REG30_ADC_DIS1, 0x00);
  setChargeVoltage_mV(7300);
  setChargeCurrent_mA(2500);
  setVindpm_mV(4400);
  setInputCurrentLimit_mA(iindpm_mA);
  setChargeEnable(true);
  vbat_offset_filt_V = 0.0f;
}

// Charging = PG and IBAT > threshold (for reporting). No EN_CHG write.
// For display compensation we use PG + (read EN_CHG from chip) + IBAT > threshold.
static void updateVbatDisplayComp(bool pg, float ibat_A, bool chg_enable) {
  float offset_target = 0.0f;
  bool charging_now = (pg && chg_enable && isfinite(ibat_A) && (ibat_A > ibat_charge_th_A));
  if (charging_now) {
    offset_target = (ibat_A * R_int_ohm) + V_surface_V;
  }
  vbat_offset_filt_V += offset_alpha * (offset_target - vbat_offset_filt_V);
  if (isfinite(vbat_avg_V)) {
    vbat_disp_V = vbat_avg_V - vbat_offset_filt_V;
  } else {
    vbat_disp_V = NAN;
  }
}

bool Charger_Init(int sda, int scl) {
  Wire.begin(sda, scl);
  Wire.setClock(400000);

  Wire.beginTransmission(BQ_ADDR);
  if (Wire.endTransmission() != 0) {
    charger_ok = false;
    return false;
  }
  configureBQ();
  vbat_idx = 0;
  vbat_filled = false;
  vbat_avg_V = NAN;
  vbat_disp_V = NAN;
  vbat_offset_filt_V = 0.0f;
  last_pg = false;
  last_soc = NAN;
  last_is_charging = false;
  charger_ok = true;
  return true;
}

void Charger_Update(void) {
  if (!charger_ok) return;

  uint16_t raw_vbat = 0, raw_ibat = 0;
  float vbat = readVoltageFromADC(REG3B_VBAT_ADC, raw_vbat);
  float ibat = readCurrentFromADC(REG33_IBAT_ADC, raw_ibat);
  bool pg = isPowerGood();

  vbatAvgPush(vbat);

  uint8_t r0f = 0;
  bool chg_en = (read8(REG0F_CTRL0, r0f) && ((r0f >> 5) & 1));

  updateVbatDisplayComp(pg, ibat, chg_en);
  last_soc = calcSOC_pct(vbat_disp_V);
  last_pg = pg;
  last_is_charging = (pg && isfinite(ibat) && (ibat > ibat_charge_th_A));
}

float Charger_GetVBATDisplay_V(void) {
  return vbat_disp_V;
}

float Charger_GetSOC_pct(void) {
  return last_soc;
}

bool Charger_IsPowerGood(void) {
  return last_pg;
}

bool Charger_IsCharging(void) {
  return last_is_charging;
}

bool Charger_InitOk(void) {
  return charger_ok;
}
