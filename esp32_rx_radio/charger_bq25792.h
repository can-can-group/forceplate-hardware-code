/*
 * charger_bq25792.h - Read-only BQ25792 battery/charger module for ESP32.
 * Shared by esp32_spi_slave and esp32_rx_radio. No charge control (no EN_CHG
 * hysteresis or auto IINDPM in Charger_Update).
 */

#ifndef CHARGER_BQ25792_H
#define CHARGER_BQ25792_H

#include <Arduino.h>

// Initialize I2C and BQ25792. Call once from setup(). sda/scl = GPIO pins (e.g. 8, 9).
// Returns true if BQ25792 was detected and configured.
bool Charger_Init(int sda, int scl);

// Call periodically from loop() to read telemetry and update VBAT average, display comp, SOC.
// Read-only: does not call setChargeEnable, chargeHysteresisUpdate, or autoControlStep.
void Charger_Update(void);

// Getters (use after Charger_Update).
float Charger_GetVBATDisplay_V(void);   // Compensated VBAT for SOC/UI (V)
float Charger_GetSOC_pct(void);          // State of charge 0..100% (linear 6.0..7.3 V)
bool Charger_IsPowerGood(void);          // Input power good
bool Charger_IsCharging(void);           // PG and IBAT > threshold (for reporting)

// Returns true if Charger_Init() succeeded.
bool Charger_InitOk(void);

#endif
