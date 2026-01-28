#include <Arduino.h>
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <Wire.h>
#include <math.h>
#include "common_frame.h"

// ============================================================================
// BMS CONFIGURATION (from esp_bmsv2.ino)
// ============================================================================

static const uint8_t BQ_ADDR = 0x6B;
static const int I2C_SDA = 8;
static const int I2C_SCL = 9;
static const uint32_t I2C_HZ = 400000;

// VBAT calibration
static const float VBAT_GAIN     = 1.0000f;
static const float VBAT_OFFSET_V = -0.111f;

// Sampling
static const uint32_t BMS_SAMPLE_MS = 30000;  // Read battery every 30 seconds (reduced to save wireless bandwidth)
static const float EMA_ALPHA = 0.10f;

// BQ register map (subset for reading)
static const uint8_t REG00_VSYSMIN  = 0x00;
static const uint8_t REG01_VREG     = 0x01;
static const uint8_t REG03_ICHG     = 0x03;
static const uint8_t REG06_IINDPM   = 0x06;
static const uint8_t REG0F_CTRL0    = 0x0F;  // EN_CHG bit5
static const uint8_t REG10_CTRL1    = 0x10;  // WATCHDOG bits[2:0]
static const uint8_t REG09_ITERM    = 0x09;  // Termination Control (ITERM)
static const uint8_t REG0A_CELL     = 0x0A;  // Recharge / Cell config (CELL[1:0])
static const uint8_t REG14_CTRL5    = 0x14;  // Charger Control 5 (EN_IBAT bit5)
static const uint8_t REG2E_ADC_CTRL = 0x2E;  // ADC Control (ADC_EN bit7, ADC_RATE bit6)
static const uint8_t REG2F_ADC_DIS0 = 0x2F;  // ADC Function Disable 0
static const uint8_t REG3B_VBAT_ADC = 0x3B;  // VBAT ADC (16-bit, MSB-first, mV)
static const uint8_t REG32_IBAT     = 0x32;  // IBAT ADC (16-bit, LSB-first, signed, mA)
static const uint8_t REG33_IBAT     = 0x33;  // IBAT ADC alternate register

// IBAT register auto-detection state
static uint8_t ibat_working_reg = 0;  // 0=unknown, else = REG32 or REG33

// BMS state
static float vBatFiltCal = NAN;
static float currentSoc = 0.0f;
static float iBatFilt = NAN;  // Filtered battery current (A) - slow filter for SOC
static float iBatFiltFast = NAN;  // Fast filtered current for charging detection
static bool isCharging = false;  // Charging status (positive current > threshold) - actual state sent to Teensy
static uint32_t lastBmsSampleMs = 0;
static uint32_t lastChargingCheckMs = 0;  // Faster check for charging status
static bool bmsInitialized = false;

// SOC lookup table (VBATcal -> SOC)
struct BmsPt { float v; float soc; };
static const BmsPt SOC_LUT[] = {
  {5.80f,   0.0f},
  {6.00f,   5.0f},
  {6.20f,  10.0f},
  {6.30f,  20.0f},
  {6.40f,  30.0f},
  {6.50f,  40.0f},
  {6.56f,  50.0f},
  {6.60f,  60.0f},
  {6.64f,  70.0f},
  {6.68f,  80.0f},
  {6.74f,  90.0f},
  {7.00f,  95.0f},
  {7.10f,  98.0f},
  {7.20f, 100.0f}
};

// ESP-NOW Battery Packet
struct __attribute__((packed)) ESPNowBattery {
    uint8_t magic[2];     // 0xBA, 0xBB
    float voltage;        // Calibrated VBAT in volts
    float soc;            // State of charge 0-100%
    uint16_t crc16;
};  // 12 bytes

#define PIN_MOSI 11
#define PIN_MISO 13
#define PIN_SCLK 12
#define PIN_CS   10

// UART pins for Teensy communication (Serial1)
#define TEENSY_UART_TX_PIN 2   // GPIO2 → Teensy RX3 (Pin 15)
#define TEENSY_UART_RX_PIN 3   // GPIO3 ← Teensy TX3 (Pin 14)

#define FRAME_BYTES 144
#define QUEUED_XFERS 32
#define SAMPLES_PER_FRAME 10

// ============================================================================
// RADIO CONFIGURATION - MODIFY THESE FOR YOUR SETUP
// ============================================================================

// Transport selection (set one to 1, the other to 0)
#define USE_WIFI_UDP 0
#define USE_ESPNOW 1

// Wi-Fi Configuration
#define WIFI_SSID "YourWiFiSSID"
#define WIFI_PASSWORD "YourWiFiPassword"
#define DEST_IP_STR "192.168.1.100"  // RX radio IP address
#define UDP_PORT 30303

// ESP-NOW Configuration
#define ESPNOW_CHANNEL 1
static const uint8_t ESPNOW_PEER_MAC[6] = {0x02, 0xAA, 0xBB, 0x00, 0x00, 0x03};  // RX Radio MAC

// Custom MAC Configuration
#define USE_CUSTOM_MAC 1
static const uint8_t CUSTOM_STA_MAC_TX[6] = {0x02, 0xAA, 0xBB, 0x00, 0x00, 0x01};  // SPI Slave MAC

// Network buffer size
#define NET_QUEUE_SIZE 64



// DMA-capable buffers + transactions
static uint8_t rxbuf[QUEUED_XFERS][FRAME_BYTES] __attribute__((aligned(4)));
static spi_slave_transaction_t trx[QUEUED_XFERS];

// SPI stats (shared between tasks)
static volatile uint32_t frames_ok=0, frames_crc=0, frames_missed=0;
static volatile uint16_t last_idx=0; static volatile bool have_idx=false;
static volatile uint32_t last_rx_us=0, win_min_dt_us=UINT32_MAX, win_max_dt_us=0;
static volatile uint32_t win_ok=0, win_crc=0, win_missed=0;
static uint32_t t0_ms;

// Raw data output control
static volatile bool output_raw_data = true;

// ============================================================================
// RADIO NETWORKING VARIABLES
// ============================================================================

// Network queue for frames to transmit
static QueueHandle_t net_queue = NULL;

// Network statistics
static volatile uint32_t net_sent_ok = 0;
static volatile uint32_t net_send_fail = 0;
static volatile uint32_t net_queue_drops = 0;
static volatile uint32_t net_win_sent = 0;
static volatile uint32_t net_win_fail = 0;
static volatile uint32_t net_win_drops = 0;

// ESP-NOW command statistics
static volatile uint32_t espnow_commands_received = 0;
static volatile uint32_t espnow_command_errors = 0;


// ============================================================================
// ESP-NOW COMMAND STRUCTURES AND FUNCTIONS
// ============================================================================

// ESP-NOW command packet (expanded to 56 bytes for calibration commands)
struct __attribute__((packed)) ESPNowCommand {
    uint8_t magic[2];   // 0xCD, 0xD1 (Command magic)
    uint8_t command[48]; // Command string (null-terminated, expanded for cal_* commands)
    uint32_t timestamp; // Timestamp for deduplication
    uint16_t crc16;     // CRC16 of packet
};

// ESP-NOW response packet (compact single-line responses)
struct __attribute__((packed)) ESPNowResponse {
    uint8_t magic[2];   // 0xAB, 0xCD (Response magic)
    char response[220]; // Response string (null-terminated) - compact cal responses
    uint16_t crc16;     // CRC16 of packet
};

// Forward declarations
bool validate_espnow_command(const ESPNowCommand* cmd);
static bool create_espnow_response(ESPNowResponse* resp, const char* response_str);
static bool send_espnow_response(const char* response_str);
static void process_espnow_command(const ESPNowCommand* cmd);

// ============================================================================
// ESP-NOW COMMAND PROCESSING
// ============================================================================

// Case-insensitive string comparison helper
static bool strncmp_case_insensitive(const char* s1, const char* s2, size_t n) {
    for (size_t i = 0; i < n && s1[i] != '\0' && s2[i] != '\0'; i++) {
        char c1 = (s1[i] >= 'a' && s1[i] <= 'z') ? (s1[i] - 'a' + 'A') : s1[i];
        char c2 = (s2[i] >= 'a' && s2[i] <= 'z') ? (s2[i] - 'a' + 'A') : s2[i];
        if (c1 != c2) return false;
    }
    return true;
}

static void process_espnow_command(const ESPNowCommand* cmd) {
    char safe_command[49];
    strncpy(safe_command, (const char*)cmd->command, 48);
    safe_command[48] = '\0';
    
    Serial.printf("[SPI_SLAVE] ESP-NOW Command: %s\n", safe_command);
    espnow_commands_received++;
    
    // Convert command to uppercase for comparison (but send original to Teensy)
    String cmd_upper = String(safe_command);
    cmd_upper.toUpperCase();
    
    // PING command - forward to Teensy and send response back
    if (cmd_upper == "PING") {
        Serial.println("[SPI_SLAVE] PING received, forwarding to Teensy...");
        
        // Clear RX buffer
        while (Serial1.available()) Serial1.read();
        
        Serial1.println("PING");
        Serial1.flush();
        
        // Wait for PONG response
        unsigned long timeout = millis() + 3000;
        bool got_pong = false;
        String teensy_response = "";
        
        while (millis() < timeout) {
            if (Serial1.available()) {
                teensy_response = Serial1.readStringUntil('\n');
                teensy_response.trim();
                got_pong = true;
                break;
            }
            delay(10);
        }
        
        if (got_pong) {
            Serial.printf("[SPI_SLAVE] ✓ PONG received: '%s'\n", teensy_response.c_str());
            send_espnow_response(teensy_response.c_str());
        } else {
            Serial.println("[SPI_SLAVE] ✗ PONG timeout");
            send_espnow_response("PONG_TIMEOUT");
        }
    }
    // BLE connection status commands - forward to Teensy immediately
    else if (cmd_upper == "BLE_CONNECTED" || cmd_upper == "BLE_DISCONNECTED") {
        Serial.printf("[SPI_SLAVE] Forwarding BLE status '%s' to Remote Teensy...\n", safe_command);
        Serial1.println(safe_command);
        Serial1.flush();
        
        // Wait for response
        unsigned long timeout = millis() + 2000;
        while (millis() < timeout) {
            if (Serial1.available()) {
                String response = Serial1.readStringUntil('\n');
                Serial.printf("[SPI_SLAVE] Teensy Response: %s\n", response.c_str());
                break;
            }
            delay(10);
        }
    }
    // STATUS command - forward to Teensy and return response via ESP-NOW
    else if (cmd_upper == "STATUS") {
        Serial.println("[SPI_SLAVE] STATUS requested via ESP-NOW, forwarding to Teensy...");
        
        // Clear RX buffer
        while (Serial1.available()) Serial1.read();
        
        Serial1.println("STATUS");
        Serial1.flush();
        
        // Wait for response from Teensy
        unsigned long timeout = millis() + 3000;
        String teensy_response = "";
        
        while (millis() < timeout) {
            if (Serial1.available()) {
                teensy_response = Serial1.readStringUntil('\n');
                teensy_response.trim();
                if (teensy_response.length() > 0) {
                    break;
                }
            }
            delay(5);
        }
        
        if (teensy_response.length() > 0) {
            Serial.printf("[SPI_SLAVE] ✓ STATUS response: '%s'\n", teensy_response.c_str());
            send_espnow_response(teensy_response.c_str());
        } else {
            Serial.println("[SPI_SLAVE] ✗ STATUS timeout");
            send_espnow_response("STATUS:TIMEOUT");
        }
    }
    // Control commands - forward to Teensy
    else if (cmd_upper == "START" || cmd_upper == "STOP" ||
             cmd_upper == "RESTART" || cmd_upper == "RESET" ||
             cmd_upper == "LED_ON" || cmd_upper == "LED_OFF" ||
             cmd_upper == "MOCK_ON" || cmd_upper == "MOCK_OFF") {
        Serial.printf("[SPI_SLAVE] Forwarding '%s' to Teensy...\n", safe_command);
        Serial1.println(safe_command);
        Serial1.flush();
        
        // Wait for response
        unsigned long timeout = millis() + 2000;
        while (millis() < timeout) {
            if (Serial1.available()) {
                String response = Serial1.readStringUntil('\n');
                Serial.printf("[SPI_SLAVE] Teensy Response: %s\n", response.c_str());
                break;
            }
            delay(10);
        }
    }
    // Calibration commands - forward to Teensy, expect single-line response
    else if (cmd_upper.startsWith("CAL_")) {
        Serial.printf("[SPI_SLAVE] CAL: %s\n", safe_command);
        
        // Clear buffer
        while (Serial1.available()) Serial1.read();
        
        // Send command
        String cmd_to_teensy = String(safe_command);
        cmd_to_teensy.toUpperCase();
        Serial1.println(cmd_to_teensy);
        Serial1.flush();
        
        // Wait for single-line response (longer timeout for TARE/ADD which collect samples)
        unsigned long timeout = millis() + 5000;
        String response = "";
        
        while (millis() < timeout) {
            if (Serial1.available()) {
                response = Serial1.readStringUntil('\n');
                response.trim();
                if (response.length() > 0) {
                    Serial.printf("[SPI_SLAVE] << %s\n", response.c_str());
                    break;
                }
            }
            delay(5);
        }
        
        // Send response via ESP-NOW
        if (response.length() > 0) {
            send_espnow_response(response.c_str());
        } else {
            send_espnow_response("TIMEOUT");
        }
    }
    else {
        Serial.printf("[SPI_SLAVE] ✗ Unknown ESP-NOW command: '%s'\n", safe_command);
    }
}


// ============================================================================
// ESP-NOW COMMAND HELPER FUNCTIONS
// ============================================================================

bool validate_espnow_command(const ESPNowCommand* cmd) {
    if (cmd->magic[0] != 0xCD || cmd->magic[1] != 0xD1) {
        return false;
    }
    
    uint16_t expected_crc = crc16_ccitt_false((const uint8_t*)cmd, sizeof(ESPNowCommand) - 2);
    return cmd->crc16 == expected_crc;
}

// Forward declaration for ESP-NOW (needed by BMS battery send)
#if USE_ESPNOW
static bool espnow_initialized = false;
#endif

// ============================================================================
// BMS I2C HELPER FUNCTIONS
// ============================================================================

static float socFromLUT(float v) {
  const int n = (int)(sizeof(SOC_LUT)/sizeof(SOC_LUT[0]));
  if (v <= SOC_LUT[0].v) return SOC_LUT[0].soc;
  if (v >= SOC_LUT[n-1].v) return SOC_LUT[n-1].soc;
  for (int i=0; i<n-1; i++) {
    float v0=SOC_LUT[i].v, v1=SOC_LUT[i+1].v;
    if (v>=v0 && v<=v1) {
      float s0=SOC_LUT[i].soc, s1=SOC_LUT[i+1].soc;
      float t=(v-v0)/(v1-v0);
      return s0 + t*(s1-s0);
    }
  }
  return 0.0f;
}

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

static bool bqEnableIBAT() {
  uint8_t r14 = 0;
  if (!bqRead8(REG14_CTRL5, r14)) {
    Serial.println("[BMS] Failed to read REG14_CTRL5");
    return false;
  }
  uint8_t r14_before = r14;
  r14 |= (1 << 5);  // EN_IBAT = 1
  if (!bqWrite8(REG14_CTRL5, r14)) {
    Serial.println("[BMS] Failed to write REG14_CTRL5");
    return false;
  }
  
  // Verify write
  uint8_t r14_verify = 0;
  if (bqRead8(REG14_CTRL5, r14_verify)) {
    if ((r14_verify & (1 << 5)) == 0) {
      Serial.printf("[BMS] EN_IBAT not set! REG14: 0x%02X (was 0x%02X)\n", r14_verify, r14_before);
      return false;
    }
    Serial.printf("[BMS] EN_IBAT enabled (REG14: 0x%02X)\n", r14_verify);
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
      Serial.printf("[BMS] ADC enabled (REG2E: 0x%02X -> 0x%02X)\n", r2e_before, r2e);
    } else {
      Serial.println("[BMS] Failed to write REG2E_ADC_CTRL");
    }
  } else {
    Serial.println("[BMS] Failed to read REG2E_ADC_CTRL");
  }

  // Clear ADC disable bits (REG2F)
  if (bqRead8(REG2F_ADC_DIS0, r2f)) {
    uint8_t r2f_before = r2f;
    r2f &= ~(1 << 4);  // Clear VBAT disable bit (if applicable)
    if (bqWrite8(REG2F_ADC_DIS0, r2f)) {
      Serial.printf("[BMS] ADC disable bits cleared (REG2F: 0x%02X -> 0x%02X)\n", r2f_before, r2f);
    } else {
      Serial.println("[BMS] Failed to write REG2F_ADC_DIS0");
    }
  } else {
    Serial.println("[BMS] Failed to read REG2F_ADC_DIS0");
  }
  
  // Enable IBAT sensing
  if (!bqEnableIBAT()) {
    Serial.println("[BMS] Failed to enable IBAT sensing");
  }

  delay(20);  // Allow ADC to settle
}

static float applyVbatCal(float vraw) {
  return vraw * VBAT_GAIN + VBAT_OFFSET_V;
}

static bool readVBATraw(float &vbatV) {
  // Auto-detect format ONCE on first successful read, then keep using it
  static uint8_t vbat_format = 0;  // 0=unknown, 1=MSB, 2=LSB
  
  // If format already detected, use it directly
  if (vbat_format == 1) {
    uint16_t raw = 0;
    if (!bqRead16_MSB(REG3B_VBAT_ADC, raw)) return false;
    vbatV = raw / 1000.0f;
    return true;
  } else if (vbat_format == 2) {
    uint16_t raw = 0;
    if (!bqRead16_LSB(REG3B_VBAT_ADC, raw)) return false;
    vbatV = raw / 1000.0f;
    return true;
  }
  
  // First call - detect format
  uint16_t raw_msb = 0, raw_lsb = 0;
  bool msb_ok = bqRead16_MSB(REG3B_VBAT_ADC, raw_msb);
  bool lsb_ok = bqRead16_LSB(REG3B_VBAT_ADC, raw_lsb);
  
  if (msb_ok && lsb_ok) {
    // Both readable - choose the one that makes sense (should be 5000-7200 mV for 2S LiFePO4)
    float v_msb = raw_msb / 1000.0f;
    float v_lsb = raw_lsb / 1000.0f;
    
    if (v_msb >= 5.0f && v_msb <= 7.5f) {
      vbat_format = 1;  // MSB format is correct
      vbatV = v_msb;
      Serial.printf("[BMS] ✓ VBAT format detected: MSB (%.3fV)\n", v_msb);
    } else if (v_lsb >= 5.0f && v_lsb <= 7.5f) {
      vbat_format = 2;  // LSB format is correct
      vbatV = v_lsb;
      Serial.printf("[BMS] ✓ VBAT format detected: LSB (%.3fV)\n", v_lsb);
    } else {
      // Neither makes sense - prefer MSB (datasheet says MSB-first)
      vbat_format = 1;
      vbatV = v_msb;
      Serial.printf("[BMS] ⚠ VBAT format unclear, using MSB (%.3fV)\n", v_msb);
    }
    return true;
  } else if (msb_ok) {
    vbat_format = 1;
    vbatV = raw_msb / 1000.0f;
    Serial.printf("[BMS] ✓ VBAT format: MSB only (%.3fV)\n", vbatV);
    return true;
  } else if (lsb_ok) {
    vbat_format = 2;
    vbatV = raw_lsb / 1000.0f;
    Serial.printf("[BMS] ✓ VBAT format: LSB only (%.3fV)\n", vbatV);
    return true;
  }
  
  return false;
}

static bool bqDisableWatchdog() {
  uint8_t r10 = 0;
  if (!bqRead8(REG10_CTRL1, r10)) return false;
  r10 &= ~0x07;  // WD=0
  return bqWrite8(REG10_CTRL1, r10);
}

static bool bmsInit() {
  Serial.println("[BMS] Initializing BQ25792 for 2S LiFePO4...");
  
  if (!bqProbe()) {
    Serial.println("[BMS] ✗ BQ charger not found at address 0x6B");
    return false;
  }
  Serial.println("[BMS] ✓ BQ25792 found at address 0x6B");
  
  // 1. Disable watchdog
  if (!bqDisableWatchdog()) {
    Serial.println("[BMS] ⚠ Failed to disable watchdog (continuing anyway)");
  } else {
    Serial.println("[BMS] ✓ Watchdog disabled");
  }
  
  // 2. Set cell count to 2S (REG0A, CELL[1:0] = 1)
  uint8_t r0a = 0;
  if (bqRead8(REG0A_CELL, r0a)) {
    uint8_t r0a_before = r0a;
    r0a &= ~0x03;  // Clear CELL[1:0]
    r0a |= 0x01;   // Set CELL = 1 (2-series)
    if (bqWrite8(REG0A_CELL, r0a)) {
      Serial.printf("[BMS] ✓ Cell count set to 2S (REG0A: 0x%02X -> 0x%02X)\n", r0a_before, r0a);
    }
  }
  
  // 3. Set VSYSMIN (REG00): 6000 mV = (6000-2500)/250 = 14 = 0x0E
  uint8_t vsysmin_code = (6000 - 2500) / 250;  // 14
  if (bqWrite8(REG00_VSYSMIN, vsysmin_code & 0x3F)) {
    Serial.printf("[BMS] ✓ VSYSMIN set to 6000mV (code: 0x%02X)\n", vsysmin_code);
  }
  
  // 4. Set VREG (REG01): 7200 mV = 720 = 0x02D0 (LSB-first: 0xD0, 0x02)
  uint16_t vreg_code = 7200 / 10;  // 720
  if (bqWrite16_LSB(REG01_VREG, vreg_code)) {
    Serial.printf("[BMS] ✓ VREG set to 7200mV (code: 0x%04X)\n", vreg_code);
  }
  
  // 5. Set ICHG (REG03): 2500 mA = 250 = 0x00FA (LSB-first: 0xFA, 0x00)
  uint16_t ichg_code = 2500 / 10;  // 250
  if (bqWrite16_LSB(REG03_ICHG, ichg_code)) {
    Serial.printf("[BMS] ✓ ICHG set to 2500mA (code: 0x%04X)\n", ichg_code);
  }
  
  // 6. Set IINDPM (REG06): 2000 mA for USB phone charger compatibility
  // Most USB-C chargers provide 1.5-2A, some provide 3A
  uint16_t iindpm_code = 2000 / 10;  // 200
  if (bqWrite16_LSB(REG06_IINDPM, iindpm_code)) {
    Serial.printf("[BMS] ✓ IINDPM set to 2000mA for USB compatibility (code: 0x%04X)\n", iindpm_code);
  }
  
  // 7. Set ITERM (REG09): 200 mA = 200/40 = 5
  uint8_t iterm_val = 0;
  if (bqRead8(REG09_ITERM, iterm_val)) {
    uint8_t iterm_before = iterm_val;
    uint8_t iterm_code = 200 / 40;  // 5
    iterm_val &= ~0x1F;  // Clear ITERM bits (assuming bits[4:0])
    iterm_val |= (iterm_code & 0x1F);
    if (bqWrite8(REG09_ITERM, iterm_val)) {
      Serial.printf("[BMS] ✓ ITERM set to 200mA (REG09: 0x%02X -> 0x%02X)\n", iterm_before, iterm_val);
    }
  }
  
  // 8. Enable ADC and IBAT sensing
  bqAdcEnable();
  
  Serial.println("[BMS] ✓ BQ25792 initialization complete!");
  return true;
}

static bool readIBAT(float &ibatA, int16_t &raw_mA) {
  uint16_t u = 0;
  
  // If register already detected, use it directly
  if (ibat_working_reg != 0) {
    if (!bqRead16_LSB(ibat_working_reg, u)) {
      return false;
    }
    int16_t s = (int16_t)u;
    raw_mA = s;
    ibatA = (float)s / 1000.0f;
    return true;
  }
  
  // First call - detect which register works
  uint16_t u32 = 0, u33 = 0;
  bool r32_ok = bqRead16_LSB(REG32_IBAT, u32);
  bool r33_ok = bqRead16_LSB(REG33_IBAT, u33);
  
  if (r32_ok && r33_ok) {
    // Both work - prefer REG32 (reference implementation)
    ibat_working_reg = REG32_IBAT;
    u = u32;
    Serial.printf("[BMS] ✓ IBAT register detected: REG32 (0x%04X)\n", u32);
  } else if (r32_ok) {
    ibat_working_reg = REG32_IBAT;
    u = u32;
    Serial.printf("[BMS] ✓ IBAT register: REG32 only (0x%04X)\n", u32);
  } else if (r33_ok) {
    ibat_working_reg = REG33_IBAT;
    u = u33;
    Serial.printf("[BMS] ✓ IBAT register: REG33 only (0x%04X)\n", u33);
  } else {
    return false;
  }
  
  int16_t s = (int16_t)u;
  raw_mA = s;
  ibatA = (float)s / 1000.0f;
  return true;
}

static void bmsUpdateReading() {
  if (!bqProbe()) return;
  
  float vraw = 0;
  if (!readVBATraw(vraw)) return;
  
  float vcal = applyVbatCal(vraw);
  
  // EMA filter for voltage
  if (isnan(vBatFiltCal)) {
    vBatFiltCal = vcal;
  } else {
    vBatFiltCal = EMA_ALPHA * vcal + (1.0f - EMA_ALPHA) * vBatFiltCal;
  }
  
  currentSoc = socFromLUT(vBatFiltCal);
  
  // Read and filter battery current (for SOC calculation, but use fast check for status)
  float ibat = 0;
  int16_t raw_mA = 0;
  if (readIBAT(ibat, raw_mA)) {
    if (isnan(iBatFilt)) {
      iBatFilt = ibat;
    } else {
      iBatFilt = EMA_ALPHA * ibat + (1.0f - EMA_ALPHA) * iBatFilt;
    }
  }
}

// Charging status detection with stable debouncing
static void checkChargingStatus() {
  if (!bqProbe() || !bmsInitialized) {
    static uint32_t lastErrorLog = 0;
    if (millis() - lastErrorLog >= 5000) {
      Serial.printf("[BMS] checkChargingStatus: BMS not ready (bqProbe=%d, bmsInitialized=%d)\n", bqProbe(), bmsInitialized);
      lastErrorLog = millis();
    }
    return;
  }
  
  float ibat = 0;
  int16_t raw_mA = 0;
  if (!readIBAT(ibat, raw_mA)) {
    static uint32_t lastReadError = 0;
    if (millis() - lastReadError >= 5000) {
      Serial.println("[BMS] checkChargingStatus: Failed to read IBAT");
      lastReadError = millis();
    }
    return;  // Exit if read fails
  }
  
  // Use slower EMA for stable charging detection (prevents oscillation)
  // Lower alpha = smoother/slower response, prevents rapid toggling
  const float EMA_ALPHA_FAST = 0.3f;  // Slower filter to avoid oscillation
  if (isnan(iBatFiltFast)) {
    iBatFiltFast = ibat;  // Initialize on first read
    Serial.printf("[BMS] checkChargingStatus: Initialized iBatFiltFast=%.3fA (raw=%.3fA, raw_mA=%d)\n", iBatFiltFast, ibat, raw_mA);
  } else {
    iBatFiltFast = EMA_ALPHA_FAST * ibat + (1.0f - EMA_ALPHA_FAST) * iBatFiltFast;
  }
  
  // Thresholds with wide hysteresis to prevent oscillation
  // USB chargers may pulse during charging, so need generous margins
  const float CHARGE_START_THRESHOLD = 0.05f;   // 50mA - must exceed this to START charging
  const float CHARGE_STOP_THRESHOLD = 0.02f;    // 20mA - must drop below this to STOP charging
  
  // Asymmetric debounce: Quick to detect charging, slow to stop
  const uint32_t START_DEBOUNCE_MS = 500;       // 500ms to start (quick to show LED)
  const uint32_t STOP_DEBOUNCE_MS = 2000;       // 2 seconds to stop (prevents oscillation)
  
  uint32_t now = millis();
  
  // Debouncing state variables (static to persist across calls)
  static bool desiredChargingState = false;
  static uint32_t desiredStateStartMs = 0;
  static uint32_t lastStateChangeMs = 0;
  
  // Initialize timers on first run
  if (lastStateChangeMs == 0) {
    lastStateChangeMs = now;
    desiredStateStartMs = now;
  }
  
  // Determine desired charging state using filtered value with hysteresis
  bool newDesiredState;
  if (isCharging) {
    // Currently charging - require current to drop significantly before stopping
    // This prevents stopping during brief current dips
    newDesiredState = (iBatFiltFast > CHARGE_STOP_THRESHOLD);
  } else {
    // Not charging - require current to exceed threshold before starting
    newDesiredState = (iBatFiltFast > CHARGE_START_THRESHOLD);
  }
  
  // Track when desired state changes
  if (newDesiredState != desiredChargingState) {
    desiredChargingState = newDesiredState;
    desiredStateStartMs = now;
  }
  
  // Calculate how long desired state has been stable
  uint32_t desiredStateHoldMs = now - desiredStateStartMs;
  
  // Use asymmetric debounce - quick to start, slow to stop
  uint32_t requiredHoldMs = desiredChargingState ? START_DEBOUNCE_MS : STOP_DEBOUNCE_MS;
  
  // Debouncing: Only allow state change if desired state has been stable for required time
  bool stateNeedsChange = (desiredChargingState != isCharging);
  bool canChangeState = (desiredStateHoldMs >= requiredHoldMs);
  
  // Log current status every 2 seconds for debugging
  static uint32_t lastLogMs = 0;
  if (now - lastLogMs >= 2000) {
    Serial.printf("[BMS] IBAT: raw=%.3fA (raw_mA=%d), filtered=%.3fA, isCharging=%d, desired=%d, hold=%lums (need %lums)\n", 
                  ibat, raw_mA, iBatFiltFast, isCharging, desiredChargingState, desiredStateHoldMs, requiredHoldMs);
    lastLogMs = now;
  }
  
  // Send charging status to remote Teensy when it changes (with debouncing)
  if (stateNeedsChange && canChangeState) {
    isCharging = desiredChargingState;
    lastStateChangeMs = now;  // Reset change timer
    
    if (isCharging) {
      Serial1.println("BATTERY_CHARGING");
      Serial1.flush();
      Serial.printf("[BMS] ✓ CHARGING DETECTED (IBAT=%.3fA, raw_mA=%d) → Sending BATTERY_CHARGING to Remote Teensy\n", iBatFiltFast, raw_mA);
    } else {
      Serial1.println("BATTERY_NOT_CHARGING");
      Serial1.flush();
      Serial.printf("[BMS] ✗ CHARGING STOPPED (IBAT=%.3fA, raw_mA=%d) → Sending BATTERY_NOT_CHARGING to Remote Teensy\n", iBatFiltFast, raw_mA);
    }
  }
}

static bool sendBatteryPacket() {
  if (!espnow_initialized) return false;
  if (isnan(vBatFiltCal)) return false;
  
  ESPNowBattery pkt;
  pkt.magic[0] = 0xBA;
  pkt.magic[1] = 0xBB;
  pkt.voltage = vBatFiltCal;
  pkt.soc = currentSoc;
  pkt.crc16 = crc16_ccitt_false((const uint8_t*)&pkt, sizeof(ESPNowBattery) - 2);
  
  esp_err_t result = esp_now_send(ESPNOW_PEER_MAC, (uint8_t*)&pkt, sizeof(pkt));
  return (result == ESP_OK);
}

// Network objects
#if USE_WIFI_UDP
static WiFiUDP udp;
static IPAddress dest_ip;
static bool wifi_connected = false;
#endif

#if USE_ESPNOW
// espnow_initialized declared earlier (needed by BMS battery send)
static volatile uint32_t espnow_send_success = 0;
static volatile uint32_t espnow_send_fail = 0;
#endif

// Radio sequence number (independent of inner frame_idx)
static uint16_t radio_seq = 0;

// ============================================================================
// CUSTOM MAC ADDRESS FUNCTIONS
// ============================================================================

static void set_custom_mac_if_enabled() {
#if USE_CUSTOM_MAC
  WiFi.mode(WIFI_STA);
  esp_wifi_set_mac(WIFI_IF_STA, (uint8_t*)CUSTOM_STA_MAC_TX);
  
  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  Serial.printf("[TX] Custom STA MAC set: %02X:%02X:%02X:%02X:%02X:%02X\n", 
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
#endif
}

// ============================================================================
// WIFI UDP FUNCTIONS
// ============================================================================

#if USE_WIFI_UDP
static void wifi_init() {
  set_custom_mac_if_enabled();
  
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("[TX] Connecting to WiFi: %s", WIFI_SSID);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 50) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifi_connected = true;
    dest_ip.fromString(DEST_IP_STR);
    udp.begin(0);  // Use any available port for sending
    
    Serial.printf("\n[TX] WiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("[TX] Target: %s:%d\n", DEST_IP_STR, UDP_PORT);
  } else {
    Serial.println("\n[TX] WiFi connection failed!");
    wifi_connected = false;
  }
}

static void wifi_send_frame(const uint8_t* frame_data) {
  if (!wifi_connected || WiFi.status() != WL_CONNECTED) {
    net_send_fail++;
    net_win_fail++;
    return;
  }
  
  // Build radio header
  RadioHdr hdr;
  hdr.magic = RADIO_MAGIC;
  hdr.seq = radio_seq++;
  hdr.len = FRAME_BYTES;
  
  // Send UDP packet (header + payload)
  udp.beginPacket(dest_ip, UDP_PORT);
  udp.write((uint8_t*)&hdr, sizeof(hdr));
  udp.write(frame_data, FRAME_BYTES);
  
  if (udp.endPacket()) {
    net_sent_ok++;
    net_win_sent++;
  } else {
    net_send_fail++;
    net_win_fail++;
  }
}

static void wifi_check_connection() {
  static uint32_t last_check = 0;
  uint32_t now = millis();
  
  if (now - last_check > 5000) {  // Check every 5 seconds
    if (WiFi.status() != WL_CONNECTED && wifi_connected) {
      Serial.println("[TX] WiFi disconnected, attempting reconnect...");
      wifi_connected = false;
      WiFi.reconnect();
    } else if (WiFi.status() == WL_CONNECTED && !wifi_connected) {
      Serial.println("[TX] WiFi reconnected!");
      wifi_connected = true;
    }
    last_check = now;
  }
}
#endif

// ============================================================================
// ESP-NOW FUNCTIONS
// ============================================================================

#if USE_ESPNOW
static void espnow_send_callback(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    espnow_send_success++;
  } else {
    espnow_send_fail++;
    net_send_fail++;
    net_win_fail++;
  }
}

static void espnow_recv_callback(const esp_now_recv_info *recv_info, const uint8_t *data, int len) {
  // Check if this is a command packet
  if (len == sizeof(ESPNowCommand)) {
    const ESPNowCommand* cmd = (const ESPNowCommand*)data;
    if (validate_espnow_command(cmd)) {
      process_espnow_command(cmd);
      return;
    } else {
      espnow_command_errors++;
      Serial.printf("[SPI_SLAVE] ✗ Invalid ESP-NOW command packet received\n");
      return;
    }
  }
  
  // If not a command packet, ignore (we don't expect data frames on SPI slave)
  Serial.printf("[SPI_SLAVE] Received unexpected ESP-NOW packet (len=%d)\n", len);
}

static bool create_espnow_response(ESPNowResponse* resp, const char* response_str) {
  memset(resp, 0, sizeof(ESPNowResponse));
  resp->magic[0] = 0xAB;
  resp->magic[1] = 0xCD;
  strncpy(resp->response, response_str, 219);
  resp->response[219] = '\0';
  
  // Calculate CRC16
  resp->crc16 = crc16_ccitt_false((const uint8_t*)resp, sizeof(ESPNowResponse) - 2);
  return true;
}

static bool send_espnow_response(const char* response_str) {
  if (!espnow_initialized) return false;
  
  ESPNowResponse resp;
  if (!create_espnow_response(&resp, response_str)) return false;
  
  esp_err_t result = esp_now_send(ESPNOW_PEER_MAC, (uint8_t*)&resp, sizeof(resp));
  if (result == ESP_OK) {
    Serial.printf("[SPI_SLAVE] ✓ Response '%s' sent via ESP-NOW\n", response_str);
    return true;
  }
  return false;
}

static void espnow_init() {
  set_custom_mac_if_enabled();
  
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("[TX] ESP-NOW init failed!");
    return;
  }
  
  esp_now_register_send_cb(espnow_send_callback);
  esp_now_register_recv_cb(espnow_recv_callback);
  
  // Add peer
  esp_now_peer_info_t peer_info = {};
  memcpy(peer_info.peer_addr, ESPNOW_PEER_MAC, 6);
  peer_info.channel = ESPNOW_CHANNEL;
  peer_info.encrypt = false;
  
  if (esp_now_add_peer(&peer_info) == ESP_OK) {
    espnow_initialized = true;
    Serial.printf("[TX] ESP-NOW initialized, channel %d\n", ESPNOW_CHANNEL);
    Serial.printf("[TX] Peer MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  ESPNOW_PEER_MAC[0], ESPNOW_PEER_MAC[1], ESPNOW_PEER_MAC[2],
                  ESPNOW_PEER_MAC[3], ESPNOW_PEER_MAC[4], ESPNOW_PEER_MAC[5]);
  } else {
    Serial.println("[TX] Failed to add ESP-NOW peer!");
  }
}

static void espnow_send_frame(const uint8_t* frame_data) {
  if (!espnow_initialized) {
    net_send_fail++;
    net_win_fail++;
    return;
  }
  
  // Send raw 144-byte frame (ESP-NOW has its own reliability)
  esp_err_t result = esp_now_send(ESPNOW_PEER_MAC, frame_data, FRAME_BYTES);
  
  if (result == ESP_OK) {
    net_sent_ok++;
    net_win_sent++;
  } else {
    net_send_fail++;
    net_win_fail++;
  }
}
#endif

// ============================================================================
// NETWORK TX TASK
// ============================================================================

static void net_tx_task(void* param) {
  uint8_t frame_buffer[FRAME_BYTES];
  
  Serial.println("[TX] Network TX task started");
  
  while (true) {
    // Wait for frame from SPI RX task
    if (xQueueReceive(net_queue, frame_buffer, portMAX_DELAY) == pdTRUE) {
      
#if USE_WIFI_UDP
      wifi_send_frame(frame_buffer);
      wifi_check_connection();
#elif USE_ESPNOW
      espnow_send_frame(frame_buffer);
#endif
      
    }
  }
}

// ============================================================================
// SPI FUNCTIONS (MODIFIED)
// ============================================================================

static void requeue_all() {
  for (int i=0;i<QUEUED_XFERS;i++) {
    memset(&trx[i], 0, sizeof(trx[i]));
    trx[i].length    = FRAME_BYTES * 8;
    trx[i].rx_buffer = rxbuf[i];
    trx[i].user      = (void*)(intptr_t)i;  // remember which buffer
    ESP_ERROR_CHECK(spi_slave_queue_trans(SPI2_HOST, &trx[i], portMAX_DELAY));
  }
}

static void spi_rx_task(void*){
  while (true) {
    spi_slave_transaction_t* ret;
    esp_err_t e = spi_slave_get_trans_result(SPI2_HOST, &ret, portMAX_DELAY);
    if (e != ESP_OK) continue;
    int idx = (int)(intptr_t)ret->user;
    InnerFrame* f = (InnerFrame*)rxbuf[idx];

    uint32_t now_us = micros();

    bool ok = (f->sync[0]==0xA5 && f->sync[1]==0x5A);
    if (ok) {
      uint16_t want = f->crc16;
      uint16_t got  = crc16_ccitt_false((uint8_t*)f, offsetof(InnerFrame, crc16));
      ok = (want == got);
    }
    if (!ok) {
      frames_crc++; win_crc++;
    } else {
      if (have_idx) {
        uint16_t expected = last_idx + 1;
        uint16_t missed = (uint16_t)(f->frame_idx - expected);
        if (missed) { frames_missed += missed; win_missed += missed; }
      } else {
        have_idx = true;
      }
      last_idx = f->frame_idx;

      if (last_rx_us != 0) {
        uint32_t dt = now_us - last_rx_us;
        if (dt < win_min_dt_us) win_min_dt_us = dt;
        if (dt > win_max_dt_us) win_max_dt_us = dt;
      }
      last_rx_us = now_us;

      frames_ok++; win_ok++;
      
      // Queue frame for network transmission (non-blocking)
      if (net_queue != NULL) {
        if (xQueueSend(net_queue, rxbuf[idx], 0) != pdTRUE) {
          net_queue_drops++;
          net_win_drops++;
        }
      }
      
      // Output raw data if debug mode is enabled
      if (output_raw_data) {
        process_load_cell_data_compact(f);
      }
    }

    // immediately re-queue the same buffer so we never run dry
    ESP_ERROR_CHECK(spi_slave_queue_trans(SPI2_HOST, ret, portMAX_DELAY));
  }
}

static void stats_task(void*){
  const uint32_t PRINT_MS = 5000;
  uint32_t last_ms = millis();
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(100));
    uint32_t now = millis();
    if (now - last_ms < PRINT_MS) continue;

    double tsec = (now - t0_ms) / 1000.0;
    uint32_t ok = frames_ok, crc = frames_crc, miss = frames_missed;
    uint32_t w_ok = win_ok, w_crc = win_crc, w_miss = win_missed;
    uint32_t dtmin = (win_min_dt_us==UINT32_MAX?0:win_min_dt_us);
    uint32_t dtmax = win_max_dt_us;

    double kbps_total = (ok * FRAME_BYTES * 8.0) / tsec / 1000.0;
    double sps_total  = (ok * SAMPLES_PER_FRAME * 4) / tsec; // 4 channels per sample
    double kbps_win   = (w_ok * FRAME_BYTES * 8.0) / (PRINT_MS/1000.0) / 1000.0;
    double sps_win    = (w_ok * SAMPLES_PER_FRAME * 4) / (PRINT_MS/1000.0); // 4 channels per sample

    // Network stats
    uint32_t net_ok = net_sent_ok, net_fail = net_send_fail, net_drops = net_queue_drops;
    uint32_t net_w_ok = net_win_sent, net_w_fail = net_win_fail, net_w_drops = net_win_drops;
    
    Serial.printf("[TX] T=%.1fs | SPI: ok=%lu crc=%lu miss=%lu | NET: sent=%lu fail=%lu drops=%lu | win5s SPI: ok=%lu crc=%lu miss=%lu | NET: sent=%lu fail=%lu drops=%lu | rate=%.1f kb/s (5s=%.1f) | sps=%.1f (5s=%.1f) | dt_us[min=%lu max=%lu] last_idx=%u\n",
      tsec,(unsigned long)ok,(unsigned long)crc,(unsigned long)miss,
      (unsigned long)net_ok,(unsigned long)net_fail,(unsigned long)net_drops,
      (unsigned long)w_ok,(unsigned long)w_crc,(unsigned long)w_miss,
      (unsigned long)net_w_ok,(unsigned long)net_w_fail,(unsigned long)net_w_drops,
      kbps_total,kbps_win,sps_total,sps_win,
      (unsigned long)dtmin,(unsigned long)dtmax,(unsigned)last_idx);
    
    // Battery status log
    Serial.printf("🔋 BAT | VBAT=%.2fV SOC=%.1f%% | BMS=%s\n",
      isnan(vBatFiltCal) ? 0.0f : vBatFiltCal, currentSoc,
      bmsInitialized ? "OK" : "N/A");

    // reset 5 s window
    win_ok=win_crc=win_missed=0;
    win_min_dt_us=UINT32_MAX; win_max_dt_us=0;
    net_win_sent=net_win_fail=net_win_drops=0;
    last_ms = now;
  }
}


// Process load cell data from received frame (compact format)
void process_load_cell_data_compact(InnerFrame* f) {
  for (int sample = 0; sample < SAMPLES_PER_FRAME; sample++) {
    int32_t lc1, lc2, lc3, lc4;
    extract_load_cell_sample(f->samples, sample, &lc1, &lc2, &lc3, &lc4);
    
    // Compact format: DATA:FRAME:SAMPLE:LC1:LC2:LC3:LC4
    Serial.printf("DATA:%u:%d:%ld:%ld:%ld:%ld\n", 
                  f->frame_idx, sample, lc1, lc2, lc3, lc4);
  }
}


// Handle serial commands
void handle_serial_commands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toUpperCase();
    
    if (command.length() == 0) return;
    
    Serial.printf("[SPI_SLAVE] Command: %s\n", command.c_str());
    
    // BLE connection status commands - forward to Teensy immediately
    if (command == "BLE_CONNECTED" || command == "BLE_DISCONNECTED") {
      Serial.printf("[SPI_SLAVE] Forwarding BLE status '%s' to Remote Teensy...\n", command.c_str());
      Serial1.println(command);
      Serial1.flush();
      
      unsigned long timeout = millis() + 2000;
      while (millis() < timeout) {
        if (Serial1.available()) {
          String response = Serial1.readStringUntil('\n');
          Serial.printf("[SPI_SLAVE] Teensy Response: %s\n", response.c_str());
          break;
        }
        delay(10);
      }
    }
    // Teensy control commands
    else if (command == "START" || command == "STOP" || command == "RESTART" || command == "RESET" ||
        command == "LED_ON" || command == "LED_OFF" ||
        command == "MOCK_ON" || command == "MOCK_OFF") {
      Serial.printf("[SPI_SLAVE] Forwarding '%s' to Teensy...\n", command.c_str());
      Serial1.println(command);
      Serial1.flush();
      
      unsigned long timeout = millis() + 2000;
      while (millis() < timeout) {
        if (Serial1.available()) {
          String response = Serial1.readStringUntil('\n');
          Serial.printf("[SPI_SLAVE] Teensy Response: %s\n", response.c_str());
          break;
        }
        delay(10);
      }
    }
    // PING test
    else if (command == "PING") {
      Serial.println("[SPI_SLAVE] PING-PONG TEST");
      
      // Clear RX buffer
      while (Serial1.available()) Serial1.read();
      
      unsigned long start_time = millis();
      Serial1.println("PING");
      Serial1.flush();
      
      unsigned long timeout = millis() + 3000;
      bool got_response = false;
      String response_text = "";
      
      while (millis() < timeout) {
        if (Serial1.available()) {
          char c = Serial1.read();
          if (c == '\n' || c == '\r') {
            if (response_text.length() > 0) {
              got_response = true;
              break;
            }
          } else {
            response_text += c;
          }
        }
        delay(1);
      }
      
      unsigned long round_trip = millis() - start_time;
      if (got_response) {
        Serial.printf("[SPI_SLAVE] ✓ PONG received: '%s' (%lu ms)\n", response_text.c_str(), round_trip);
      } else {
        Serial.printf("[SPI_SLAVE] ✗ TIMEOUT after %lu ms\n", round_trip);
      }
    }
    // ESP32 control commands
    else if (command == "DEBUG_ON") {
      output_raw_data = true;
      Serial.println("[SPI_SLAVE] ✓ Raw data output enabled");
    } else if (command == "DEBUG_OFF") {
      output_raw_data = false;
      Serial.println("[SPI_SLAVE] ✓ Raw data output disabled");
    } else if (command == "STATUS") {
      Serial.println("\n[SPI_SLAVE] === STATUS ===");
      Serial.printf("  SPI Frames: OK=%lu CRC=%lu Missed=%lu\n", (unsigned long)frames_ok, (unsigned long)frames_crc, (unsigned long)frames_missed);
      Serial.printf("  Network: Sent=%lu Fail=%lu Drops=%lu\n", (unsigned long)net_sent_ok, (unsigned long)net_send_fail, (unsigned long)net_queue_drops);
      Serial.printf("  ESP-NOW: Cmds=%lu Errors=%lu\n", (unsigned long)espnow_commands_received, (unsigned long)espnow_command_errors);
      Serial.printf("  BMS: %s | VBAT=%.2fV SOC=%.1f%%\n", bmsInitialized ? "OK" : "N/A", 
                    isnan(vBatFiltCal) ? 0.0f : vBatFiltCal, currentSoc);
      Serial.printf("  Raw data: %s | Heap: %lu bytes\n", output_raw_data ? "ON" : "OFF", ESP.getFreeHeap());
      Serial.println("==========================\n");
    } else if (command == "HELP") {
      Serial.println("\n[SPI_SLAVE] === AVAILABLE COMMANDS ===");
      Serial.println("  Teensy: START, STOP, RESTART, RESET, PING");
      Serial.println("  ESP32:  DEBUG_ON/OFF, STATUS, HELP");
      Serial.println("========================================\n");
    } else {
      Serial.printf("[SPI_SLAVE] ✗ Unknown command: '%s'\n", command.c_str());
    }
  }
}

void setup() {
  Serial.begin(921600); // fast prints so we never stall
  delay(200);
  Serial.println("\n[TX] SPI SLAVE + RADIO TX starting");

  // Initialize I2C for BMS
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_HZ);
  Wire.setTimeOut(50);
  Serial.printf("[BMS] I2C initialized (SDA=GPIO%d, SCL=GPIO%d, %lu Hz)\n", I2C_SDA, I2C_SCL, I2C_HZ);
  
  // Initialize BMS
  bmsInitialized = bmsInit();

  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Create network queue
  net_queue = xQueueCreate(NET_QUEUE_SIZE, FRAME_BYTES);
  if (net_queue == NULL) {
    Serial.println("[TX] Failed to create network queue!");
    while(1) delay(1000);
  }

  // Initialize networking
#if USE_WIFI_UDP
  Serial.println("[TX] Initializing Wi-Fi UDP...");
  wifi_init();
#elif USE_ESPNOW
  Serial.println("[TX] Initializing ESP-NOW...");
  espnow_init();
#endif

  // Initialize SPI
  spi_bus_config_t bus{};
  bus.mosi_io_num = PIN_MOSI;
  bus.miso_io_num = PIN_MISO;
  bus.sclk_io_num = PIN_SCLK;
  bus.max_transfer_sz = FRAME_BYTES;

  spi_slave_interface_config_t slv{};
  slv.mode = 0;
  slv.spics_io_num = PIN_CS;
  slv.queue_size = QUEUED_XFERS;

  ESP_ERROR_CHECK(spi_slave_initialize(SPI2_HOST, &bus, &slv, SPI_DMA_CH_AUTO));
  gpio_set_pull_mode((gpio_num_t)PIN_CS,   GPIO_PULLUP_ONLY);
  gpio_set_pull_mode((gpio_num_t)PIN_MOSI, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode((gpio_num_t)PIN_SCLK, GPIO_PULLUP_ONLY);

  requeue_all();

  // Initialize Serial1 for Teensy communication (9600 baud for software serial TX compatibility)
  Serial1.setRxBufferSize(512);
  Serial1.begin(9600, SERIAL_8N1, TEENSY_UART_RX_PIN, TEENSY_UART_TX_PIN);
  Serial.printf("[SPI_SLAVE] Serial1: 9600 bps (TX=GPIO%d → Teensy, RX=GPIO%d ← Teensy)\n", 
                TEENSY_UART_TX_PIN, TEENSY_UART_RX_PIN);
  
  // Send initial charging status after Serial1 is ready (with delay for first reading)
  if (bmsInitialized) {
    delay(300);  // Allow BMS to settle and Serial1 to be ready
    // Do multiple readings to initialize filters
    for (int i = 0; i < 3; i++) {
      bmsUpdateReading();
      delay(50);
    }
    // Force an initial check and send status
    checkChargingStatus();
    // Also send current status immediately (in case checkChargingStatus didn't send it)
    if (!isnan(iBatFiltFast)) {
      if (isCharging) {
        Serial1.println("BATTERY_CHARGING");
        Serial1.flush();
        Serial.printf("[BMS] Initial status: Charging (IBAT=%.3fA)\n", iBatFiltFast);
      } else {
        Serial1.println("BATTERY_NOT_CHARGING");
        Serial1.flush();
        Serial.printf("[BMS] Initial status: Not charging (IBAT=%.3fA)\n", iBatFiltFast);
      }
    } else {
      // If reading failed, default to not charging
      Serial1.println("BATTERY_NOT_CHARGING");
      Serial1.flush();
      Serial.println("[BMS] Initial status: Not charging (reading failed)");
      isCharging = false;
    }
    
    // Send initial SOC to Remote Teensy
    delay(100);
    bmsUpdateReading();
    Serial1.printf("BATTERY_SOC:%.1f\n", currentSoc);
    Serial1.flush();
    Serial.printf("[BMS] Initial SOC sent to Remote Teensy: %.1f%%\n", currentSoc);
  }
  
  // Test UART connection at startup
  delay(1000); // Give Teensy time to boot
  Serial.println("[TX] Testing UART connection to Teensy...");
  Serial1.println("STATUS");
  Serial1.flush();
  
  unsigned long test_timeout = millis() + 2000;
  bool teensy_responded = false;
  while (millis() < test_timeout) {
    if (Serial1.available()) {
      String response = Serial1.readStringUntil('\n');
      Serial.printf("[TX] Teensy startup response: %s\n", response.c_str());
      teensy_responded = true;
      break;
    }
    delay(10);
  }
  
  if (!teensy_responded) {
    Serial.println("[SPI_SLAVE] ⚠ No response from Teensy (use PING to test)");
  } else {
    Serial.println("[SPI_SLAVE] ✓ Teensy connected");
  }

  // Start tasks
  t0_ms = millis();
  xTaskCreatePinnedToCore(spi_rx_task, "spi_rx", 4096, NULL, 9, NULL, 1);
  xTaskCreatePinnedToCore(net_tx_task, "net_tx", 4096, NULL, 5, NULL, 0);
  xTaskCreatePinnedToCore(stats_task,  "stats",  4096, NULL, 3, NULL, 1);
  
  Serial.println("\n[SPI_SLAVE] ✓ ESP32 SPI Slave ready (type HELP for commands)");
}

void loop() {
  // Forward any Teensy responses / handle special commands
  while (Serial1.available()) {
    String line = Serial1.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      Serial.printf("[SPI_SLAVE] Teensy: %s\n", line.c_str());
      
      // Check if Remote Teensy is requesting BLE status (on boot)
      if (line == "REQUEST_BLE_STATUS") {
        Serial.println("[SPI_SLAVE] Remote Teensy requesting BLE status, forwarding via ESP-NOW...");
        // Send command to RX Radio which will forward to BLE Slave
        send_espnow_response("REQUEST_BLE_STATUS");
      }
    }
  }
  
  // Periodic BMS reading and sending
  uint32_t now = millis();
  if (bmsInitialized && (now - lastBmsSampleMs >= BMS_SAMPLE_MS)) {
    lastBmsSampleMs = now;
    bmsUpdateReading();
    sendBatteryPacket();
    
    // Also send SOC to Remote Teensy for battery indicator
    Serial1.printf("BATTERY_SOC:%.1f\n", currentSoc);
    Serial1.flush();
  }
  
  // Fast charging status check (every 100ms for instant detection)
  if (bmsInitialized && (now - lastChargingCheckMs >= 100)) {
    lastChargingCheckMs = now;
    checkChargingStatus();
  }
  
  handle_serial_commands();
  vTaskDelay(pdMS_TO_TICKS(100)); 
}
