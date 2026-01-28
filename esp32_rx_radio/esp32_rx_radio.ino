#include <Arduino.h>
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <nvs_flash.h>
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
static const uint32_t BMS_SAMPLE_MS = 2500;   // Read battery every 2.5 seconds
static const uint32_t BMS_UART_SEND_MS = 30000; // Send battery data to BLE slave every 30 seconds
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

// BMS state - Local
static float vBatFiltCal = NAN;
static float currentSoc = 0.0f;
static float iBatFilt = NAN;  // Filtered battery current (A) - slow filter for SOC
static float iBatFiltFast = NAN;  // Fast filtered current for charging detection
static bool isCharging = false;  // Charging status (positive current > threshold) - actual state sent to Teensy
static uint32_t lastBmsSampleMs = 0;
static uint32_t lastBmsUartSendMs = 0;
static uint32_t lastChargingCheckMs = 0;  // Faster check for charging status
static bool bmsInitialized = false;

// BMS state - Remote (received from SPI Slave via ESP-NOW)
static float remoteVoltage = NAN;
static float remoteSoc = 0.0f;
static uint32_t lastRemoteBmsMs = 0;

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

// ESP-NOW Battery Packet (received from SPI Slave)
struct __attribute__((packed)) ESPNowBattery {
    uint8_t magic[2];     // 0xBA, 0xBB
    float voltage;        // Calibrated VBAT in volts
    float soc;            // State of charge 0-100%
    uint16_t crc16;
};  // 12 bytes

// UART Battery Packet (sent to BLE Slave)
struct __attribute__((packed)) UartBatteryPacket {
    uint8_t sync[2];      // 0xBB, 0x55
    float local_voltage;
    float local_soc;
    float remote_voltage;
    float remote_soc;
    uint16_t crc16;
};  // 20 bytes

// ============================================================================
// PIN CONFIGURATION
// ============================================================================

#define PIN_MOSI 11
#define PIN_MISO 13
#define PIN_SCLK 12
#define PIN_CS   10

// UART pins for Teensy communication (Serial1)
#define TEENSY_UART_TX_PIN 2   // GPIO2 for UART TX to Teensy (→ Teensy Pin 15 RX3)
#define TEENSY_UART_RX_PIN 3   // GPIO3 for UART RX from Teensy (← Teensy Pin 14 TX3)

#define FRAME_BYTES 144
#define QUEUED_XFERS 8
#define SAMPLES_PER_FRAME 10

#define ESPNOW_CHANNEL 1
static const uint8_t ESPNOW_PEER_MAC[6] = {0x02, 0xAA, 0xBB, 0x00, 0x00, 0x01};  // SPI Slave MAC
#define USE_CUSTOM_MAC 1
static const uint8_t CUSTOM_STA_MAC_DUAL[6] = {0x02, 0xAA, 0xBB, 0x00, 0x00, 0x03};  // RX Radio MAC


// ============================================================================
// UART FORWARDING CONFIGURATION
// ============================================================================

#define UART_BAUD_RATE 921600   // Safe baud rate matching Serial console
#define UART_TX_PIN 17          // GPIO17 for UART TX
#define UART_RX_PIN 18          // GPIO18 for UART RX (not used but defined)

// UART packet structure (24 bytes total)
struct __attribute__((packed)) UartPacket {
    uint8_t sync[2];        // 0xAA, 0x55
    uint8_t type;           // 'L' = local, 'R' = remote, 'C' = combined
    uint16_t frame_idx;     // Frame index
    uint8_t sample_idx;     // Sample index within frame
    int32_t lc[4];          // Load cell values
    uint16_t crc16;         // CRC16 of packet (excluding sync and crc)
};

// UART statistics
static unsigned long uart_packets_sent = 0;
static unsigned long uart_bytes_sent = 0;
static unsigned long uart_send_errors = 0;

// UART streaming suspension for calibration commands
static bool suspend_uart_stream = false;
static unsigned long suspend_uart_since_ms = 0;

// ============================================================================
// LATEST DATA SYNCHRONIZATION
// ============================================================================

struct LatestData {
    int32_t lc[4];
    uint16_t frame_idx;
    uint8_t sample_idx;
    bool valid;
};

static LatestData latest_local = {0};
static LatestData latest_remote = {0};

// SPI buffers
static uint8_t spi_rxbuf[QUEUED_XFERS][FRAME_BYTES] __attribute__((aligned(4)));
static spi_slave_transaction_t spi_trx[QUEUED_XFERS];

// ============================================================================
// UART FORWARDING FUNCTIONS
// ============================================================================

static inline uint16_t calculate_uart_crc(const UartPacket* packet) {
    // CRC of everything except sync bytes and crc field
    const uint8_t* data = (const uint8_t*)packet + 2; // Skip sync bytes
    uint32_t length = sizeof(UartPacket) - 4; // Exclude sync and crc
    return crc16_ccitt_false(data, length);
}

static inline bool send_uart_packet(uint8_t type, uint16_t frame_idx, uint8_t sample_idx,
                                   int32_t lc1, int32_t lc2, int32_t lc3, int32_t lc4) {
    UartPacket packet;
    packet.sync[0] = 0xAA;
    packet.sync[1] = 0x55;
    packet.type = type;
    packet.frame_idx = frame_idx;
    packet.sample_idx = sample_idx;
    packet.lc[0] = lc1;
    packet.lc[1] = lc2;
    packet.lc[2] = lc3;
    packet.lc[3] = lc4;
    packet.crc16 = calculate_uart_crc(&packet);
    
    size_t bytes_written = Serial2.write((uint8_t*)&packet, sizeof(packet));
    
    if (bytes_written == sizeof(packet)) {
        uart_packets_sent++;
        uart_bytes_sent += bytes_written;
        return true;
    } else {
        uart_send_errors++;
        return false;
    }
}

static inline void forward_sample_to_uart(char type, uint16_t frame_idx, uint8_t sample_idx,
                                         int32_t lc1, int32_t lc2, int32_t lc3, int32_t lc4) {
    // Skip if UART streaming is suspended (e.g., during calibration commands)
    if (suspend_uart_stream) return;
    
    // Auto-resume after 2 seconds if something went wrong
    if (suspend_uart_since_ms > 0 && (millis() - suspend_uart_since_ms) > 2000) {
        suspend_uart_stream = false;
        suspend_uart_since_ms = 0;
    }
    
    // Send only individual sample - Redis-like system on slave handles combining
    send_uart_packet(type, frame_idx, sample_idx, lc1, lc2, lc3, lc4);
}

// ============================================================================
// ULTRA-FAST DATA PROCESSING
// ============================================================================

static inline void update_latest_local(uint16_t frame_idx, uint8_t sample_idx, 
                                      int32_t lc1, int32_t lc2, int32_t lc3, int32_t lc4) {
    latest_local.lc[0] = lc1;
    latest_local.lc[1] = lc2;
    latest_local.lc[2] = lc3;
    latest_local.lc[3] = lc4;
    latest_local.frame_idx = frame_idx;
    latest_local.sample_idx = sample_idx;
    latest_local.valid = true;
}

static inline void update_latest_remote(uint16_t frame_idx, uint8_t sample_idx,
                                       int32_t lc1, int32_t lc2, int32_t lc3, int32_t lc4) {
    latest_remote.lc[0] = lc1;
    latest_remote.lc[1] = lc2;
    latest_remote.lc[2] = lc3;
    latest_remote.lc[3] = lc4;
    latest_remote.frame_idx = frame_idx;
    latest_remote.sample_idx = sample_idx;
    latest_remote.valid = true;
}

// Statistics tracking
static unsigned long local_samples_count = 0;
static unsigned long remote_samples_count = 0;
static unsigned long local_frames_count = 0;
static unsigned long remote_frames_count = 0;

// ESP-NOW command statistics
static unsigned long espnow_commands_sent = 0;
static unsigned long espnow_command_errors = 0;

// Response capture for PING test
static volatile bool espnow_response_ready = false;
static String espnow_response_data = "";
static SemaphoreHandle_t response_mutex = NULL;

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
bool create_espnow_command(ESPNowCommand* cmd, const char* command_str);
bool validate_espnow_response(const ESPNowResponse* resp);

// ============================================================================
// ESP-NOW COMMAND FORWARDING
// ============================================================================

static bool send_espnow_command(const char* command_str) {
    ESPNowCommand cmd;
    if (!create_espnow_command(&cmd, command_str)) {
        Serial.printf("[RX_RADIO] ✗ Failed to create command packet for '%s'\n", command_str);
        espnow_command_errors++;
        return false;
    }
    
    esp_err_t result = esp_now_send(ESPNOW_PEER_MAC, (uint8_t*)&cmd, sizeof(cmd));
    if (result == ESP_OK) {
        espnow_commands_sent++;
        Serial.printf("[RX_RADIO] ✓ Command '%s' sent via ESP-NOW\n", command_str);
        return true;
    } else {
        espnow_command_errors++;
        Serial.printf("[RX_RADIO] ✗ Failed to send command '%s' via ESP-NOW (error: %d)\n", command_str, result);
        return false;
    }
}

// ============================================================================
// ESP-NOW COMMAND HELPER FUNCTION IMPLEMENTATIONS
// ============================================================================

bool create_espnow_command(ESPNowCommand* cmd, const char* command_str) {
    size_t cmd_len = strlen(command_str);
    if (cmd_len > 47) return false; // Command too long (max 47 chars + null terminator)
    
    memset(cmd, 0, sizeof(ESPNowCommand));
    cmd->magic[0] = 0xCD;
    cmd->magic[1] = 0xD1;
    
    // Copy command string (null-terminated)
    strncpy((char*)cmd->command, command_str, 47);
    cmd->command[47] = '\0'; // Ensure null termination
    
    cmd->timestamp = millis();
    
    // Calculate CRC16 of everything except CRC field
    cmd->crc16 = crc16_ccitt_false((const uint8_t*)cmd, sizeof(ESPNowCommand) - 2);
    return true;
}

bool validate_espnow_response(const ESPNowResponse* resp) {
    if (resp->magic[0] != 0xAB || resp->magic[1] != 0xCD) {
        return false;
    }
    
    uint16_t expected_crc = crc16_ccitt_false((const uint8_t*)resp, sizeof(ESPNowResponse) - 2);
    return resp->crc16 == expected_crc;
}

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
  
  // Send charging status to Local Teensy when it changes (with debouncing)
  if (stateNeedsChange && canChangeState) {
    isCharging = desiredChargingState;
    lastStateChangeMs = now;  // Reset change timer
    
    if (isCharging) {
      Serial1.println("BATTERY_CHARGING");
      Serial1.flush();
      Serial.printf("[BMS] ✓ CHARGING DETECTED (IBAT=%.3fA, raw_mA=%d) → Sending BATTERY_CHARGING to Local Teensy\n", iBatFiltFast, raw_mA);
    } else {
      Serial1.println("BATTERY_NOT_CHARGING");
      Serial1.flush();
      Serial.printf("[BMS] ✗ CHARGING STOPPED (IBAT=%.3fA, raw_mA=%d) → Sending BATTERY_NOT_CHARGING to Local Teensy\n", iBatFiltFast, raw_mA);
    }
  }
}

static bool validateEspNowBattery(const ESPNowBattery* pkt) {
  if (pkt->magic[0] != 0xBA || pkt->magic[1] != 0xBB) {
    return false;
  }
  uint16_t expected_crc = crc16_ccitt_false((const uint8_t*)pkt, sizeof(ESPNowBattery) - 2);
  return pkt->crc16 == expected_crc;
}

static uint16_t calculateBatteryUartCrc(const UartBatteryPacket* pkt) {
  // CRC of everything except sync bytes and crc field
  const uint8_t* data = (const uint8_t*)pkt + 2;
  uint32_t length = sizeof(UartBatteryPacket) - 4;
  return crc16_ccitt_false(data, length);
}

static bool sendBatteryToBleSlave() {
  UartBatteryPacket pkt;
  pkt.sync[0] = 0xBB;
  pkt.sync[1] = 0x55;
  pkt.local_voltage = isnan(vBatFiltCal) ? 0.0f : vBatFiltCal;
  pkt.local_soc = currentSoc;
  pkt.remote_voltage = isnan(remoteVoltage) ? 0.0f : remoteVoltage;
  pkt.remote_soc = remoteSoc;
  pkt.crc16 = calculateBatteryUartCrc(&pkt);
  
  size_t bytes_written = Serial2.write((uint8_t*)&pkt, sizeof(pkt));
  return (bytes_written == sizeof(pkt));
}

static inline void count_sample(char type, uint16_t frame_idx, uint8_t sample_idx,
                               int32_t lc1, int32_t lc2, int32_t lc3, int32_t lc4) {
    if (type == 'L') {
        local_samples_count++;
        if (sample_idx == 0) local_frames_count++;
        update_latest_local(frame_idx, sample_idx, lc1, lc2, lc3, lc4);
    } else if (type == 'R') {
        remote_samples_count++;
        if (sample_idx == 0) remote_frames_count++;
        update_latest_remote(frame_idx, sample_idx, lc1, lc2, lc3, lc4);
    }
    
    // Forward sample to UART slave ESP32
    forward_sample_to_uart(type, frame_idx, sample_idx, lc1, lc2, lc3, lc4);
}

static inline void process_frame_ultra_fast(const InnerFrame* frame, char type) {
    const uint8_t* samples = frame->samples;
    uint16_t frame_idx = frame->frame_idx;
    
    for (int sample = 0; sample < SAMPLES_PER_FRAME; sample++) {
        int32_t lc1, lc2, lc3, lc4;
        
        // Use the tested unpacking function from common_frame.h
        extract_load_cell_sample(samples, sample, &lc1, &lc2, &lc3, &lc4);
        
        count_sample(type, frame_idx, sample, lc1, lc2, lc3, lc4);
    }
}

// ============================================================================
// SPI PROCESSING
// ============================================================================

static void spi_requeue_all() {
    for (int i = 0; i < QUEUED_XFERS; i++) {
        memset(&spi_trx[i], 0, sizeof(spi_trx[i]));
        spi_trx[i].length = FRAME_BYTES * 8;
        spi_trx[i].rx_buffer = spi_rxbuf[i];
        spi_trx[i].user = (void*)(intptr_t)i;
        ESP_ERROR_CHECK(spi_slave_queue_trans(SPI2_HOST, &spi_trx[i], portMAX_DELAY));
    }
}

static void spi_rx_task(void* param) {
    while (true) {
        spi_slave_transaction_t* ret;
        esp_err_t e = spi_slave_get_trans_result(SPI2_HOST, &ret, portMAX_DELAY);
        if (e != ESP_OK) continue;
        
        int idx = (int)(intptr_t)ret->user;
        InnerFrame* f = (InnerFrame*)spi_rxbuf[idx];

        if (f->sync[0] == 0xA5 && f->sync[1] == 0x5A) {
            uint16_t want = f->crc16;
            uint16_t got = crc16_ccitt_false((uint8_t*)f, offsetof(InnerFrame, crc16));
            
            if (want == got) {
                process_frame_ultra_fast(f, 'L');
            }
        }

        ESP_ERROR_CHECK(spi_slave_queue_trans(SPI2_HOST, ret, portMAX_DELAY));
    }
}

// ============================================================================
// ESP-NOW PROCESSING
// ============================================================================

static void espnow_rx_callback(const esp_now_recv_info *recv_info, const uint8_t *data, int len) {
    // Check if this is a battery packet from SPI Slave
    if (len == sizeof(ESPNowBattery)) {
        const ESPNowBattery* batt = (const ESPNowBattery*)data;
        
        if (validateEspNowBattery(batt)) {
            remoteVoltage = batt->voltage;
            remoteSoc = batt->soc;
            lastRemoteBmsMs = millis();
            return;
        }
    }
    
    // Check if this is a response packet
    if (len == sizeof(ESPNowResponse)) {
        const ESPNowResponse* resp = (const ESPNowResponse*)data;
        
        if (validate_espnow_response(resp)) {
            String response_str = String(resp->response);
            
            // Check if remote Teensy is requesting BLE status (on boot)
            if (response_str == "REQUEST_BLE_STATUS") {
                Serial.println("[RX_RADIO] Remote Teensy requesting BLE status, forwarding to BLE Slave...");
                Serial2.println("@@BLE_STATUS");  // Use @@ prefix for commands to BLE Slave
                Serial2.flush();
                // Response (BLE_CONNECTED/BLE_DISCONNECTED) will be handled by handle_serial_commands
                // which forwards to BOTH local and remote Teensy
                return;
            }
            
            // Store response for command handlers (PING, CAL, etc.)
            // The command handler will forward to BLE Slave
            if (response_mutex != NULL && xSemaphoreTake(response_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                espnow_response_data = response_str;
                espnow_response_ready = true;
                xSemaphoreGive(response_mutex);
            }
            
            // Don't forward here - let the command handler do it to avoid duplicates
        }
        return;
    }
    
    // Process data frames
    if (len != INNER_FRAME_SIZE) return;
    
    const InnerFrame* frame = (const InnerFrame*)data;
    if (!validate_inner_frame(frame)) return;
    
    process_frame_ultra_fast(frame, 'R');
}

// ============================================================================
// INITIALIZATION
// ============================================================================

static void espnow_init() {
#if USE_CUSTOM_MAC
    WiFi.mode(WIFI_STA);
    esp_wifi_set_mac(WIFI_IF_STA, (uint8_t*)CUSTOM_STA_MAC_DUAL);
    Serial.printf("Custom MAC set: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  CUSTOM_STA_MAC_DUAL[0], CUSTOM_STA_MAC_DUAL[1], CUSTOM_STA_MAC_DUAL[2],
                  CUSTOM_STA_MAC_DUAL[3], CUSTOM_STA_MAC_DUAL[4], CUSTOM_STA_MAC_DUAL[5]);
#endif
    
    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    Serial.printf("WiFi channel set to: %d\n", ESPNOW_CHANNEL);
    
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed!");
        return;
    }
    Serial.println("ESP-NOW initialized successfully");
    
    esp_now_register_recv_cb(espnow_rx_callback);
    
    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, ESPNOW_PEER_MAC, 6);
    peer_info.channel = ESPNOW_CHANNEL;
    peer_info.encrypt = false;
    
    if (esp_now_add_peer(&peer_info) == ESP_OK) {
        Serial.printf("ESP-NOW peer added: %02X:%02X:%02X:%02X:%02X:%02X\n",
                      ESPNOW_PEER_MAC[0], ESPNOW_PEER_MAC[1], ESPNOW_PEER_MAC[2],
                      ESPNOW_PEER_MAC[3], ESPNOW_PEER_MAC[4], ESPNOW_PEER_MAC[5]);
    } else {
        Serial.println("Failed to add ESP-NOW peer!");
    }
}

static void handle_serial_commands() {
    // Handle commands from USB Serial Monitor
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        command.toUpperCase();
        
        Serial.printf("[RX_RADIO] USB Command received: %s\n", command.c_str());
        process_command(command);
    }
    
    // Handle commands from ESP32 BLE Slave via UART2
    if (Serial2.available()) {
        String command = Serial2.readStringUntil('\n');
        command.trim();
        command.toUpperCase();
        
        if (command.length() > 0) {
            Serial.printf("[RX_RADIO] UART Command received from BLE Slave: %s\n", command.c_str());
            
            // Forward BLE connection status to BOTH Local and Remote Teensy
            if (command == "BLE_CONNECTED" || command == "BLE_DISCONNECTED") {
                Serial.printf("[RX_RADIO] Forwarding BLE status '%s' to BOTH Local and Remote Teensy...\n", command.c_str());
                
                // Send to Local Teensy via Serial1
                Serial1.println(command);
                Serial1.flush();
                
                // Send to Remote Teensy via ESP-NOW
                send_espnow_command(command.c_str());
            } else {
                // Process other commands normally
                process_command(command);
            }
        }
    }
}

static void process_command(String command) {
    // Local Teensy control commands (with optional LOCAL_ prefix)
    // Handle LOCAL_* commands by stripping prefix
    String local_cmd = command;
    if (command.startsWith("LOCAL_") && !command.startsWith("LOCAL_PING") && !command.startsWith("LOCAL_CAL_") && !command.startsWith("LOCAL_STATUS")) {
        local_cmd = command.substring(6); // Remove "LOCAL_" prefix
    }
    
    if (local_cmd == "START" || local_cmd == "STOP" || local_cmd == "RESTART" || local_cmd == "RESET" ||
        local_cmd == "LED_ON" || local_cmd == "LED_OFF" ||
        local_cmd == "MOCK_ON" || local_cmd == "MOCK_OFF") {
        command = local_cmd; // Use stripped command
            Serial.printf("[RX_RADIO] Forwarding '%s' to Local Teensy...\n", command.c_str());
            
            Serial1.println(command);
            Serial1.flush();
            
            // Wait for response from Teensy
            unsigned long timeout = millis() + 2000;
            bool response_received = false;
            while (millis() < timeout) {
                if (Serial1.available()) {
                    String response = Serial1.readStringUntil('\n');
                    Serial.printf("[RX_RADIO] Local Teensy Response: %s\n", response.c_str());
                    response_received = true;
                    break;
                }
                delay(10);
            }
            if (!response_received) {
                Serial.println("[RX_RADIO] ⚠ Local Teensy response timeout");
            }
    }
    // Local PING test
    else if (command == "LOCAL_PING") {
        Serial.println("\n[RX_RADIO] LOCAL PING-PONG TEST");
        
        // Clear any stale data in Serial1 RX buffer
        while (Serial1.available()) Serial1.read();
        
        unsigned long start_time = millis();
        Serial1.print("PING\n");
        Serial1.flush();
        
        // Wait for response from Local Teensy
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
            response_text.trim();
            Serial.printf("[RX_RADIO] ✓ PONG received: '%s' (%lu ms)\n", response_text.c_str(), round_trip);
            Serial2.printf("##LOCAL:%s\n", response_text.c_str());
            Serial2.flush();
        } else {
            Serial.printf("[RX_RADIO] ✗ TIMEOUT after %lu ms\n", round_trip);
            Serial2.println("##LOCAL:TIMEOUT");
            Serial2.flush();
        }
    }
    // Remote Teensy control commands (via ESP-NOW)
    else if (command == "REMOTE_START" || command == "REMOTE_STOP" || command == "REMOTE_RESTART" || command == "REMOTE_RESET" ||
             command == "REMOTE_LED_ON" || command == "REMOTE_LED_OFF" ||
             command == "REMOTE_MOCK_ON" || command == "REMOTE_MOCK_OFF") {
        String teensy_command = command.substring(7); // Remove "REMOTE_" prefix
        Serial.printf("[RX_RADIO] Sending '%s' to Remote Teensy via ESP-NOW...\n", teensy_command.c_str());
        if (send_espnow_command(teensy_command.c_str())) {
            Serial.printf("[RX_RADIO] ✓ Remote command '%s' sent successfully\n", teensy_command.c_str());
        } else {
            Serial.printf("[RX_RADIO] ✗ Failed to send remote command '%s'\n", teensy_command.c_str());
        }
    }
    // LOCAL_STATUS - Get status from local Teensy (for BLE STATUS command)
    else if (command == "LOCAL_STATUS") {
        Serial.println("[RX_RADIO] LOCAL_STATUS requested");
        
        // Clear any stale data in Serial1 RX buffer
        while (Serial1.available()) Serial1.read();
        
        Serial1.println("STATUS");
        Serial1.flush();
        
        // Wait for response from Local Teensy
        unsigned long timeout = millis() + 3000;
        bool got_response = false;
        String response_text = "";
        
        while (millis() < timeout) {
            if (Serial1.available()) {
                response_text = Serial1.readStringUntil('\n');
                response_text.trim();
                if (response_text.length() > 0) {
                    got_response = true;
                    break;
                }
            }
            delay(5);
        }
        
        if (got_response) {
            Serial.printf("[RX_RADIO] Local STATUS: %s\n", response_text.c_str());
            Serial2.printf("##LOCAL:%s\n", response_text.c_str());
            Serial2.flush();
        } else {
            Serial.println("[RX_RADIO] ✗ Local STATUS timeout");
            Serial2.println("##LOCAL:STATUS:TIMEOUT");
            Serial2.flush();
        }
    }
    // REMOTE_STATUS - Get status from remote Teensy via ESP-NOW (for BLE STATUS command)
    else if (command == "REMOTE_STATUS") {
        Serial.println("[RX_RADIO] REMOTE_STATUS requested");
        
        // Clear previous response
        if (response_mutex != NULL && xSemaphoreTake(response_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            espnow_response_ready = false;
            espnow_response_data = "";
            xSemaphoreGive(response_mutex);
        }
        
        if (send_espnow_command("STATUS")) {
            // Wait for response from SPI Slave
            unsigned long timeout = millis() + 3000;
            String response = "";
            
            while (millis() < timeout) {
                if (response_mutex != NULL && xSemaphoreTake(response_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    if (espnow_response_ready) {
                        response = espnow_response_data;
                        espnow_response_ready = false;
                        xSemaphoreGive(response_mutex);
                        break;
                    }
                    xSemaphoreGive(response_mutex);
                }
                delay(10);
            }
            
            if (response.length() > 0) {
                Serial.printf("[RX_RADIO] Remote STATUS: %s\n", response.c_str());
                Serial2.printf("##REMOTE:%s\n", response.c_str());
            } else {
                Serial.println("[RX_RADIO] ✗ Remote STATUS timeout");
                Serial2.println("##REMOTE:STATUS:TIMEOUT");
            }
            Serial2.flush();
        } else {
            Serial.println("[RX_RADIO] ✗ Failed to send STATUS via ESP-NOW");
            Serial2.println("##REMOTE:STATUS:ESPNOW_ERR");
            Serial2.flush();
        }
    }
    // Remote PING test
    else if (command == "REMOTE_PING") {
        Serial.println("\n[RX_RADIO] REMOTE PING-PONG TEST (via ESP-NOW)");
        
        // Clear previous response
        if (response_mutex != NULL && xSemaphoreTake(response_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            espnow_response_ready = false;
            espnow_response_data = "";
            xSemaphoreGive(response_mutex);
        }
        
        unsigned long start_time = millis();
        if (send_espnow_command("PING")) {
            // Wait for response from SPI Slave
            unsigned long timeout = millis() + 5000;
            bool got_response = false;
            String response_text = "";
            
            while (millis() < timeout) {
                if (response_mutex != NULL && xSemaphoreTake(response_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    if (espnow_response_ready) {
                        response_text = espnow_response_data;
                        espnow_response_ready = false;
                        got_response = true;
                        xSemaphoreGive(response_mutex);
                        break;
                    }
                    xSemaphoreGive(response_mutex);
                }
                delay(10);
            }
            
            unsigned long round_trip = millis() - start_time;
            if (got_response) {
                Serial.printf("[RX_RADIO] ✓ PONG received: '%s' (%lu ms)\n", response_text.c_str(), round_trip);
            } else {
                Serial.printf("[RX_RADIO] ✗ TIMEOUT after %lu ms\n", round_trip);
            }
        } else {
            Serial.println("[RX_RADIO] ✗ Failed to send ESP-NOW PING");
        }
    }
    // ALL commands - control both local and remote Teensy simultaneously
    else if (command == "ALL_START" || command == "ALL_STOP" || command == "ALL_RESTART" || command == "ALL_RESET" ||
             command == "ALL_LED_ON" || command == "ALL_LED_OFF" ||
             command == "ALL_MOCK_ON" || command == "ALL_MOCK_OFF") {
        String teensy_command = command.substring(4); // Remove "ALL_" prefix
        Serial.printf("[RX_RADIO] Executing '%s' on BOTH Local and Remote Teensy...\n", teensy_command.c_str());
        
        bool local_success = false;
        bool remote_success = false;
        
        // Send to Local Teensy first
        Serial.printf("[RX_RADIO] 1/2 Sending '%s' to Local Teensy...\n", teensy_command.c_str());
        Serial1.println(teensy_command);
        Serial1.flush();
        
        // Wait for local response
        unsigned long timeout = millis() + 2000;
        while (millis() < timeout) {
            if (Serial1.available()) {
                String response = Serial1.readStringUntil('\n');
                Serial.printf("[RX_RADIO] Local Teensy Response: %s\n", response.c_str());
                local_success = response.indexOf("OK") >= 0;
                break;
            }
            delay(10);
        }
        
        // Send to Remote Teensy via ESP-NOW
        Serial.printf("[RX_RADIO] 2/2 Sending '%s' to Remote Teensy via ESP-NOW...\n", teensy_command.c_str());
        remote_success = send_espnow_command(teensy_command.c_str());
        
        // Summary
        Serial.printf("[RX_RADIO] === ALL_%s SUMMARY ===\n", teensy_command.c_str());
        Serial.printf("  Local Teensy:  %s\n", local_success ? "✓ SUCCESS" : "✗ FAILED");
        Serial.printf("  Remote Teensy: %s\n", remote_success ? "✓ SUCCESS" : "✗ FAILED");
        Serial.println("=============================");
    }
    // LOCAL calibration commands - forward to Local Teensy via UART
    else if (command.startsWith("LOCAL_CAL_")) {
        String cal_command = command.substring(6);  // Remove "LOCAL_" prefix
        Serial.printf("[RX_RADIO] LOCAL_CAL: %s\n", cal_command.c_str());
        
        // Suspend binary data streaming
        suspend_uart_stream = true;
        suspend_uart_since_ms = millis();
        Serial2.flush();
        
        // Clear buffer
        while (Serial1.available()) Serial1.read();
        
        // Send command
        cal_command.toUpperCase();
        Serial1.println(cal_command);
        Serial1.flush();
        
        // Wait for single-line response
        unsigned long timeout = millis() + 5000;
        String response = "";
        
        while (millis() < timeout) {
            if (Serial1.available()) {
                response = Serial1.readStringUntil('\n');
                response.trim();
                if (response.length() > 0) {
                    Serial.printf("[RX_RADIO] << %s\n", response.c_str());
                    break;
                }
            }
            delay(5);
        }
        
        // Resume streaming
        suspend_uart_stream = false;
        
        // Forward to BLE Slave
        if (response.length() > 0) {
            Serial2.printf("##LOCAL:%s\n", response.c_str());
        } else {
            Serial2.println("##LOCAL:TIMEOUT");
        }
        Serial2.flush();
    }
    // REMOTE calibration commands - forward to Remote Teensy via ESP-NOW
    else if (command.startsWith("REMOTE_CAL_")) {
        String cal_command = command.substring(7);  // Remove "REMOTE_" prefix
        Serial.printf("[RX_RADIO] REMOTE_CAL: %s\n", cal_command.c_str());
        
        // Suspend binary data streaming
        suspend_uart_stream = true;
        suspend_uart_since_ms = millis();
        Serial2.flush();
        
        // Clear previous response
        if (response_mutex != NULL && xSemaphoreTake(response_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            espnow_response_ready = false;
            espnow_response_data = "";
            xSemaphoreGive(response_mutex);
        }
        
        // Send command via ESP-NOW
        cal_command.toUpperCase();
        
        if (send_espnow_command(cal_command.c_str())) {
            // Wait for response
            unsigned long timeout = millis() + 5000;
            String response = "";
            
            while (millis() < timeout) {
                if (response_mutex != NULL && xSemaphoreTake(response_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    if (espnow_response_ready) {
                        response = espnow_response_data;
                        espnow_response_ready = false;
                        xSemaphoreGive(response_mutex);
                        break;
                    }
                    xSemaphoreGive(response_mutex);
                }
                delay(10);
            }
            
            // Resume streaming
            suspend_uart_stream = false;
            
            // Forward to BLE Slave
            if (response.length() > 0) {
                Serial.printf("[RX_RADIO] << %s\n", response.c_str());
                Serial2.printf("##REMOTE:%s\n", response.c_str());
            } else {
                Serial2.println("##REMOTE:TIMEOUT");
            }
            Serial2.flush();
        } else {
            suspend_uart_stream = false;
            Serial2.println("##REMOTE:ESPNOW_ERR");
            Serial2.flush();
        }
    }
    // STATUS command
    else if (command == "STATUS") {
        Serial.println("\n[RX_RADIO] === STATUS ===");
        Serial.printf("  Samples:   Local=%lu  Remote=%lu\n", local_samples_count, remote_samples_count);
        Serial.printf("  Frames:    Local=%lu  Remote=%lu\n", local_frames_count, remote_frames_count);
        Serial.printf("  UART TX:   %lu packets (%lu bytes), %lu errors\n", uart_packets_sent, uart_bytes_sent, uart_send_errors);
        Serial.printf("  ESP-NOW:   %lu commands sent, %lu errors\n", espnow_commands_sent, espnow_command_errors);
        Serial.printf("  BMS Local: %s | VBAT=%.2fV SOC=%.1f%%\n", bmsInitialized ? "OK" : "N/A", 
                      isnan(vBatFiltCal) ? 0.0f : vBatFiltCal, currentSoc);
        Serial.printf("  BMS Remote: VBAT=%.2fV SOC=%.1f%% (age=%lums)\n", 
                      isnan(remoteVoltage) ? 0.0f : remoteVoltage, remoteSoc, 
                      lastRemoteBmsMs > 0 ? (millis() - lastRemoteBmsMs) : 0);
        Serial.printf("  Heap:      %lu bytes free\n", ESP.getFreeHeap());
        Serial.println("=========================\n");
    }
    // HELP command
    else if (command == "HELP") {
        Serial.println("\n[RX_RADIO] === AVAILABLE COMMANDS ===");
        Serial.println("  Local:  START, STOP, RESTART, RESET, LOCAL_PING");
        Serial.println("  Remote: REMOTE_START/STOP/RESTART/RESET, REMOTE_PING");
        Serial.println("  Both:   ALL_START/STOP/RESTART/RESET");
        Serial.println("  Calibration:");
        Serial.println("    LOCAL_CAL_SHOW, LOCAL_CAL_TARE, LOCAL_CAL_ADD_<kg>, etc.");
        Serial.println("    REMOTE_CAL_SHOW, REMOTE_CAL_TARE, REMOTE_CAL_ADD_<kg>, etc.");
        Serial.println("  System: STATUS, HELP");
        Serial.println("=====================================\n");
    } else if (command == "") {
        // Empty command, do nothing
    } else {
        Serial.printf("[RX_RADIO] ✗ Unknown command: '%s'\n", command.c_str());
        Serial.println("[RX_RADIO] Type 'HELP' for available commands");
    }
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    Serial.begin(921600);   // Safer baud rate to prevent USB driver issues
    delay(50);
    
    // Initialize I2C for BMS
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(I2C_HZ);
    Wire.setTimeOut(50);
    Serial.printf("[BMS] I2C initialized (SDA=GPIO%d, SCL=GPIO%d, %lu Hz)\n", I2C_SDA, I2C_SCL, I2C_HZ);
    
  // Initialize BMS
  bmsInitialized = bmsInit();
  
  // Send initial charging status and SOC to Teensy after Serial1 is ready
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
    
    // Send initial SOC to Local Teensy
    delay(100);
    bmsUpdateReading();
    Serial1.printf("BATTERY_SOC:%.1f\n", currentSoc);
    Serial1.flush();
    Serial.printf("[BMS] Initial SOC sent to Local Teensy: %.1f%%\n", currentSoc);
  }
    
    // Create response mutex for PING test synchronization
    response_mutex = xSemaphoreCreateMutex();
    if (response_mutex == NULL) {
        Serial.println("[RX_RADIO] ⚠ Failed to create response mutex!");
    } else {
        Serial.println("[RX_RADIO] ✓ Response mutex created");
    }
    
    // Initialize UART2 for forwarding data to slave ESP32
    Serial2.begin(UART_BAUD_RATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    Serial2.setTxBufferSize(8192);  // Increase TX buffer for high throughput
    Serial.printf("UART2 initialized at %d bps on TX pin %d\n", UART_BAUD_RATE, UART_TX_PIN);
    
    // Initialize Serial1 for Teensy communication
    Serial1.setRxBufferSize(512);
    Serial1.begin(9600, SERIAL_8N1, TEENSY_UART_RX_PIN, TEENSY_UART_TX_PIN);
    Serial.printf("Serial1: 9600 bps (TX=GPIO%d → Teensy, RX=GPIO%d ← Teensy)\n", 
                  TEENSY_UART_TX_PIN, TEENSY_UART_RX_PIN);
    
    // Test UART connection at startup
    delay(1000); // Give Teensy time to boot
    Serial.println("[RX_RADIO] Testing UART connection to Local Teensy...");
    Serial1.println("STATUS");
    Serial1.flush();
    
    unsigned long test_timeout = millis() + 2000;
    bool teensy_responded = false;
    while (millis() < test_timeout) {
        if (Serial1.available()) {
            String response = Serial1.readStringUntil('\n');
            Serial.printf("[RX_RADIO] Local Teensy startup response: %s\n", response.c_str());
            teensy_responded = true;
            break;
        }
        delay(10);
    }
    
    if (!teensy_responded) {
        Serial.println("[RX_RADIO] ⚠ No response from Local Teensy (use LOCAL_PING to test)");
    } else {
        Serial.println("[RX_RADIO] ✓ Local Teensy connected");
    }
    
    // Send a startup test packet
    delay(1000);  // Wait for slave to initialize
    UartPacket startup_packet;
    startup_packet.sync[0] = 0xAA;
    startup_packet.sync[1] = 0x55;
    startup_packet.type = 'S';  // Startup packet
    startup_packet.frame_idx = 0;
    startup_packet.sample_idx = 0;
    startup_packet.lc[0] = 11111;
    startup_packet.lc[1] = 22222;
    startup_packet.lc[2] = 33333;
    startup_packet.lc[3] = 44444;
    startup_packet.crc16 = calculate_uart_crc(&startup_packet);
    
    size_t bytes_sent = Serial2.write((uint8_t*)&startup_packet, sizeof(startup_packet));
    Serial.printf("Startup UART test: Sent %d bytes\n", bytes_sent);
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize ESP-NOW
    espnow_init();
    
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
    gpio_set_pull_mode((gpio_num_t)PIN_CS, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)PIN_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)PIN_SCLK, GPIO_PULLUP_ONLY);

    spi_requeue_all();
    
    // Start high priority SPI task on Core 1 (max priority is 24)
    xTaskCreatePinnedToCore(spi_rx_task, "spi_rx", 4096, NULL, 24, NULL, 1);
    
    Serial.println("\n[RX_RADIO] ✓ ESP32 RX Radio ready (type HELP for commands)");
}

void loop() {
    static unsigned long last_stats_time = 0;
    static unsigned long last_local_count = 0;
    static unsigned long last_remote_count = 0;
    static unsigned long last_uart_packets = 0;
    static unsigned long last_uart_bytes = 0;
    
    unsigned long now = millis();
    
    // Periodic BMS reading (local)
    if (bmsInitialized && (now - lastBmsSampleMs >= BMS_SAMPLE_MS)) {
        lastBmsSampleMs = now;
        bmsUpdateReading();
    }
    
    // Fast charging status check (every 100ms for instant detection)
    if (bmsInitialized && (now - lastChargingCheckMs >= 100)) {
        lastChargingCheckMs = now;
        checkChargingStatus();
    }
    
    // Periodic battery data send to BLE Slave
    if (now - lastBmsUartSendMs >= BMS_UART_SEND_MS) {
        lastBmsUartSendMs = now;
        sendBatteryToBleSlave();
        
        // Also send SOC to Local Teensy for battery indicator
        Serial1.printf("BATTERY_SOC:%.1f\n", currentSoc);
        Serial1.flush();
    }
    
    // Print live stats every 5 seconds
    if (now - last_stats_time >= 5000) {
        float local_sps = (local_samples_count - last_local_count) / 5.0f;
        float remote_sps = (remote_samples_count - last_remote_count) / 5.0f;
        float uart_pps = (uart_packets_sent - last_uart_packets) / 5.0f;
        float uart_kbps = ((uart_bytes_sent - last_uart_bytes) * 8.0f) / 5000.0f;
        
        Serial.printf("📊 LIVE | Local: %.0f sps | Remote: %.0f sps | UART: %.0f pps (%.1f kbps)\n",
                     local_sps, remote_sps, uart_pps, uart_kbps);
        
        // Battery status log
        Serial.printf("🔋 BAT | Local: %.2fV (%.1f%%) | Remote: %.2fV (%.1f%%) age=%lums\n",
                     isnan(vBatFiltCal) ? 0.0f : vBatFiltCal, currentSoc,
                     isnan(remoteVoltage) ? 0.0f : remoteVoltage, remoteSoc,
                     lastRemoteBmsMs > 0 ? (millis() - lastRemoteBmsMs) : 0);
        
        last_stats_time = now;
        last_local_count = local_samples_count;
        last_remote_count = remote_samples_count;
        last_uart_packets = uart_packets_sent;
        last_uart_bytes = uart_bytes_sent;
    }
    
    // Forward any responses from Local Teensy to BLE Slave
    while (Serial1.available()) {
        String line = Serial1.readStringUntil('\n');
        line.trim();
        
        if (line.length() > 0) {
            Serial.printf("[RX_RADIO] Local Teensy: %s\n", line.c_str());
            
            // Check if Teensy is requesting BLE status (on boot)
            if (line == "REQUEST_BLE_STATUS") {
                Serial.println("[RX_RADIO] Teensy requesting BLE status, forwarding to BLE Slave...");
                Serial2.println("@@BLE_STATUS");  // Use @@ prefix for commands to BLE Slave
                Serial2.flush();
                // Response will come back via ##LOCAL: prefix and be forwarded to Teensy
            } else {
                // Use "##" prefix to distinguish text from binary UART packets
                Serial2.printf("##LOCAL:%s\n", line.c_str());
                Serial2.flush();
            }
        }
    }
    
    handle_serial_commands();
    vTaskDelay(pdMS_TO_TICKS(100));
}
