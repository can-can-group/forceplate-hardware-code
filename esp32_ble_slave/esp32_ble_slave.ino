#include <Arduino.h>
#include <WiFi.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "common_frame.h"

// ============================================================================
// DEVICE INFORMATION (for BLE Device Information Service)
// ============================================================================
#define DEVICE_MANUFACTURER_NAME "well body tech"
#define DEVICE_MODEL_NUMBER      "mesh force plate"
#define FIRMWARE_VERSION         "1.0.0"        // Format: MAJOR.MINOR.PATCH
#define HARDWARE_VERSION         "PCB-1.0"      // Format: PCB-MAJOR.MINOR
#define SOFTWARE_VERSION         "lib-1.0.0"    // Format: lib-MAJOR.MINOR.PATCH

// ============================================================================
// UART CONFIGURATION (must match master ESP32)
// ============================================================================

#define UART_BAUD_RATE 921600   // Safe baud rate matching Serial console
#define UART_RX_PIN 18          // GPIO18 for UART RX from ESP32 RX Radio
#define UART_TX_PIN 17          // GPIO17 for UART TX to ESP32 RX Radio

// UART packet structure (24 bytes total) - must match master
struct __attribute__((packed)) UartPacket {
    uint8_t sync[2];        // 0xAA, 0x55
    uint8_t type;           // 'L' = local, 'R' = remote, 'C' = combined
    uint16_t frame_idx;     // Frame index
    uint8_t sample_idx;     // Sample index within frame
    int32_t lc[4];          // Load cell values
    uint16_t crc16;         // CRC16 of packet (excluding sync and crc)
};

// ============================================================================
// UART BATTERY PACKET (received from RX Radio)
// ============================================================================

struct __attribute__((packed)) UartBatteryPacket {
    uint8_t sync[2];      // 0xBB, 0x55
    float local_voltage;
    float local_soc;
    float remote_voltage;
    float remote_soc;
    uint16_t crc16;
};  // 20 bytes

// Battery data storage
static float battery_local_voltage = 0.0f;
static float battery_local_soc = 0.0f;
static float battery_remote_voltage = 0.0f;
static float battery_remote_soc = 0.0f;
static uint32_t last_battery_received_ms = 0;
static uint32_t last_battery_ble_send_ms = 0;
static unsigned long battery_packets_received = 0;

// Status data storage (for STATUS command)
static String status_local_state = "UNKNOWN";
static String status_local_led = "UNKNOWN";
static String status_remote_state = "UNKNOWN";
static String status_remote_led = "UNKNOWN";

// ============================================================================
// BLE CONFIGURATION
// ============================================================================

// Custom service for load cell data
#define BLE_SERVICE_UUID        "12345678-1234-1234-1234-123456789abc"

// Standard BLE Battery Service (so diagnostic tools can read battery level automatically)
#define BLE_BATTERY_SERVICE_UUID        BLEUUID((uint16_t)0x180F)  // Standard Battery Service
#define BLE_BATTERY_LEVEL_CHAR_UUID     BLEUUID((uint16_t)0x2A19)  // Standard Battery Level (0-100%)
#define BLE_DATA_CHARACTERISTIC_UUID "87654321-4321-4321-4321-cba987654321"
#define BLE_CMD_CHARACTERISTIC_UUID "11111111-2222-3333-4444-555555555555"  // Bidirectional: WRITE + NOTIFY

BLEServer* pServer = nullptr;
BLECharacteristic* pDataCharacteristic = nullptr;    // Sensor data (binary NOTIFY)
BLECharacteristic* pCmdCharacteristic = nullptr;     // Commands (WRITE) + Responses (NOTIFY)
BLECharacteristic* pStdBatteryLevelChar = nullptr;        // Standard Battery Level (0-100%) - min of both

// Device Information Service characteristics
BLECharacteristic* pManufacturerChar = nullptr;
BLECharacteristic* pModelNumberChar = nullptr;
BLECharacteristic* pSerialNumberChar = nullptr;
BLECharacteristic* pFirmwareRevisionChar = nullptr;
BLECharacteristic* pHardwareRevisionChar = nullptr;
BLECharacteristic* pSoftwareRevisionChar = nullptr;
BLECharacteristic* pAppearanceChar = nullptr;

bool deviceConnected = false;
bool oldDeviceConnected = false;

// ============================================================================
// DATA STORE
// ============================================================================

#define QUEUE_SIZE 20  // Buffer for incoming samples

struct SampleData {
    int32_t lc[4];
    uint32_t timestamp;
    uint16_t frame_idx;
    uint8_t sample_idx;
    bool consumed;
};

struct DataStore {
    // Local data queue
    SampleData local_queue[QUEUE_SIZE];
    int local_head;
    int local_tail;
    int local_count;
    
    // Remote data queue
    SampleData remote_queue[QUEUE_SIZE];
    int remote_head;
    int remote_tail;
    int remote_count;
    
    // Latest consumed samples (for fallback)
    int32_t last_local_lc[4];
    int32_t last_remote_lc[4];
    bool has_local_data;
    bool has_remote_data;
};

static DataStore data_store = {0};

// Hardware timer for 1000Hz sampling
hw_timer_t* sampling_timer = nullptr;
static volatile bool timer_sample_ready = false;
static volatile unsigned long timer_interrupts_count = 0;

// Packet rate control
// Removed: packet_rate_counter - now only send full 10-sample packets

// Data flow tracking
static unsigned long last_data_received_ms = 0;
#define DATA_TIMEOUT_MS 500  // Consider data stale after 500ms (no BLE send)

// ============================================================================
// STATISTICS
// ============================================================================

static unsigned long uart_packets_received = 0;
static unsigned long uart_bytes_received = 0;
static unsigned long uart_crc_errors = 0;
static unsigned long uart_sync_errors = 0;
static unsigned long local_samples_received = 0;
static unsigned long remote_samples_received = 0;
static unsigned long ble_notifications_sent = 0;
static unsigned long ble_send_errors = 0;
static unsigned long timer_samples_generated = 0;
static unsigned long commands_forwarded = 0;
static unsigned long command_responses_received = 0;
static unsigned long command_timeouts = 0;
static unsigned long ble_commands_received = 0;
static unsigned long ble_command_errors = 0;

// Response capture for command forwarding
static volatile bool command_response_ready = false;
static String command_response_data = "";
static SemaphoreHandle_t command_response_mutex = NULL;

// ============================================================================
// BLE TRANSMISSION CONFIGURATION
// ============================================================================

#define BLE_SAMPLE_DECIMATION 1  // Send every sample for maximum throughput

static unsigned long sample_decimation_counter = 0;
static unsigned long samples_batched = 0;
static unsigned long ble_packets_sent = 0;

// Compact 8-channel load cell sample using 16-bit values
struct __attribute__((packed)) CompactLoadCellSample {
    int16_t local_lc[4];    // Local load cells 1-4 (8 bytes)
    int16_t remote_lc[4];   // Remote load cells 5-8 (8 bytes)
};  // Total: 16 bytes per sample

#define SAMPLES_PER_BLE_PACKET 10
struct __attribute__((packed)) OptimizedBLEPacket {
    uint8_t sample_count;
    CompactLoadCellSample samples[SAMPLES_PER_BLE_PACKET];
};  // Total: 1 + (16 * 10) = 161 bytes

static OptimizedBLEPacket current_ble_packet;
static int current_sample_count = 0;

// ============================================================================
// UART PACKET PROCESSING
// ============================================================================

static inline uint16_t calculate_uart_crc(const UartPacket* packet) {
    const uint8_t* data = (const uint8_t*)packet + 2;
    uint32_t length = sizeof(UartPacket) - 4;
    return crc16_ccitt_false(data, length);
}

static inline bool validate_uart_packet(const UartPacket* packet) {
    if (packet->sync[0] != 0xAA || packet->sync[1] != 0x55) {
        uart_sync_errors++;
        return false;
    }
    
    uint16_t expected_crc = calculate_uart_crc(packet);
    if (packet->crc16 != expected_crc) {
        uart_crc_errors++;
        return false;
    }
    
    return true;
}

static inline void process_uart_packet(const UartPacket* packet) {
    if (!validate_uart_packet(packet)) {
        return;
    }
    
    uart_packets_received++;
    uart_bytes_received += sizeof(UartPacket);
    
    switch (packet->type) {
        case 'L':
            local_samples_received++;
            add_to_queue('L', packet->lc, packet->frame_idx, packet->sample_idx);
            break;
            
        case 'R':
            remote_samples_received++;
            add_to_queue('R', packet->lc, packet->frame_idx, packet->sample_idx);
            break;
    }
}

// ============================================================================
// BATTERY PACKET PROCESSING
// ============================================================================

static inline uint16_t calculate_battery_crc(const UartBatteryPacket* packet) {
    const uint8_t* data = (const uint8_t*)packet + 2;
    uint32_t length = sizeof(UartBatteryPacket) - 4;
    return crc16_ccitt_false(data, length);
}

static inline bool validate_battery_packet(const UartBatteryPacket* packet) {
    if (packet->sync[0] != 0xBB || packet->sync[1] != 0x55) {
        return false;
    }
    
    uint16_t expected_crc = calculate_battery_crc(packet);
    return packet->crc16 == expected_crc;
}

static inline void process_battery_packet(const UartBatteryPacket* packet) {
    if (!validate_battery_packet(packet)) {
        return;
    }
    
    battery_local_voltage = packet->local_voltage;
    battery_local_soc = packet->local_soc;
    battery_remote_voltage = packet->remote_voltage;
    battery_remote_soc = packet->remote_soc;
    last_battery_received_ms = millis();
    battery_packets_received++;
}

static void send_battery_ble_notification() {
    // Only send if we have recent battery data (within 10 seconds)
    if (last_battery_received_ms == 0 || (millis() - last_battery_received_ms) > 10000) return;
    
    // Update Standard BLE Battery Level (0x2A19) - use MIN of both batteries (worst-case)
    // This gives a conservative estimate for overall system battery level
    if (pStdBatteryLevelChar != nullptr) {
        uint8_t localLevel = (uint8_t)constrain((int)battery_local_soc, 0, 100);
        uint8_t remoteLevel = (uint8_t)constrain((int)battery_remote_soc, 0, 100);
        uint8_t batteryLevel = min(localLevel, remoteLevel);
        pStdBatteryLevelChar->setValue(&batteryLevel, 1);
        
        if (deviceConnected) {
            try { pStdBatteryLevelChar->notify(); } catch (...) {}
        }
    }
}

// Send battery status JSON response for BAT command
static void send_battery_json_response() {
    char json[200];
    snprintf(json, sizeof(json), 
        "{\"target\":\"BLE\",\"cmd\":\"BAT\",\"ok\":true,"
        "\"local\":{\"v\":%.2f,\"pct\":%.1f},"
        "\"remote\":{\"v\":%.2f,\"pct\":%.1f},\"ms\":0}",
        battery_local_voltage, battery_local_soc,
        battery_remote_voltage, battery_remote_soc);
    send_ble_json_response(json);
}

// ============================================================================
// BLE JSON RESPONSE FUNCTIONS
// ============================================================================

// Send JSON response via BLE Command characteristic (bidirectional)
static void send_ble_json_response(const char* json) {
    if (deviceConnected && pCmdCharacteristic != nullptr) {
        pCmdCharacteristic->setValue((uint8_t*)json, strlen(json));
        pCmdCharacteristic->notify();
        Serial.printf("[BLE] -> %s\n", json);
    }
}

// Parse Teensy response and send as JSON via BLE
// Response formats from Teensy: "TARE:OK", "SHOW:LC1=off,a,n;LC2=...", "READ:v1;v2;v3;v4;tot", etc.
static void send_json_response(const char* prefix, const char* response, unsigned long round_trip_ms) {
    char json[300];
    
    // Determine if this is LOCAL or REMOTE
    bool is_remote = (strcmp(prefix, "REMOTE") == 0);
    const char* target = is_remote ? "REMOTE" : "LOCAL";
    
    // Parse the response format: "CMD:DATA" or "CMD:STATUS"
    String resp = String(response);
    int colon_pos = resp.indexOf(':');
    
    if (colon_pos <= 0) {
        // Unknown format - send raw
        snprintf(json, sizeof(json), "{\"target\":\"%s\",\"raw\":\"%s\",\"ms\":%lu}", 
                 target, response, round_trip_ms);
        send_ble_json_response(json);
        return;
    }
    
    String cmd = resp.substring(0, colon_pos);
    String data = resp.substring(colon_pos + 1);
    
    // Handle different command responses
    if (cmd == "TARE" || cmd == "FIT" || cmd == "CLEAR" || cmd == "ADD" || cmd == "ADD_CH") {
        // Simple OK/ERR responses
        bool ok = (data == "OK");
        if (ok) {
            snprintf(json, sizeof(json), "{\"target\":\"%s\",\"cmd\":\"%s\",\"ok\":true,\"ms\":%lu}", 
                     target, cmd.c_str(), round_trip_ms);
        } else {
            snprintf(json, sizeof(json), "{\"target\":\"%s\",\"cmd\":\"%s\",\"ok\":false,\"err\":\"%s\",\"ms\":%lu}", 
                     target, cmd.c_str(), data.c_str(), round_trip_ms);
        }
    }
    else if (cmd == "SHOW") {
        // Format: SHOW:LC1=off,a,n;LC2=off,a,n;LC3=off,a,n;LC4=off,a,n
        // Parse into JSON array
        long off[4] = {0};
        float a[4] = {0};
        int n[4] = {0};
        
        // Parse each LC section
        int lc_idx = 0;
        int start = 0;
        while (lc_idx < 4) {
            int semi = data.indexOf(';', start);
            String lc_data = (semi > 0) ? data.substring(start, semi) : data.substring(start);
            
            // Parse "LC1=off,a,n" - skip "LCx=" prefix
            int eq = lc_data.indexOf('=');
            if (eq > 0) {
                String vals = lc_data.substring(eq + 1);
                int c1 = vals.indexOf(',');
                int c2 = vals.indexOf(',', c1 + 1);
                if (c1 > 0 && c2 > 0) {
                    off[lc_idx] = vals.substring(0, c1).toInt();
                    a[lc_idx] = vals.substring(c1 + 1, c2).toFloat();
                    n[lc_idx] = vals.substring(c2 + 1).toInt();
                }
            }
            
            lc_idx++;
            if (semi < 0) break;
            start = semi + 1;
        }
        
        snprintf(json, sizeof(json), 
                 "{\"target\":\"%s\",\"cmd\":\"SHOW\",\"lc\":[{\"off\":%ld,\"a\":%.6f,\"n\":%d},{\"off\":%ld,\"a\":%.6f,\"n\":%d},{\"off\":%ld,\"a\":%.6f,\"n\":%d},{\"off\":%ld,\"a\":%.6f,\"n\":%d}],\"ms\":%lu}",
                 target, off[0], a[0], n[0], off[1], a[1], n[1], off[2], a[2], n[2], off[3], a[3], n[3], round_trip_ms);
    }
    else if (cmd == "READ" || cmd == "READ_RAW") {
        // Format: READ:v1;v2;v3;v4;tot
        long v[5] = {0};
        int idx = 0;
        int start = 0;
        while (idx < 5) {
            int semi = data.indexOf(';', start);
            String val = (semi > 0) ? data.substring(start, semi) : data.substring(start);
            v[idx] = val.toInt();
            idx++;
            if (semi < 0) break;
            start = semi + 1;
        }
        snprintf(json, sizeof(json), "{\"target\":\"%s\",\"cmd\":\"%s\",\"v\":[%ld,%ld,%ld,%ld,%ld],\"ms\":%lu}", 
                 target, cmd.c_str(), v[0], v[1], v[2], v[3], v[4], round_trip_ms);
    }
    else if (cmd == "PTS") {
        // Format: PTS:n1;n2;n3;n4
        int n[4] = {0};
        int idx = 0;
        int start = 0;
        while (idx < 4) {
            int semi = data.indexOf(';', start);
            String val = (semi > 0) ? data.substring(start, semi) : data.substring(start);
            n[idx] = val.toInt();
            idx++;
            if (semi < 0) break;
            start = semi + 1;
        }
        snprintf(json, sizeof(json), "{\"target\":\"%s\",\"cmd\":\"PTS\",\"n\":[%d,%d,%d,%d],\"ms\":%lu}", 
                 target, n[0], n[1], n[2], n[3], round_trip_ms);
    }
    else if (data == "TIMEOUT" || data == "ESPNOW_ERR") {
        // Error responses
        snprintf(json, sizeof(json), "{\"target\":\"%s\",\"cmd\":\"%s\",\"ok\":false,\"err\":\"%s\",\"ms\":%lu}", 
                 target, cmd.c_str(), data.c_str(), round_trip_ms);
    }
    else {
        // Unknown command - send as generic response
        snprintf(json, sizeof(json), "{\"target\":\"%s\",\"cmd\":\"%s\",\"data\":\"%s\",\"ms\":%lu}", 
                 target, cmd.c_str(), data.c_str(), round_trip_ms);
    }
    
    send_ble_json_response(json);
}

// Send simple status JSON for control commands (START, STOP, etc.)
static void send_control_json_response(const char* target, const char* cmd, bool ok, unsigned long ms) {
    char json[150];
    snprintf(json, sizeof(json), "{\"target\":\"%s\",\"cmd\":\"%s\",\"ok\":%s,\"ms\":%lu}", 
             target, cmd, ok ? "true" : "false", ms);
    send_ble_json_response(json);
}

// Send PING response
static void send_ping_json_response(const char* target, unsigned long ms) {
    char json[100];
    snprintf(json, sizeof(json), "{\"target\":\"%s\",\"cmd\":\"PING\",\"ok\":true,\"ms\":%lu}", target, ms);
    send_ble_json_response(json);
}

// Send timeout error
static void send_timeout_json_response(const char* target, const char* cmd, unsigned long ms) {
    char json[150];
    snprintf(json, sizeof(json), "{\"target\":\"%s\",\"cmd\":\"%s\",\"ok\":false,\"err\":\"TIMEOUT\",\"ms\":%lu}", 
             target, cmd, ms);
    send_ble_json_response(json);
}

// Parse STATUS response from Teensy (format: STATUS:state=RUNNING,led=ON,frames=1234)
static void parse_status_response(const String& response, String& state, String& led) {
    state = "UNKNOWN";
    led = "UNKNOWN";
    
    // Find state= field
    int state_pos = response.indexOf("state=");
    if (state_pos >= 0) {
        int comma_pos = response.indexOf(',', state_pos);
        if (comma_pos > state_pos) {
            state = response.substring(state_pos + 6, comma_pos);
        }
    }
    
    // Find led= field
    int led_pos = response.indexOf("led=");
    if (led_pos >= 0) {
        int comma_pos = response.indexOf(',', led_pos);
        if (comma_pos > led_pos) {
            led = response.substring(led_pos + 4, comma_pos);
        } else {
            // No comma after led, take rest of string
            led = response.substring(led_pos + 4);
            // Remove any trailing non-alphanumeric characters
            led.trim();
            int end_pos = led.indexOf(',');
            if (end_pos < 0) end_pos = led.indexOf(';');
            if (end_pos >= 0) led = led.substring(0, end_pos);
        }
    }
}

// Send combined STATUS JSON response (includes status, battery, and key stats)
static void send_status_json_response(unsigned long ms) {
    unsigned long uptime_sec = millis() / 1000;
    char json[512];
    snprintf(json, sizeof(json), 
        "{\"target\":\"BLE\",\"cmd\":\"STATUS\",\"ok\":true,"
        "\"local\":{\"state\":\"%s\",\"led\":\"%s\",\"bat\":{\"v\":%.2f,\"pct\":%.1f}},"
        "\"remote\":{\"state\":\"%s\",\"led\":\"%s\",\"bat\":{\"v\":%.2f,\"pct\":%.1f}},"
        "\"ble\":\"%s\",\"uptime\":%lu,\"ms\":%lu}",
        status_local_state.c_str(), status_local_led.c_str(),
        battery_local_voltage, battery_local_soc,
        status_remote_state.c_str(), status_remote_led.c_str(),
        battery_remote_voltage, battery_remote_soc,
        deviceConnected ? "CONNECTED" : "DISCONNECTED",
        uptime_sec, ms);
    send_ble_json_response(json);
}

// ============================================================================
// BLE COMMAND PROCESSING
// ============================================================================

static void forward_command_to_rx_radio(String command) {
    bool is_ping = (command == "LOCAL_PING" || command == "REMOTE_PING");
    bool is_cal_command = (command.startsWith("LOCAL_CAL_") || command.startsWith("REMOTE_CAL_"));
    bool is_remote = (command.startsWith("REMOTE_") || command.startsWith("ALL_"));
    bool is_local = command.startsWith("LOCAL_") || (!is_remote && !command.startsWith("ALL_"));
    
    // Determine target for JSON response
    const char* target = "LOCAL";
    if (command.startsWith("REMOTE_")) target = "REMOTE";
    else if (command.startsWith("ALL_")) target = "ALL";
    
    // Extract base command (e.g., "START" from "REMOTE_START", "CAL_TARE" from "LOCAL_CAL_TARE")
    String base_cmd = command;
    if (command.startsWith("LOCAL_")) base_cmd = command.substring(6);
    else if (command.startsWith("REMOTE_")) base_cmd = command.substring(7);
    else if (command.startsWith("ALL_")) base_cmd = command.substring(4);
    
    Serial.printf("[BLE_SLAVE] Forwarding '%s' to RX Radio...\n", command.c_str());
        
        // Clear previous response
        if (command_response_mutex != NULL && xSemaphoreTake(command_response_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            command_response_ready = false;
            command_response_data = "";
            xSemaphoreGive(command_response_mutex);
        }
        
        unsigned long start_time = millis();
        Serial2.println(command);
        Serial2.flush();
        commands_forwarded++;
        
    // Wait for response (longer timeout for calibration commands)
    unsigned long timeout = millis() + (is_cal_command ? 15000 : 5000);
        bool got_response = false;
        String response_text = "";
        
        while (millis() < timeout) {
            if (command_response_mutex != NULL && xSemaphoreTake(command_response_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                if (command_response_ready) {
                    response_text = command_response_data;
                    command_response_ready = false;
                    got_response = true;
                    xSemaphoreGive(command_response_mutex);
                    break;
                }
                xSemaphoreGive(command_response_mutex);
            }
            delay(10);
        }
        
        unsigned long round_trip_ms = millis() - start_time;
        
        if (got_response) {
        // Parse prefix (LOCAL: or REMOTE:) from response
        String prefix = "";
                String actual_response = response_text;
                if (response_text.startsWith("LOCAL:")) {
            prefix = "LOCAL";
            actual_response = response_text.substring(6);
                } else if (response_text.startsWith("REMOTE:")) {
            prefix = "REMOTE";
            actual_response = response_text.substring(7);
        }
        
        Serial.printf("[BLE_SLAVE] Response: %s (%lu ms)\n", actual_response.c_str(), round_trip_ms);
            command_responses_received++;
        
        // Send JSON response via BLE
            if (is_ping) {
            send_ping_json_response(target, round_trip_ms);
        } else if (is_cal_command) {
            // Calibration commands - parse the detailed response
            send_json_response(target, actual_response.c_str(), round_trip_ms);
            } else {
            // Control commands (START, STOP, etc.) - check for success indicators
            bool ok = (actual_response.indexOf("OK") >= 0 ||
                       actual_response.indexOf("RUNNING") >= 0 ||
                       actual_response.indexOf("STOPPED") >= 0);
            send_control_json_response(target, base_cmd.c_str(), ok, round_trip_ms);
            }
    } else {
        Serial.printf("[BLE_SLAVE] ✗ TIMEOUT after %lu ms\n", round_trip_ms);
            command_timeouts++;
        
        // Send timeout error via BLE
        send_timeout_json_response(target, base_cmd.c_str(), round_trip_ms);
    }
}

static void process_ble_command(String command) {
    command.trim();
    command.toUpperCase();
    
    if (command.length() == 0) {
        Serial.println("[BLE_SLAVE] ⚠ Empty BLE command received");
        ble_command_errors++;
        return;
    }
    
    Serial.printf("[BLE_SLAVE] BLE Command received: %s\n", command.c_str());
    ble_commands_received++;
    
    // Local commands (handled by BLE Slave)
    if (command == "STATS") {
        show_detailed_stats();
        send_control_json_response("BLE", "STATS", true, 0);
        return;
    }
    
    if (command == "RESET_STATS") {
        reset_all_stats();
        send_control_json_response("BLE", "RESET_STATS", true, 0);
        return;
    }
    
    if (command == "HELP") {
        show_help();
        send_control_json_response("BLE", "HELP", true, 0);
        return;
    }

    if (command == "BAT") {
        Serial.printf("[BLE_SLAVE] Battery: Local=%.2fV (%.1f%%), Remote=%.2fV (%.1f%%)\n",
                     battery_local_voltage, battery_local_soc, 
                     battery_remote_voltage, battery_remote_soc);
        send_battery_json_response();
        return;
    }
    
    // BLE_STATUS - Request-based BLE connection status (replaces periodic broadcast)
    if (command == "BLE_STATUS") {
        Serial.printf("[BLE_SLAVE] BLE Status requested: %s\n", deviceConnected ? "CONNECTED" : "DISCONNECTED");
        // Send status to RX Radio (which forwards to Teensys)
        Serial2.println(deviceConnected ? "BLE_CONNECTED" : "BLE_DISCONNECTED");
        Serial2.flush();
        send_control_json_response("BLE", "BLE_STATUS", true, 0);
        return;
    }
    
    // STATUS - Get combined status from both plates (LED on/off, acquisition state)
    if (command == "STATUS") {
        Serial.println("[BLE_SLAVE] STATUS command - querying both plates...");
        unsigned long start_time = millis();
        
        // Reset status values
        status_local_state = "UNKNOWN";
        status_local_led = "UNKNOWN";
        status_remote_state = "UNKNOWN";
        status_remote_led = "UNKNOWN";
        
        // Clear previous response
        if (command_response_mutex != NULL && xSemaphoreTake(command_response_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            command_response_ready = false;
            command_response_data = "";
            xSemaphoreGive(command_response_mutex);
        }
        
        // Query LOCAL status
        Serial.println("[BLE_SLAVE] Querying LOCAL_STATUS...");
        Serial2.println("LOCAL_STATUS");
        Serial2.flush();
        
        // Wait for local response
        unsigned long timeout = millis() + 3000;
        bool got_local = false;
        
        while (millis() < timeout) {
            if (command_response_mutex != NULL && xSemaphoreTake(command_response_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                if (command_response_ready) {
                    String response = command_response_data;
                    command_response_ready = false;
                    xSemaphoreGive(command_response_mutex);
                    
                    // Check if this is a LOCAL response
                    if (response.startsWith("LOCAL:")) {
                        String status_part = response.substring(6);
                        parse_status_response(status_part, status_local_state, status_local_led);
                        Serial.printf("[BLE_SLAVE] LOCAL: state=%s, led=%s\n", 
                                     status_local_state.c_str(), status_local_led.c_str());
                        got_local = true;
                        break;
                    }
                    // If not local, might be stale data, continue waiting
                } else {
                    xSemaphoreGive(command_response_mutex);
                }
            }
            delay(10);
        }
        
        if (!got_local) {
            Serial.println("[BLE_SLAVE] LOCAL_STATUS timeout");
        }
        
        // Clear for next response
        if (command_response_mutex != NULL && xSemaphoreTake(command_response_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            command_response_ready = false;
            command_response_data = "";
            xSemaphoreGive(command_response_mutex);
        }
        
        // Query REMOTE status
        Serial.println("[BLE_SLAVE] Querying REMOTE_STATUS...");
        Serial2.println("REMOTE_STATUS");
        Serial2.flush();
        
        // Wait for remote response
        timeout = millis() + 3000;
        bool got_remote = false;
        
        while (millis() < timeout) {
            if (command_response_mutex != NULL && xSemaphoreTake(command_response_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                if (command_response_ready) {
                    String response = command_response_data;
                    command_response_ready = false;
                    xSemaphoreGive(command_response_mutex);
                    
                    // Check if this is a REMOTE response
                    if (response.startsWith("REMOTE:")) {
                        String status_part = response.substring(7);
                        parse_status_response(status_part, status_remote_state, status_remote_led);
                        Serial.printf("[BLE_SLAVE] REMOTE: state=%s, led=%s\n", 
                                     status_remote_state.c_str(), status_remote_led.c_str());
                        got_remote = true;
                        break;
                    }
                } else {
                    xSemaphoreGive(command_response_mutex);
                }
            }
            delay(10);
        }
        
        if (!got_remote) {
            Serial.println("[BLE_SLAVE] REMOTE_STATUS timeout");
        }
        
        unsigned long round_trip_ms = millis() - start_time;
        Serial.printf("[BLE_SLAVE] STATUS complete (%lu ms)\n", round_trip_ms);
        
        // Send combined JSON response
        send_status_json_response(round_trip_ms);
        return;
    }

    // Valid commands to forward to ESP32 RX Radio
    // All commands use consistent prefixes: LOCAL_*, REMOTE_*, ALL_*
    if (command == "LOCAL_START" || command == "LOCAL_STOP" || command == "LOCAL_RESTART" || command == "LOCAL_RESET" ||
        command == "REMOTE_START" || command == "REMOTE_STOP" || command == "REMOTE_RESTART" || command == "REMOTE_RESET" ||
        command == "ALL_START" || command == "ALL_STOP" || command == "ALL_RESTART" || command == "ALL_RESET" ||
        command == "LOCAL_PING" || command == "REMOTE_PING" ||
        command == "LOCAL_LED_ON" || command == "LOCAL_LED_OFF" ||
        command == "REMOTE_LED_ON" || command == "REMOTE_LED_OFF" ||
        command == "ALL_LED_ON" || command == "ALL_LED_OFF" ||
        command == "LOCAL_MOCK_ON" || command == "LOCAL_MOCK_OFF" ||
        command == "REMOTE_MOCK_ON" || command == "REMOTE_MOCK_OFF" ||
        command == "ALL_MOCK_ON" || command == "ALL_MOCK_OFF" ||
        command.startsWith("LOCAL_CAL_") || command.startsWith("REMOTE_CAL_")) {
        
        forward_command_to_rx_radio(command);
    } else {
        Serial.printf("[BLE_SLAVE] ✗ Unknown BLE command: '%s'\n", command.c_str());
        ble_command_errors++;
        
        // Send error via BLE
        char json[150];
        snprintf(json, sizeof(json), "{\"cmd\":\"%s\",\"ok\":false,\"err\":\"UNKNOWN_CMD\"}", command.c_str());
        send_ble_json_response(json);
    }
}

// ============================================================================
// BLE SERVER CALLBACKS
// ============================================================================

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("[BLE_SLAVE] ✓ BLE Client connected");
        // Notify Teensy about BLE connection status
        Serial2.println("BLE_CONNECTED");
        Serial2.flush();
    };

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("[BLE_SLAVE] ✗ BLE Client disconnected");
        // Notify Teensy about BLE disconnection
        Serial2.println("BLE_DISCONNECTED");
        Serial2.flush();
    }
};

class MyCommandCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
        String command = pCharacteristic->getValue().c_str();
        
        if (command.length() > 0) {
            Serial.printf("[BLE_SLAVE] BLE Write received: '%s' (%d bytes)\n", command.c_str(), command.length());
            process_ble_command(command);
        }
    }
};

// ============================================================================
// DATA STORE FUNCTIONS
// ============================================================================

static inline void add_to_queue(uint8_t type, const int32_t* lc, uint16_t frame_idx, uint8_t sample_idx) {
    uint32_t now = millis();
    last_data_received_ms = now;  // Track when we last received fresh data
    
    if (type == 'L') {
        if (data_store.local_count < QUEUE_SIZE) {
            SampleData* sample = &data_store.local_queue[data_store.local_tail];
            memcpy(sample->lc, lc, sizeof(sample->lc));
            sample->timestamp = now;
            sample->frame_idx = frame_idx;
            sample->sample_idx = sample_idx;
            sample->consumed = false;
            
            data_store.local_tail = (data_store.local_tail + 1) % QUEUE_SIZE;
            data_store.local_count++;
        } else {
            SampleData* sample = &data_store.local_queue[data_store.local_tail];
            memcpy(sample->lc, lc, sizeof(sample->lc));
            sample->timestamp = now;
            sample->frame_idx = frame_idx;
            sample->sample_idx = sample_idx;
            sample->consumed = false;
            
            data_store.local_tail = (data_store.local_tail + 1) % QUEUE_SIZE;
            data_store.local_head = (data_store.local_head + 1) % QUEUE_SIZE;
        }
    } else if (type == 'R') {
        if (data_store.remote_count < QUEUE_SIZE) {
            SampleData* sample = &data_store.remote_queue[data_store.remote_tail];
            memcpy(sample->lc, lc, sizeof(sample->lc));
            sample->timestamp = now;
            sample->frame_idx = frame_idx;
            sample->sample_idx = sample_idx;
            sample->consumed = false;
            
            data_store.remote_tail = (data_store.remote_tail + 1) % QUEUE_SIZE;
            data_store.remote_count++;
        } else {
            SampleData* sample = &data_store.remote_queue[data_store.remote_tail];
            memcpy(sample->lc, lc, sizeof(sample->lc));
            sample->timestamp = now;
            sample->frame_idx = frame_idx;
            sample->sample_idx = sample_idx;
            sample->consumed = false;
            
            data_store.remote_tail = (data_store.remote_tail + 1) % QUEUE_SIZE;
            data_store.remote_head = (data_store.remote_head + 1) % QUEUE_SIZE;
        }
    }
}

static inline bool get_next_sample(int32_t* local_lc, int32_t* remote_lc) {
    // Check if data flow has stopped (stale data)
    unsigned long now = millis();
    unsigned long age = (last_data_received_ms > 0) ? (now - last_data_received_ms) : 999999;
    
    if (age > DATA_TIMEOUT_MS) {
        // Data flow stopped - reset fallback flags and don't send
        data_store.has_local_data = false;
        data_store.has_remote_data = false;
        return false;
    }
    
    bool got_local = false;
    bool got_remote = false;
    
    // Initialize to zero (missing channels show as 0)
    memset(local_lc, 0, 4 * sizeof(int32_t));
    memset(remote_lc, 0, 4 * sizeof(int32_t));
    
    if (data_store.local_count > 0) {
        SampleData* sample = &data_store.local_queue[data_store.local_head];
        if (!sample->consumed) {
            memcpy(local_lc, sample->lc, sizeof(sample->lc));
            memcpy(data_store.last_local_lc, sample->lc, sizeof(sample->lc));
            sample->consumed = true;
            data_store.has_local_data = true;
            got_local = true;
            
            data_store.local_head = (data_store.local_head + 1) % QUEUE_SIZE;
            data_store.local_count--;
        }
    }
    
    if (data_store.remote_count > 0) {
        SampleData* sample = &data_store.remote_queue[data_store.remote_head];
        if (!sample->consumed) {
            memcpy(remote_lc, sample->lc, sizeof(sample->lc));
            memcpy(data_store.last_remote_lc, sample->lc, sizeof(sample->lc));
            sample->consumed = true;
            data_store.has_remote_data = true;
            got_remote = true;
            
            data_store.remote_head = (data_store.remote_head + 1) % QUEUE_SIZE;
            data_store.remote_count--;
        }
    }
    
    // Use last known data as fallback (only while data is flowing)
    if (!got_local && data_store.has_local_data) {
        memcpy(local_lc, data_store.last_local_lc, sizeof(data_store.last_local_lc));
        got_local = true;
    }
    
    if (!got_remote && data_store.has_remote_data) {
        memcpy(remote_lc, data_store.last_remote_lc, sizeof(data_store.last_remote_lc));
        got_remote = true;
    }
    
    // Return true if we have ANY data (local OR remote)
    return got_local || got_remote;
}

// ============================================================================
// HARDWARE TIMER FUNCTIONS (1000 Hz)
// ============================================================================

void IRAM_ATTR timer_sample_callback() {
    timer_interrupts_count++;
    timer_sample_ready = true;
}

static inline void create_8channel_sample() {
    int32_t local_lc[4];
    int32_t remote_lc[4];
    
    if (!get_next_sample(local_lc, remote_lc)) {
        return;
    }
    
    timer_samples_generated++;
    
    sample_decimation_counter++;
    if (sample_decimation_counter % BLE_SAMPLE_DECIMATION != 0) {
        return;
    }
    
    if (current_sample_count < SAMPLES_PER_BLE_PACKET) {
        CompactLoadCellSample* sample = &current_ble_packet.samples[current_sample_count];
        
        for (int i = 0; i < 4; i++) {
            sample->local_lc[i] = (int16_t)constrain(local_lc[i], -32768, 32767);
            sample->remote_lc[i] = (int16_t)constrain(remote_lc[i], -32768, 32767);
        }
        current_sample_count++;
    }
    
    // Only send when we have a FULL batch of 10 samples
    // This ensures consistent packet sizes for reliable BLE transfer
    if (current_sample_count >= SAMPLES_PER_BLE_PACKET) {
        send_current_batch();
    }
}

// ============================================================================
// BATCH SAMPLE PROCESSING
// ============================================================================

static inline void send_current_batch() {
    // Only send if: samples ready AND connected
    // The data flow check in get_next_sample() already prevents sending when no data
    if (current_sample_count > 0 && deviceConnected) {
        current_ble_packet.sample_count = current_sample_count;
        size_t packet_size = 1 + (current_sample_count * sizeof(CompactLoadCellSample));
        
        try {
            pDataCharacteristic->setValue((uint8_t*)&current_ble_packet, packet_size);
            pDataCharacteristic->notify();
            ble_notifications_sent++;
            ble_packets_sent++;
            samples_batched += current_sample_count;
        } catch (...) {
            ble_send_errors++;
        }
        
        current_sample_count = 0;
    }
}

// ============================================================================
// UART READING TASK
// ============================================================================

static void uart_rx_task(void* param) {
    // Buffer for both packet types (use larger one)
    uint8_t packet_buffer[32];  // Max of UartPacket (24) or UartBatteryPacket (20)
    int bytes_received = 0;
    bool sync_found = false;
    uint8_t packet_type = 0;  // 0xAA = load cell, 0xBB = battery
    int bytes_needed = 0;
    
    Serial.printf("UART RX Task started, handling load cell (%d bytes) and battery (%d bytes) packets\n", 
                  (int)sizeof(UartPacket), (int)sizeof(UartBatteryPacket));
    
    while (true) {
        int available = Serial2.available();
        if (available > 0) {
            if (!sync_found) {
                uint8_t byte = Serial2.peek();
                
                // Check for text message prefix (##REMOTE: or ##LOCAL:)
                if (byte == '#') {
                    Serial2.read(); // consume the '#'
                    
                    // Check if it's ## (double hash) for calibration responses
                    if (Serial2.peek() == '#') {
                        Serial2.read(); // consume second '#'
                        
                        // Read single-line response (format: "REMOTE:response" or "LOCAL:response")
                        String text_response = Serial2.readStringUntil('\n');
                        text_response.trim();
                     
                     if (text_response.length() > 0) {
                            // Parse prefix and response
                             String prefix = "";
                            String response = text_response;
                            int colon_pos = text_response.indexOf(':');
                            if (colon_pos > 0 && colon_pos < 10) {
                                prefix = text_response.substring(0, colon_pos);
                                response = text_response.substring(colon_pos + 1);
                            }
                            
                            // Display response
                            Serial.printf("[%s] %s\n", prefix.c_str(), response.c_str());
                            
                            // Store for command response callback
                            if (command_response_mutex != NULL && xSemaphoreTake(command_response_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                                command_response_data = text_response;
                                command_response_ready = true;
                                xSemaphoreGive(command_response_mutex);
                            }
                        }
                    }
                    bytes_received = 0;
                    continue;
                }
                
                // Check for text commands from RX Radio using @@ prefix (e.g., @@BLE_STATUS)
                // Use @@ (0x40 0x40) as command prefix - won't conflict with binary data
                if (byte == '@') {
                    Serial2.read();  // consume first '@'
                    if (Serial2.peek() == '@') {
                        Serial2.read();  // consume second '@'
                        
                        // Read the command line
                        String uart_command = Serial2.readStringUntil('\n');
                        uart_command.trim();
                        uart_command.toUpperCase();
                        
                        if (uart_command.length() > 0) {
                            Serial.printf("[BLE_SLAVE] UART command from RX Radio: %s\n", uart_command.c_str());
                            
                            // Handle BLE_STATUS request from Teensy (on boot)
                            if (uart_command == "BLE_STATUS") {
                                // Send current BLE status back to RX Radio (which forwards to Teensy)
                                Serial2.println(deviceConnected ? "BLE_CONNECTED" : "BLE_DISCONNECTED");
                                Serial2.flush();
                                Serial.printf("[BLE_SLAVE] Sent BLE status: %s\n", deviceConnected ? "CONNECTED" : "DISCONNECTED");
                            }
                        }
                    }
                    bytes_received = 0;
                    continue;
                }
                
                byte = Serial2.read();
                
                // Check for load cell packet sync (0xAA 0x55)
                if (byte == 0xAA) {
                    packet_buffer[0] = byte;
                    bytes_received = 1;
                    packet_type = 0xAA;
                }
                // Check for battery packet sync (0xBB 0x55)
                else if (byte == 0xBB) {
                    packet_buffer[0] = byte;
                    bytes_received = 1;
                    packet_type = 0xBB;
                }
                else if (bytes_received == 1 && byte == 0x55) {
                    packet_buffer[1] = byte;
                    bytes_received = 2;
                    sync_found = true;
                    
                    // Set expected packet size based on type
                    if (packet_type == 0xAA) {
                        bytes_needed = sizeof(UartPacket);
                    } else if (packet_type == 0xBB) {
                        bytes_needed = sizeof(UartBatteryPacket);
                    }
                } else {
                    bytes_received = 0;
                    packet_type = 0;
                }
            } else {
                int to_read = min(available, bytes_needed - bytes_received);
                
                if (to_read > 0) {
                    int actual_read = Serial2.readBytes(packet_buffer + bytes_received, to_read);
                    bytes_received += actual_read;
                    
                    if (bytes_received >= bytes_needed) {
                        // Process based on packet type
                        if (packet_type == 0xAA) {
                            process_uart_packet((UartPacket*)packet_buffer);
                        } else if (packet_type == 0xBB) {
                            process_battery_packet((UartBatteryPacket*)packet_buffer);
                        }
                        
                        bytes_received = 0;
                        sync_found = false;
                        packet_type = 0;
                    }
                }
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}

// ============================================================================
// BLE TRANSMISSION TASKS
// ============================================================================

static void timer_processing_task(void* param) {
    while (true) {
        if (timer_sample_ready) {
            timer_sample_ready = false;
            create_8channel_sample();
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void ble_tx_task(void* param) {
    // This task only handles cleanup when data flow stops
    // Normal packet sending is done in create_8channel_sample() when batch is full
    static unsigned long last_sample_count = 0;
    static unsigned long stale_count = 0;
    
    while (true) {
        if (deviceConnected) {
            // Only flush if samples have been pending for too long (data flow stopped)
            if (current_sample_count > 0) {
                if (current_sample_count == last_sample_count) {
                    stale_count++;
                    // If samples haven't changed for 100ms (10 iterations), flush them
                    if (stale_count >= 10) {
                        send_current_batch();
                        stale_count = 0;
                    }
                } else {
                    stale_count = 0;
                }
                last_sample_count = current_sample_count;
            } else {
                stale_count = 0;
                last_sample_count = 0;
            }
        } else {
            current_sample_count = 0;
            stale_count = 0;
            last_sample_count = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ============================================================================
// STATISTICS DISPLAY
// ============================================================================

static void show_detailed_stats() {
    unsigned long uptime_sec = millis() / 1000;
    unsigned long uptime_min = uptime_sec / 60;
    unsigned long uptime_hrs = uptime_min / 60;
    
    Serial.println("\n[BLE_SLAVE] === SYSTEM STATISTICS ===");
    Serial.printf("  Uptime: %02lu:%02lu:%02lu\n", uptime_hrs, uptime_min % 60, uptime_sec % 60);
    Serial.println("  --- UART ---");
    Serial.printf("  Packets: %lu (%lu bytes), CRC errors: %lu, Sync errors: %lu\n", 
                 uart_packets_received, uart_bytes_received, uart_crc_errors, uart_sync_errors);
    Serial.println("  --- Samples ---");
    Serial.printf("  Local: %lu | Remote: %lu | Timer: %lu | Batched: %lu\n", 
                 local_samples_received, remote_samples_received, timer_samples_generated, samples_batched);
    Serial.println("  --- BLE ---");
    Serial.printf("  Status: %s | Notifications: %lu | Packets: %lu | Errors: %lu\n", 
                 deviceConnected ? "CONNECTED" : "DISCONNECTED", ble_notifications_sent, ble_packets_sent, ble_send_errors);
    Serial.println("  --- Battery ---");
    Serial.printf("  Local: %.2fV (%.1f%%) | Remote: %.2fV (%.1f%%) | Packets: %lu\n", 
                 battery_local_voltage, battery_local_soc, battery_remote_voltage, battery_remote_soc, battery_packets_received);
    Serial.println("  --- Commands ---");
    Serial.printf("  BLE cmds: %lu | Forwarded: %lu | Responses: %lu | Timeouts: %lu\n", 
                 ble_commands_received, commands_forwarded, command_responses_received, command_timeouts);
    Serial.println("  --- Queues ---");
    Serial.printf("  Local: %d/%d | Remote: %d/%d | Batch: %d/%d | Heap: %lu bytes\n", 
                 data_store.local_count, QUEUE_SIZE, data_store.remote_count, QUEUE_SIZE, 
                 current_sample_count, SAMPLES_PER_BLE_PACKET, ESP.getFreeHeap());
    Serial.println("=====================================\n");
}

static void reset_all_stats() {
            uart_packets_received = 0;
            uart_bytes_received = 0;
            uart_crc_errors = 0;
            uart_sync_errors = 0;
            local_samples_received = 0;
            remote_samples_received = 0;
            ble_notifications_sent = 0;
            ble_send_errors = 0;
    timer_samples_generated = 0;
            commands_forwarded = 0;
            command_responses_received = 0;
            command_timeouts = 0;
            ble_commands_received = 0;
            ble_command_errors = 0;
    sample_decimation_counter = 0;
    samples_batched = 0;
    ble_packets_sent = 0;
    
    Serial.println("[BLE_SLAVE] ✓ All statistics reset");
}

static void show_help() {
    Serial.println("\n[BLE_SLAVE] === AVAILABLE COMMANDS ===");
    Serial.println("  Control:");
    Serial.println("    START, STOP, RESTART, RESET (local Teensy)");
    Serial.println("    REMOTE_START/STOP/RESTART/RESET (remote Teensy)");
    Serial.println("    ALL_START/STOP/RESTART/RESET (both)");
    Serial.println("  LED Control:");
    Serial.println("    LOCAL_LED_ON, LOCAL_LED_OFF (local Teensy)");
    Serial.println("    REMOTE_LED_ON, REMOTE_LED_OFF (remote Teensy)");
    Serial.println("    ALL_LED_ON, ALL_LED_OFF (both)");
    Serial.println("  Ping:");
    Serial.println("    LOCAL_PING, REMOTE_PING");
    Serial.println("  Calibration:");
    Serial.println("    LOCAL_CAL_TARE, LOCAL_CAL_SHOW, LOCAL_CAL_ADD_<kg>");
    Serial.println("    LOCAL_CAL_ADD_CH_<ch>_<kg>, LOCAL_CAL_CLEAR, LOCAL_CAL_READ");
    Serial.println("    REMOTE_CAL_TARE, REMOTE_CAL_SHOW, etc.");
    Serial.println("  Stats/Info:");
    Serial.println("    STATS, RESET_STATS, HELP, BAT");
    Serial.println("\n=== BLE CHARACTERISTICS ===");
    Serial.println("  Service: 12345678-1234-1234-1234-123456789abc");
    Serial.println("  Data:    87654321-... (NOTIFY binary sensor data)");
    Serial.println("  Cmd:     11111111-... (WRITE cmd, NOTIFY JSON response)");
    Serial.println("\n=== BAT COMMAND RESPONSE ===");
    Serial.println("  {\"target\":\"BLE\",\"cmd\":\"BAT\",\"ok\":true,");
    Serial.println("   \"local\":{\"v\":4.12,\"pct\":85.0},");
    Serial.println("   \"remote\":{\"v\":3.98,\"pct\":72.0},\"ms\":0}");
    Serial.println("\n=== STANDARD BLE SERVICES ===");
    Serial.println("  Battery Service: 0x180F (Battery Level: 0x2A19)");
    Serial.println("  Device Info: 0x180A (Manufacturer, Model, Serial, Versions)");
    Serial.println("  Appearance: Weight Scale icon (0x0B40)");
    Serial.println("=====================================\n");
}

// ============================================================================
// SERIAL COMMANDS
// ============================================================================

static void handle_serial_commands() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        command.toUpperCase();
        
        if (command.length() == 0) return;
        
        Serial.printf("[BLE_SLAVE] Command received: %s\n", command.c_str());
        
        // Forward control commands to ESP32 RX Radio
        if (command == "START" || command == "STOP" || command == "RESTART" || command == "RESET" ||
            command == "REMOTE_START" || command == "REMOTE_STOP" || command == "REMOTE_RESTART" || command == "REMOTE_RESET" ||
            command == "ALL_START" || command == "ALL_STOP" || command == "ALL_RESTART" || command == "ALL_RESET" ||
            command == "LOCAL_PING" || command == "REMOTE_PING" ||
            command == "LOCAL_LED_ON" || command == "LOCAL_LED_OFF" ||
            command == "REMOTE_LED_ON" || command == "REMOTE_LED_OFF" ||
            command == "ALL_LED_ON" || command == "ALL_LED_OFF" ||
            command == "LOCAL_MOCK_ON" || command == "LOCAL_MOCK_OFF" ||
            command == "REMOTE_MOCK_ON" || command == "REMOTE_MOCK_OFF" ||
            command == "ALL_MOCK_ON" || command == "ALL_MOCK_OFF" ||
            command.startsWith("LOCAL_CAL_") || command.startsWith("REMOTE_CAL_")) {
            
            forward_command_to_rx_radio(command);
        }
        else if (command == "STATS") {
            show_detailed_stats();
        }
        else if (command == "RESET_STATS") {
            reset_all_stats();
        }
        else if (command == "HELP") {
            show_help();
        }
        else if (command == "BAT") {
            Serial.printf("[BLE_SLAVE] Battery: Local=%.2fV (%.1f%%), Remote=%.2fV (%.1f%%)\n",
                         battery_local_voltage, battery_local_soc, 
                         battery_remote_voltage, battery_remote_soc);
        } else {
            Serial.printf("[BLE_SLAVE] ✗ Unknown command: '%s'\n", command.c_str());
            Serial.println("[BLE_SLAVE] Type 'HELP' for available commands");
        }
    }
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

static String generate_serial_number() {
    // Get MAC address using Arduino ESP32 API (works without WiFi initialization)
    String macStr = WiFi.macAddress();
    // Convert from "AA:BB:CC:DD:EE:FF" format to "AABBCCDDEEFF"
    macStr.replace(":", "");
    macStr.toUpperCase();
    return macStr;
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    Serial.begin(921600);
    delay(100);
    
    command_response_mutex = xSemaphoreCreateMutex();
    if (command_response_mutex == NULL) {
        Serial.println("[BLE_SLAVE] ⚠ Failed to create mutex!");
    }
    
    Serial.println("\n[BLE_SLAVE] ESP32 BLE Slave - Load Cell Data Forwarder");
    Serial.println("[BLE_SLAVE] BLE Service: 12345678-1234-1234-1234-123456789abc");
    
    // Initialize UART2
    Serial2.begin(UART_BAUD_RATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    Serial2.setRxBufferSize(8192);
    Serial.printf("UART2 initialized at %d bps\n", UART_BAUD_RATE);
    
    // Initialize BLE
    BLEDevice::init("LoadCell_BLE_Server");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    
    BLEService *pService = pServer->createService(BLE_SERVICE_UUID);
    
    // Data characteristic - binary sensor data (NOTIFY)
    pDataCharacteristic = pService->createCharacteristic(
                        BLE_DATA_CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_READ |
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
    pDataCharacteristic->addDescriptor(new BLE2902());
    
    // Command characteristic - bidirectional (WRITE commands, NOTIFY responses)
    pCmdCharacteristic = pService->createCharacteristic(
                        BLE_CMD_CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_WRITE |
                        BLECharacteristic::PROPERTY_WRITE_NR |
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
    pCmdCharacteristic->addDescriptor(new BLE2902());
    pCmdCharacteristic->setCallbacks(new MyCommandCallbacks());

    pService->start();
    
    // ========== STANDARD BLE BATTERY SERVICE (0x180F) ==========
    // Single Battery Level showing minimum of both batteries (worst-case)
    // Custom characteristic below has full details (local+remote voltage+SOC)
    BLEService *pBatteryService = pServer->createService(BLEUUID((uint16_t)0x180F));
    
    pStdBatteryLevelChar = pBatteryService->createCharacteristic(
                        BLEUUID((uint16_t)0x2A19),
                        BLECharacteristic::PROPERTY_READ |
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
    pStdBatteryLevelChar->addDescriptor(new BLE2902());
    
    uint8_t initialLevel = 0;
    pStdBatteryLevelChar->setValue(&initialLevel, 1);
    
    pBatteryService->start();
    Serial.println("[BLE] Standard Battery Service started");
    // ===========================================================
    
    // ========== DEVICE INFORMATION SERVICE (0x180A) ==========
    BLEService *pDeviceInfoService = pServer->createService(BLEUUID((uint16_t)0x180A));
    
    // Manufacturer Name String (0x2A29)
    pManufacturerChar = pDeviceInfoService->createCharacteristic(
        BLEUUID((uint16_t)0x2A29), BLECharacteristic::PROPERTY_READ);
    pManufacturerChar->setValue(DEVICE_MANUFACTURER_NAME);
    
    // Model Number String (0x2A24)
    pModelNumberChar = pDeviceInfoService->createCharacteristic(
        BLEUUID((uint16_t)0x2A24), BLECharacteristic::PROPERTY_READ);
    pModelNumberChar->setValue(DEVICE_MODEL_NUMBER);
    
    // Serial Number String (0x2A25)
    pSerialNumberChar = pDeviceInfoService->createCharacteristic(
        BLEUUID((uint16_t)0x2A25), BLECharacteristic::PROPERTY_READ);
    String serial = generate_serial_number();
    pSerialNumberChar->setValue(serial.c_str());
    
    // Firmware Revision String (0x2A26)
    pFirmwareRevisionChar = pDeviceInfoService->createCharacteristic(
        BLEUUID((uint16_t)0x2A26), BLECharacteristic::PROPERTY_READ);
    pFirmwareRevisionChar->setValue(FIRMWARE_VERSION);
    
    // Hardware Revision String (0x2A27)
    pHardwareRevisionChar = pDeviceInfoService->createCharacteristic(
        BLEUUID((uint16_t)0x2A27), BLECharacteristic::PROPERTY_READ);
    pHardwareRevisionChar->setValue(HARDWARE_VERSION);
    
    // Software Revision String (0x2A28)
    pSoftwareRevisionChar = pDeviceInfoService->createCharacteristic(
        BLEUUID((uint16_t)0x2A28), BLECharacteristic::PROPERTY_READ);
    pSoftwareRevisionChar->setValue(SOFTWARE_VERSION);
    
    pDeviceInfoService->start();
    Serial.println("[BLE] Device Information Service started");
    Serial.printf("[BLE] Serial: %s\n", serial.c_str());
    // ===========================================================
    
    // ========== GENERIC ACCESS PROFILE SERVICE (0x1800) ==========
    // Add Appearance characteristic for device icon in scanners
    BLEService *pGAPService = pServer->createService(BLEUUID((uint16_t)0x1800));
    
    // Appearance (0x2A01) - Weight Scale (0x0B40)
    pAppearanceChar = pGAPService->createCharacteristic(
        BLEUUID((uint16_t)0x2A01), BLECharacteristic::PROPERTY_READ);
    uint16_t appearance = 0x0B40;  // Generic Weight Scale
    pAppearanceChar->setValue((uint8_t*)&appearance, 2);
    
    pGAPService->start();
    Serial.println("[BLE] Generic Access Profile Service started (Appearance: Weight Scale)");
    // ===========================================================
    
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
    pAdvertising->addServiceUUID(BLE_BATTERY_SERVICE_UUID);  // Advertise standard battery service
    pAdvertising->addServiceUUID(BLEUUID((uint16_t)0x180A));  // Device Information Service
    pAdvertising->setScanResponse(true);  // Enable scan response for battery service
    pAdvertising->setMinPreferred(0x06);  // Recommended for iOS
    BLEDevice::startAdvertising();
    
    Serial.println("[BLE_SLAVE] BLE server started, waiting for connections...");
    
    // Initialize hardware timer for 1000Hz sampling
    sampling_timer = timerBegin(1000000);
    timerAttachInterrupt(sampling_timer, &timer_sample_callback);
    timerAlarm(sampling_timer, 1000, true, 0);
    
    // Start tasks
    xTaskCreatePinnedToCore(uart_rx_task, "uart_rx", 8192, NULL, 23, NULL, 0);
    xTaskCreatePinnedToCore(timer_processing_task, "timer_proc", 4096, NULL, 24, NULL, 1);
    xTaskCreatePinnedToCore(ble_tx_task, "ble_tx", 4096, NULL, 22, NULL, 1);
    
    Serial.println("[BLE_SLAVE] ✓ System ready (type HELP for commands)");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    static unsigned long last_stats_time = 0;
    static unsigned long last_local_samples = 0;
    static unsigned long last_remote_samples = 0;
    static unsigned long last_ble_packets = 0;
    static unsigned long last_uart_packets = 0;
    static unsigned long last_samples_batched = 0;
    static unsigned long last_ble_status_send_ms = 0;
    
    unsigned long now = millis();
    
    // NOTE: Removed periodic BLE status sending - now request-based via BLE_STATUS command
    // Teensys can request BLE status when needed instead of receiving it every 5 seconds
    
    // Send battery BLE notifications every 30 seconds (reduced to save bandwidth)
    if (now - last_battery_ble_send_ms >= 30000) {
        last_battery_ble_send_ms = now;
        send_battery_ble_notification();
    }
    
    // Print live stats every 5 seconds
    if (now - last_stats_time >= 5000) {
        // Calculate rates (per second over 5 second window)
        float local_sps = (local_samples_received - last_local_samples) / 5.0f;
        float remote_sps = (remote_samples_received - last_remote_samples) / 5.0f;
        float ble_pps = (ble_packets_sent - last_ble_packets) / 5.0f;
        float uart_pps = (uart_packets_received - last_uart_packets) / 5.0f;
        float batch_sps = (samples_batched - last_samples_batched) / 5.0f;
        
        // Check data flow status
        unsigned long data_age = (last_data_received_ms > 0) ? (now - last_data_received_ms) : 999999;
        bool data_flowing = (data_age < DATA_TIMEOUT_MS);
        
        Serial.printf("📊 LIVE | L: %.0f | R: %.0f | UART: %.0f | BLE: %s %.0f (%.0f) | Q: L=%d R=%d | Flow: %s | BAT: L=%.1f%% R=%.1f%%\n",
                     local_sps, remote_sps, uart_pps,
                     deviceConnected ? "✓" : "✗", ble_pps, batch_sps,
                     data_store.local_count, data_store.remote_count,
                     data_flowing ? "ON" : "OFF",
                     battery_local_soc, battery_remote_soc);
        
        // Save current values for next calculation
        last_stats_time = now;
        last_local_samples = local_samples_received;
        last_remote_samples = remote_samples_received;
        last_ble_packets = ble_packets_sent;
        last_uart_packets = uart_packets_received;
        last_samples_batched = samples_batched;
    }
    
    // Handle BLE connection changes
    if (!deviceConnected && oldDeviceConnected) {
        delay(500);
        pServer->startAdvertising();
        Serial.println("Restarting BLE advertising");
        oldDeviceConnected = deviceConnected;
    }
    
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
    }
    
    handle_serial_commands();
    delay(100);
}
