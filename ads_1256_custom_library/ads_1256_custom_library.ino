#include <SPI.h>
#include <math.h>
#include "calibration_regression.h"

// Forward declaration for LED update function (used by calibration functions)
void update_led_status();

// Include ADS1256 constants first
// (These are defined in the ads1256_constants.ino tab)

// ADS1256 pin definitions (SPI0)
#define ADS_RST_PIN    8  //ADS1256 reset pin (same as Teensy 3.1)
#define ADS_RDY_PIN    22 //ADS1256 data ready (same as Teensy 3.1) 
#define ADS_CS_PIN     10 //ADS1256 chip select (Teensy 4.1 default SPI CS)

/* 
    Teensy 4.1 SPI0 pins (for ADS1256):
    CLK  - pin 13 (SCK)
    DIN  - pin 11 (MOSI)
    DOUT - pin 12 (MISO)
    CS   - pin 10 (default CS, but we define our own above)
*/

// SPI1 pins for ESP32 communication
#define PIN_SCK1   27
#define PIN_MOSI1  26
#define PIN_MISO1   1
#define PIN_CS1     0

// Serial3 pins for ESP32 command interface
// RX3 = Pin 15 (from ESP32 GPIO2) - Used for receiving commands
// TX is on Pin 16 (software serial) - PCB fixed wiring
#define SOFT_TX_PIN 16

// ============================================================================
// LED STATUS CONTROL (WS2812)
// ============================================================================

#define LED_EN1 31
#define LED_EN2 32
#define LED_EN3 33
#define LED_EN4 34

#define LED_DI1 37
#define LED_DI2 38
#define LED_DI3 39
#define LED_DI4 40

// Number of LEDs per data line (if LEDs are chained in series)
// Set to 1 if each pin controls a single LED
// Set to N if you have N LEDs chained in series on each pin
#define LEDS_PER_PIN 50  // Change this to match your actual LED count per pin

// LED state
enum LedStatus {
  LED_BOOTING = 0,
  LED_IDLE,
  LED_STARTING,
  LED_RUNNING,
  LED_STOPPING,
  LED_ERROR,
  // Calibration states (functional)
  LED_CAL_TARE_COLLECTING,
  LED_CAL_TARE_SUCCESS,
  LED_CAL_ADD_COLLECTING,
  LED_CAL_ADD_SUCCESS,
  LED_CAL_FIT_RUNNING,
  LED_CAL_FIT_SUCCESS,
  LED_CAL_CLEAR,
  LED_CAL_ERROR
};

static LedStatus current_led_status = LED_BOOTING;
static uint32_t led_last_update_ms = 0;
static uint32_t led_success_start_ms = 0;  // For success flash timing
static uint8_t led_brightness = 80;  // Default brightness (0-255)
static bool led_enabled = true;      // Master LED enable flag
static bool ble_connected = false;  // BLE connection status
static bool ble_connect_flash_active = false;  // Flash animation when BLE connects
static uint32_t ble_connect_flash_start_ms = 0;  // When BLE connect flash started
static uint8_t ble_connect_flash_count = 0;  // Number of flashes completed

// ============================================================================
// DATA ACQUISITION STATE MANAGEMENT
// ============================================================================

enum AcquisitionState {
  STATE_STOPPED = 0,      // Data acquisition stopped
  STATE_STARTING = 1,     // Initializing/starting up
  STATE_RUNNING = 2,      // Active data acquisition
  STATE_STOPPING = 3,     // Gracefully stopping
  STATE_ERROR = 4         // Error state
};

static AcquisitionState current_state = STATE_STOPPED;
static bool state_changed = true;  // Flag to report state changes

// ============================================================================
// FORWARD DECLARATIONS AND GLOBAL VARIABLES
// ============================================================================

// Frame structure constants
#define FRAME_BYTES 144
#define SAMPLES_PER_FRAME 10
#define CHANNELS 4
#define FS_HZ 4000.0  // 4kHz total (1kHz per channel)

// Performance mode selection
#define USE_CONTINUOUS_MODE 0  // Set to 1 for experimental continuous mode, 0 for optimized standard mode

struct __attribute__((packed)) Frame {
  uint8_t  sync[2];
  uint8_t  plate_id, proto_ver;
  uint16_t frame_idx;
  uint32_t t0_us;
  uint8_t  samples[120]; // 10 × (4 ch × int24)
  uint16_t crc16;
  uint8_t  pad[12];
};
static Frame tx;

//put the ADC constants here
double resolution = 8388608.; //2^23-1
//this needs to match the setting in the ADC init function in the library tab
double Gain = 64.; //be sure to have a period 
double vRef = 5.0; //reference voltage
//we'll calculate this in setup
double bitToVolt = 0.;

// Statistics structure
struct Stats {
  uint32_t start_ms=0, last_ms=0;
  uint32_t frames_sent=0;
  uint32_t min_xfer_us=UINT32_MAX, max_xfer_us=0;
} st;

// Sample buffer for 4 channels
int32_t sample_buffer[CHANNELS][SAMPLES_PER_FRAME];
uint8_t buffer_index = 0;
uint16_t frame_idx = 0;

// Load cell values
int32_t val1;  // Load cell 1 (AIN0-AIN1)
int32_t val2;  // Load cell 2 (AIN2-AIN3)
int32_t val3;  // Load cell 3 (AIN4-AIN5)
int32_t val4;  // Load cell 4 (AIN6-AIN7)

// Forward declarations for functions in other tabs
void initADS();
void SendCMD(uint8_t cmd);
int32_t read_single_channel_fast(uint8_t channel);

// Calibration regression v2 (in calibration_regression.cpp/.h)

// ADS1256 constants (in case they're not loaded yet from constants tab)
#ifndef SELFCAL
#define SELFCAL 0xF0  // Self Offset Calibration
#endif

// Ensure critical constants are available (Arduino IDE compilation order issue)
#ifndef PGA_64
#define PGA_64 0x27  // B00100111 - PGA = 64
#endif

#ifndef DR_30000
#define DR_30000 0xF0  // B11110000 - 30kSPS
#endif

// Forward declarations for noise filtering functions
struct FilteredReading {
  int32_t raw;
  int32_t filtered;
  bool outlier_detected;
};

// Filter types and structures (needed for Arduino IDE compilation order)
enum FilterType {
  FILTER_NONE = 0,
  FILTER_SMA = 1,        // Simple Moving Average
  FILTER_EMA = 2,        // Exponential Moving Average
  FILTER_MEDIAN = 3,     // Median Filter
  FILTER_KALMAN = 4,     // Kalman Filter
  FILTER_ADAPTIVE = 5,   // Adaptive Filter
  FILTER_COMBINED = 6,   // Combined multi-stage filter
  FILTER_GAUSSIAN = 7    // Gaussian Filter
};

enum OutlierMethod {
  OUTLIER_NONE = 0,
  OUTLIER_ZSCORE = 1,    // Z-score based
  OUTLIER_IQR = 2,       // Interquartile Range
  OUTLIER_MAD = 3,       // Median Absolute Deviation
  OUTLIER_ADAPTIVE = 4   // Adaptive threshold
};

struct FilterConfig {
  FilterType filter_type;
  OutlierMethod outlier_method;
  uint8_t window_size;           // Filter window size
  double outlier_threshold;      // Outlier detection threshold
  double ema_alpha;              // EMA smoothing factor (0-1)
  double gaussian_sigma;         // Gaussian filter standard deviation
  bool enable_preprocessing;     // Enable preprocessing stage
  bool enable_postprocessing;    // Enable postprocessing stage
};

// Circular buffer template
template<typename T, size_t N>
class CircularBuffer {
private:
  T buffer[N];
  size_t head = 0;
  size_t count = 0;
  
public:
  void push(T value) {
    buffer[head] = value;
    head = (head + 1) % N;
    if (count < N) count++;
  }
  
  T get(size_t index) const {
    if (index >= count) return T(0);
    size_t pos = (head - count + index) % N;
    return buffer[pos];
  }
  
  size_t size() const { return count; }
  bool full() const { return count == N; }
  
  T newest() const {
    if (count == 0) return T(0);
    return buffer[(head - 1 + N) % N];
  }
  
  T oldest() const {
    if (count == 0) return T(0);
    return buffer[(head - count + N) % N];
  }
  
  void clear() {
    head = 0;
    count = 0;
  }
};

// Filter state for each channel
struct ChannelFilter {
  CircularBuffer<int32_t, 20> raw_buffer;      // Raw data history
  CircularBuffer<double, 20> filtered_buffer;  // Filtered data history
  
  // EMA state
  double ema_value;
  bool ema_initialized;
  
  // Kalman filter state
  double kalman_x;      // State estimate
  double kalman_P;      // Error covariance
  double kalman_Q;      // Process noise
  double kalman_R;      // Measurement noise
  bool kalman_initialized;
  
  // Gaussian filter state
  double gaussian_weights[11];  // Pre-computed Gaussian weights (up to kernel size 11)
  uint8_t gaussian_kernel_size; // Current kernel size
  bool gaussian_initialized;
  
  // Statistics for outlier detection
  double running_mean;
  double running_variance;
  double running_mad;   // Median Absolute Deviation
  uint32_t sample_count;
  
  // Adaptive thresholds
  double adaptive_threshold_high;
  double adaptive_threshold_low;
  double noise_level;
  
  // Constructor
  ChannelFilter() {
    reset();
  }
  
  void reset() {
    raw_buffer.clear();
    filtered_buffer.clear();
    ema_value = 0.0;
    ema_initialized = false;
    kalman_x = 0.0;
    kalman_P = 1.0;
    kalman_Q = 0.01;
    kalman_R = 0.1;
    kalman_initialized = false;
    gaussian_initialized = false;
    running_mean = 0.0;
    running_variance = 0.0;
    running_mad = 0.0;
    sample_count = 0;
    adaptive_threshold_high = 100000.0;
    adaptive_threshold_low = -100000.0;
    noise_level = 0.0;
  }
};

// Forward declarations for noise filtering functions (defined in noise_filtering.ino)
// Note: channel_filters[] is defined in noise_filtering.ino
int32_t get_filtered_load_cell_reading(uint8_t channel);     // With tare offset + polarity (delta counts)
int32_t get_raw_filtered_reading(uint8_t channel);           // Raw filtered, NO tare/polarity (for calibration)
int32_t get_calibrated_load_cell_reading(uint8_t channel);   // Calibrated weight in 10g units (for streaming)
FilteredReading get_filtered_reading_with_info(uint8_t channel);
void enable_filtering(bool enable);
void reset_filters();
void set_realtime_filtering();
void set_high_quality_filtering();
void set_low_noise_filtering();
void set_gaussian_filtering();
void set_filter_type(int type);
void set_outlier_method(int method);
void set_gaussian_sigma(double sigma);
void show_filter_status();
void set_tare_offset_enabled(bool enable);
bool is_tare_offset_enabled();

// ============================================================================
// SOFTWARE SERIAL TX ON PIN 16 (PCB has TX wired to Pin 16, not Pin 14)
// ============================================================================

#define SOFT_TX_BAUD 9600
#define SOFT_TX_BIT_TIME_US (1000000 / SOFT_TX_BAUD)  // ~104 µs per bit (much more reliable)

void softSerialTX_init() {
    pinMode(SOFT_TX_PIN, OUTPUT);
    digitalWriteFast(SOFT_TX_PIN, HIGH);  // Idle high (UART idle state)
}

void softSerialTX_writeByte(uint8_t b) {
    // Disable interrupts for precise timing
    noInterrupts();
    
    // Start bit (LOW)
    digitalWriteFast(SOFT_TX_PIN, LOW);
    delayMicroseconds(SOFT_TX_BIT_TIME_US);
    
    // Data bits (LSB first)
    for (int i = 0; i < 8; i++) {
        digitalWriteFast(SOFT_TX_PIN, (b >> i) & 0x01);
        delayMicroseconds(SOFT_TX_BIT_TIME_US);
    }
    
    // Stop bit (HIGH)
    digitalWriteFast(SOFT_TX_PIN, HIGH);
    delayMicroseconds(SOFT_TX_BIT_TIME_US);
    
    interrupts();
}

void softSerialTX_print(const char* str) {
    while (*str) {
        softSerialTX_writeByte(*str++);
    }
}

void softSerialTX_println(const char* str) {
    softSerialTX_print(str);
    softSerialTX_writeByte('\n');
}

void softSerialTX_printf(const char* format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    softSerialTX_print(buffer);
}

// ============================================================================
// LED CONTROL FUNCTIONS (WS2812)
// ============================================================================

// DWT cycle counter (Teensy 4.1)
static inline void led_dwt_init() {
  *(volatile uint32_t *)0xE000EDFC |= 0x01000000; // DEMCR TRCENA
  *(volatile uint32_t *)0xE0001004 = 0;          // CYCCNT
  *(volatile uint32_t *)0xE0001000 |= 1;         // enable CYCCNT
}

static inline uint32_t led_dwt_cycles() { 
  return *(volatile uint32_t *)0xE0001004; 
}

static inline void led_wait_cycles(uint32_t c) {
  uint32_t s = led_dwt_cycles();
  while ((uint32_t)(led_dwt_cycles() - s) < c) {}
}

// WS2812 800kHz timings
static inline void led_ws_send_byte(uint8_t pin, uint8_t b) {
  const uint32_t F = F_CPU;
  const uint32_t T0H = (uint32_t)(F * 0.35e-6);
  const uint32_t T1H = (uint32_t)(F * 0.70e-6);
  const uint32_t TT  = (uint32_t)(F * 1.25e-6);

  for (int i = 7; i >= 0; --i) {
    bool one = b & (1 << i);
    uint32_t start = led_dwt_cycles();
    digitalWriteFast(pin, HIGH);
    led_wait_cycles(one ? T1H : T0H);
    digitalWriteFast(pin, LOW);
    while ((uint32_t)(led_dwt_cycles() - start) < TT) {}
  }
}

static inline uint8_t led_scale8(uint8_t v, uint8_t br) {
  return (uint16_t)v * (uint16_t)br / 255;
}

static inline void led_ws_show_1pixel(uint8_t pin, uint8_t r, uint8_t g, uint8_t b, uint8_t br) {
  r = led_scale8(r, br);
  g = led_scale8(g, br);
  b = led_scale8(b, br);

  noInterrupts();
  // WS2812 order: GRB
  led_ws_send_byte(pin, g);
  led_ws_send_byte(pin, r);
  led_ws_send_byte(pin, b);
  interrupts();

  delayMicroseconds(80); // latch
}

// Send data to multiple LEDs in series on one pin (for chained WS2812)
static inline void led_ws_show_npixels(uint8_t pin, uint8_t r, uint8_t g, uint8_t b, uint8_t br, uint8_t num_leds) {
  r = led_scale8(r, br);
  g = led_scale8(g, br);
  b = led_scale8(b, br);

  noInterrupts();
  // Send data for all LEDs in the chain (no latch delay between them)
  for (uint8_t i = 0; i < num_leds; i++) {
    // WS2812 order: GRB
    led_ws_send_byte(pin, g);
    led_ws_send_byte(pin, r);
    led_ws_send_byte(pin, b);
  }
  interrupts();

  // Single latch delay at the end (after all pixels)
  delayMicroseconds(80);
}

static inline void led_allEN(bool on) {
  digitalWriteFast(LED_EN1, on);
  digitalWriteFast(LED_EN2, on);
  digitalWriteFast(LED_EN3, on);
  digitalWriteFast(LED_EN4, on);
}

static inline void led_allDI_low() {
  digitalWriteFast(LED_DI1, LOW);
  digitalWriteFast(LED_DI2, LOW);
  digitalWriteFast(LED_DI3, LOW);
  digitalWriteFast(LED_DI4, LOW);
}

static inline void led_setAll(uint8_t r, uint8_t g, uint8_t b, uint8_t br) {
  // Use multi-pixel function if LEDs are chained, otherwise single pixel
  if (LEDS_PER_PIN > 1) {
    // LEDs are chained in series - send data for all LEDs on each pin
    led_ws_show_npixels(LED_DI1, r, g, b, br, LEDS_PER_PIN);
    led_ws_show_npixels(LED_DI2, r, g, b, br, LEDS_PER_PIN);
    led_ws_show_npixels(LED_DI3, r, g, b, br, LEDS_PER_PIN);
    led_ws_show_npixels(LED_DI4, r, g, b, br, LEDS_PER_PIN);
  } else {
    // Single LED per pin
    led_ws_show_1pixel(LED_DI1, r, g, b, br);
    led_ws_show_1pixel(LED_DI2, r, g, b, br);
    led_ws_show_1pixel(LED_DI3, r, g, b, br);
    led_ws_show_1pixel(LED_DI4, r, g, b, br);
  }
}

__attribute__((used)) void update_led_status() {  // Made non-static so calibration functions can call it
  if (!led_enabled) {
    led_setAll(0, 0, 0, 0);
    return;
  }
  
  uint32_t now = millis();
  
  // Check if mock data is enabled (for color inversion)
  extern bool mock_data_is_enabled();
  bool mock_mode = mock_data_is_enabled();
  
  switch (current_led_status) {
    case LED_BOOTING:
      // Yellow solid during boot
      led_setAll(255, 200, 0, led_brightness);
      break;
      
    case LED_IDLE:
      if (ble_connected) {
        // Handle BLE connect flash animation (2 flashes, then solid)
        if (ble_connect_flash_active) {
          uint32_t flash_elapsed = now - ble_connect_flash_start_ms;
          
          // Flash pattern: 200ms ON, 200ms OFF, 200ms ON, 200ms OFF = 800ms total
          // Flash 1: 0-200ms ON, 200-400ms OFF
          // Flash 2: 400-600ms ON, 600-800ms OFF
          // After 800ms: solid
          
          if (flash_elapsed < 800) {
            uint32_t cycle_pos = flash_elapsed % 400;  // Position within each 400ms cycle
            
            if (cycle_pos < 200) {
              // Flash ON (first 200ms of each cycle)
              if (mock_mode) {
                led_setAll(255, 200, 0, led_brightness);  // Yellow
              } else {
                led_setAll(167, 36, 104, led_brightness);  // Pink/Magenta #A72468
              }
            } else {
              // Flash OFF (last 200ms of each cycle)
              led_setAll(0, 0, 0, 0);
            }
          } else {
            // Flash animation complete, go to solid
            ble_connect_flash_active = false;
            ble_connect_flash_count = 2;
            
            // Fall through to solid color
            if (mock_mode) {
              led_setAll(255, 200, 0, led_brightness);  // Yellow solid
            } else {
              led_setAll(167, 36, 104, led_brightness);  // Pink/Magenta solid #A72468
            }
          }
        } else {
          // Solid color when BLE is connected (after flash animation)
          if (mock_mode) {
            // Yellow solid - Mock data mode, BLE connected
            led_setAll(255, 200, 0, led_brightness);
          } else {
            // Pink/Magenta solid - Real data mode, BLE connected
            led_setAll(167, 36, 104, led_brightness);  // #A72468
          }
        }
      } else {
        // Breathing effect when BLE is not connected
        if (mock_mode) {
          // Yellow breathing effect (2 second cycle) - Mock data mode, BLE disconnected
          {
            float phase = (now % 2000) / 2000.0f * 2.0f * 3.14159f;
            float sine_val = (sin(phase) + 1.0f) / 2.0f;  // 0 to 1
            uint8_t br = (uint8_t)(led_brightness * (0.1f + 0.9f * sine_val));
            led_setAll(255, 200, 0, br);  // Yellow breathing
          }
        } else {
          // Pink/Magenta breathing effect (2 second cycle) - Real data mode, BLE disconnected
          {
            float phase = (now % 2000) / 2000.0f * 2.0f * 3.14159f;
            float sine_val = (sin(phase) + 1.0f) / 2.0f;  // 0 to 1
            uint8_t br = (uint8_t)(led_brightness * (0.1f + 0.9f * sine_val));
            led_setAll(167, 36, 104, br);  // #A72468 - Pink/Magenta breathing
          }
        }
      }
      break;
      
    case LED_STARTING:
      if (mock_mode) {
        // Orange slow pulse (500ms cycle) - Mock data mode
        {
          uint8_t phase = (now / 250) % 2;
          uint8_t br = phase ? led_brightness : (led_brightness / 3);
          led_setAll(255, 165, 0, br);  // Orange pulse
        }
      } else {
        // Dark blue slow pulse (500ms cycle) - Real data mode
        {
          uint8_t phase = (now / 250) % 2;
          uint8_t br = phase ? led_brightness : (led_brightness / 3);
          led_setAll(6, 28, 47, br);  // #061C2F - Dark blue
        }
      }
      break;
      
    case LED_RUNNING:
      if (mock_mode) {
        // Orange solid - Mock data mode
        led_setAll(255, 165, 0, led_brightness);  // Orange solid
      } else {
        // Dark blue solid - Real data mode
        led_setAll(6, 28, 47, led_brightness);  // #061C2F - Dark blue solid
      }
      break;
      
    case LED_STOPPING:
      if (mock_mode) {
        // Cyan fast blink (200ms cycle) - Mock data mode (inverted from orange)
        {
          uint8_t phase = (now / 100) % 2;
          led_setAll(0, 255, 255, phase ? led_brightness : 0);  // Cyan
        }
      } else {
        // Orange fast blink (200ms cycle) - Real data mode
        {
          uint8_t phase = (now / 100) % 2;
          led_setAll(255, 128, 0, phase ? led_brightness : 0);
        }
      }
      break;
      
    case LED_ERROR:
      // Red fast flash (150ms cycle) - Same for both modes
      {
        uint8_t phase = (now / 75) % 2;
        led_setAll(255, 0, 0, phase ? led_brightness : 0);
      }
      break;
      
    // Calibration states (functional) - rearranged with relatable colors
    case LED_CAL_TARE_COLLECTING:
      if (mock_mode) {
        // Magenta solid - Mock data mode (inverted from green)
        led_setAll(255, 0, 255, led_brightness);  // Magenta solid
      } else {
        // Green solid - Real data mode
        led_setAll(0, 255, 0, led_brightness);  // Green solid
      }
      break;
      
    case LED_CAL_TARE_SUCCESS:
      if (mock_mode) {
        // Magenta double flash (500ms total) - Mock data mode
        {
          uint32_t elapsed = now - led_success_start_ms;
          if (elapsed < 500) {
            uint8_t flash = (elapsed / 125) % 2;
            led_setAll(255, 0, 255, flash ? led_brightness : 0);  // Magenta
          } else {
            current_led_status = LED_IDLE;  // Return to idle after flash
          }
        }
      } else {
        // Green double flash (500ms total) - Real data mode
        {
          uint32_t elapsed = now - led_success_start_ms;
          if (elapsed < 500) {
            uint8_t flash = (elapsed / 125) % 2;
            led_setAll(0, 255, 0, flash ? led_brightness : 0);  // Green
          } else {
            current_led_status = LED_IDLE;  // Return to idle after flash
          }
        }
      }
      break;
      
    case LED_CAL_ADD_COLLECTING:
      if (mock_mode) {
        // Magenta solid - Mock data mode (inverted from green)
        led_setAll(255, 0, 255, led_brightness);  // Magenta solid
      } else {
        // Green solid - Real data mode
        led_setAll(0, 255, 0, led_brightness);  // Green solid
      }
      break;
      
    case LED_CAL_ADD_SUCCESS:
      if (mock_mode) {
        // Magenta triple flash (300ms each, 900ms total) - Mock data mode
        {
          uint32_t elapsed = now - led_success_start_ms;
          if (elapsed < 900) {
            uint8_t flash = (elapsed / 150) % 2;
            led_setAll(255, 0, 255, flash ? led_brightness : 0);  // Magenta
          } else {
            current_led_status = LED_IDLE;  // Return to idle after flash
          }
        }
      } else {
        // Green triple flash (300ms each, 900ms total) - Real data mode
        {
          uint32_t elapsed = now - led_success_start_ms;
          if (elapsed < 900) {
            uint8_t flash = (elapsed / 150) % 2;
            led_setAll(0, 255, 0, flash ? led_brightness : 0);  // Green
          } else {
            current_led_status = LED_IDLE;  // Return to idle after flash
          }
        }
      }
      break;
      
    case LED_CAL_FIT_RUNNING:
      if (mock_mode) {
        // Magenta solid - Mock data mode (inverted from green)
        led_setAll(255, 0, 255, led_brightness);  // Magenta solid
      } else {
        // Green solid - Real data mode
        led_setAll(0, 255, 0, led_brightness);  // Green solid
      }
      break;
      
    case LED_CAL_FIT_SUCCESS:
      if (mock_mode) {
        // Magenta single long flash (800ms) - Mock data mode
        {
          uint32_t elapsed = now - led_success_start_ms;
          if (elapsed < 800) {
            led_setAll(255, 0, 255, led_brightness);  // Magenta
          } else {
            current_led_status = LED_IDLE;  // Return to idle after flash
          }
        }
      } else {
        // Green single long flash (800ms) - Real data mode
        {
          uint32_t elapsed = now - led_success_start_ms;
          if (elapsed < 800) {
            led_setAll(0, 255, 0, led_brightness);  // Green
          } else {
            current_led_status = LED_IDLE;  // Return to idle after flash
          }
        }
      }
      break;
      
    case LED_CAL_CLEAR:
      if (mock_mode) {
        // Blue quick flash (200ms) - Mock data mode (inverted from orange)
        {
          uint32_t elapsed = now - led_success_start_ms;
          if (elapsed < 200) {
            led_setAll(0, 100, 255, led_brightness);  // Blue
          } else {
            current_led_status = LED_IDLE;  // Return to idle after flash
          }
        }
      } else {
        // Orange/Yellow quick flash (200ms) - Real data mode
        {
          uint32_t elapsed = now - led_success_start_ms;
          if (elapsed < 200) {
            led_setAll(255, 165, 0, led_brightness);  // Orange
          } else {
            current_led_status = LED_IDLE;  // Return to idle after flash
          }
        }
      }
      break;
      
    case LED_CAL_ERROR:
      // Red fast flash (100ms cycle)
      {
        uint8_t phase = (now / 50) % 2;
        led_setAll(255, 0, 0, phase ? led_brightness : 0);
      }
      break;
  }
  
  led_last_update_ms = now;
}

// ============================================================================
// STATE MANAGEMENT AND DATA ACQUISITION CONTROL
// ============================================================================

const char* get_state_name() {
  switch (current_state) {
    case STATE_STOPPED:  return "STOPPED";
    case STATE_STARTING: return "STARTING";
    case STATE_RUNNING:  return "RUNNING";
    case STATE_STOPPING: return "STOPPING";
    case STATE_ERROR:    return "ERROR";
    default:             return "UNKNOWN";
  }
}

void send_status_response(const char* cmd, const char* status) {
  char buf[64];
  snprintf(buf, sizeof(buf), "%s:%s", cmd, status);
  softSerialTX_println(buf);
  Serial.printf("[T41] -> ESP32: %s\n", buf);
}

void send_state_update() {
  char buf[64];
  snprintf(buf, sizeof(buf), "STATE:%s", get_state_name());
  softSerialTX_println(buf);
  Serial.printf("[T41] State update: %s\n", get_state_name());
  state_changed = false;
}

bool start_data_acquisition() {
  if (current_state == STATE_RUNNING) {
    Serial.println("[T41] Already running");
    return true;
  }
  
  Serial.println("[T41] Starting data acquisition...");
  current_state = STATE_STARTING;
  current_led_status = LED_STARTING;
  state_changed = true;

  // Reset statistics
  st.start_ms = millis();
  st.last_ms = st.start_ms;
  st.frames_sent = 0;
  st.min_xfer_us = UINT32_MAX;
  st.max_xfer_us = 0;

  // Reset buffer
  buffer_index = 0;
  frame_idx = 0;

  current_state = STATE_RUNNING;
  current_led_status = LED_RUNNING;
  state_changed = true;
  update_led_status();  // Update LED before entering RUNNING (no updates during run)

  Serial.println("[T41] Data acquisition started");
  return true;
}

bool stop_data_acquisition() {
  if (current_state == STATE_STOPPED) {
    Serial.println("[T41] Already stopped");
    return true;
  }
  
  Serial.println("[T41] Stopping data acquisition...");
  current_state = STATE_STOPPING;
  current_led_status = LED_STOPPING;
  state_changed = true;

  // Print final stats
  print_statistics();

  current_state = STATE_STOPPED;
  current_led_status = LED_IDLE;
  state_changed = true;
  update_led_status();  // Update LED when stopped

  Serial.println("[T41] Data acquisition stopped");
  return true;
}

bool restart_data_acquisition() {
  Serial.println("[T41] Restarting data acquisition...");
  stop_data_acquisition();
  delay(100);
  return start_data_acquisition();
}

// ============================================================================
// COMMAND INTERFACE FUNCTIONS
// ============================================================================

// Send compact single-line calibration response to ESP32
// Format: "CMD:data" - one line, easy to parse, no loss
static void cal_response(const char* response) {
  Serial.printf("[CAL] -> %s\n", response);  // Debug to Serial Monitor
  softSerialTX_println(response);  // Send to ESP32
}

void handle_esp32_commands() {
  if (Serial3.available()) {
    String command = Serial3.readStringUntil('\n');
    command.trim();
    command.toUpperCase();
    
    Serial.printf("[T41] ESP32 Command: %s\n", command.c_str());
    
    // BLE connection status commands
    if (command == "BLE_CONNECTED") {
      if (!ble_connected) {  // Only flash if transitioning from disconnected to connected
        ble_connected = true;
        // Start flash animation (2 flashes, then solid)
        ble_connect_flash_active = true;
        ble_connect_flash_start_ms = millis();
        ble_connect_flash_count = 0;
        Serial.println("[T41] ✓ BLE connected - LED: flashing then solid");
      } else {
        // Already connected, just update status (no flash)
        ble_connected = true;
        Serial.println("[T41] ✓ BLE already connected - LED: solid");
      }
      update_led_status();  // Update LED immediately
      send_status_response("BLE_CONNECTED", "OK");
    }
    else if (command == "BLE_DISCONNECTED") {
      ble_connected = false;
      ble_connect_flash_active = false;  // Cancel any flash animation
      Serial.println("[T41] ✗ BLE disconnected - LED: breathing");
      update_led_status();  // Update LED immediately
      send_status_response("BLE_DISCONNECTED", "OK");
    }
    else if (command == "START") {
      if (start_data_acquisition()) {
        send_status_response("START", "OK");
      } else {
        send_status_response("START", "ERROR");
      }
    }
    else if (command == "STOP") {
      if (stop_data_acquisition()) {
        send_status_response("STOP", "OK");
      } else {
        send_status_response("STOP", "ERROR");
      }
    }
    else if (command == "RESTART") {
      if (restart_data_acquisition()) {
        send_status_response("RESTART", "OK");
      } else {
        send_status_response("RESTART", "ERROR");
      }
    }
    else if (command == "RESET") {
      send_status_response("RESET", "OK");
      delay(500);
      // Software reset
      SCB_AIRCR = 0x05FA0004; // Teensy software reset
    }
    else if (command == "STATUS") {
      const char* state_names_arr[] = {"STOPPED", "STARTING", "RUNNING", "STOPPING", "ERROR"};
      char buf[128];
      snprintf(buf, sizeof(buf), "STATUS:state=%s,led=%s,frames=%lu",
               state_names_arr[current_state], 
               led_enabled ? "ON" : "OFF",
               (unsigned long)st.frames_sent);
      softSerialTX_println(buf);
      send_status_response("STATUS", "OK");
    }
    // ========== LED CONTROL COMMANDS ==========
    else if (command == "LED_ON") {
      led_enabled = true;
      update_led_status();
      send_status_response("LED_ON", "OK");
    }
    else if (command == "LED_OFF") {
      led_enabled = false;
      led_setAll(0, 0, 0, 0);
      send_status_response("LED_OFF", "OK");
    }
    // ========== MOCK DATA COMMANDS ==========
    else if (command == "MOCK_ON") {
      extern void mock_data_set_enabled(bool);
      mock_data_set_enabled(true);
      send_status_response("MOCK_ON", "OK");
    }
    else if (command == "MOCK_OFF") {
      extern void mock_data_set_enabled(bool);
      mock_data_set_enabled(false);
      send_status_response("MOCK_OFF", "OK");
    }
    // ========== NOISE FILTERING COMMANDS ==========
    else if (command == "FILTER_ENABLE") {
      enable_filtering(true);
      send_status_response("FILTER_ENABLE", "OK");
    }
    else if (command == "FILTER_DISABLE") {
      enable_filtering(false);
      send_status_response("FILTER_DISABLE", "OK");
    }
    else if (command == "FILTER_STATUS") {
      show_filter_status();
      send_status_response("FILTER_STATUS", "OK");
    }
    else if (command == "FILTER_RESET") {
      reset_filters();
      send_status_response("FILTER_RESET", "OK");
    }
    else if (command == "FILTER_REALTIME") {
      set_realtime_filtering();
      send_status_response("FILTER_REALTIME", "OK");
    }
    else if (command == "FILTER_HIGH_QUALITY") {
      set_high_quality_filtering();
      send_status_response("FILTER_HIGH_QUALITY", "OK");
    }
    else if (command == "FILTER_LOW_NOISE") {
      set_low_noise_filtering();
      send_status_response("FILTER_LOW_NOISE", "OK");
    }
    // ========== TARE OFFSET CONTROL ==========
    else if (command == "TARE_OFFSET_ON") {
      set_tare_offset_enabled(true);
      send_status_response("TARE_OFFSET_ON", "OK");
    }
    else if (command == "TARE_OFFSET_OFF") {
      set_tare_offset_enabled(false);
      send_status_response("TARE_OFFSET_OFF", "OK");
    }
    else if (command == "PING") {
      // Simple ping-pong test for communication chain verification
      Serial.println("[T41] PING received, sending PONG via Software TX (Pin 16)...");
      Serial.printf("[T41] Software TX config: Pin=%d, Baud=%d, BitTime=%d us\n", 
                    SOFT_TX_PIN, SOFT_TX_BAUD, SOFT_TX_BIT_TIME_US);
      
      // Verify pin is configured
      pinMode(SOFT_TX_PIN, OUTPUT);
      
      // Send PONG using software serial TX on Pin 16
      softSerialTX_println("PONG");
      
      Serial.println("[T41] PONG sent on Pin 16 at 9600 baud");
    }
    else if (command == "TX_TEST") {
      // Continuous TX test - send data every 100ms for 5 seconds
      Serial.println("[T41] Starting Software TX test on Pin 16 (5 seconds)...");
      char buf[20];
      for (int i = 0; i < 50; i++) {
        sprintf(buf, "TX_TEST_%d", i);
        softSerialTX_println(buf);
        Serial.printf("[T41] Sent: %s\n", buf);
        delay(100);
      }
      Serial.println("[T41] TX test complete");
    }
    // ========== CALIBRATION COMMANDS (ESP32) - COMPACT SINGLE-LINE RESPONSES ==========
    else if (command == "CAL_SHOW") {
      // Format: SHOW:LC1=off,a,n;LC2=off,a,n;LC3=off,a,n;LC4=off,a,n
      CalStatus status;
      cal_get_status(&status);
      char buf[200];
      snprintf(buf, sizeof(buf), "SHOW:LC1=%ld,%.6f,%u;LC2=%ld,%.6f,%u;LC3=%ld,%.6f,%u;LC4=%ld,%.6f,%u",
               (long)status.offsets[0], (double)status.ch_a_10g_per_count[0], (unsigned)status.points_ch_n[0],
               (long)status.offsets[1], (double)status.ch_a_10g_per_count[1], (unsigned)status.points_ch_n[1],
               (long)status.offsets[2], (double)status.ch_a_10g_per_count[2], (unsigned)status.points_ch_n[2],
               (long)status.offsets[3], (double)status.ch_a_10g_per_count[3], (unsigned)status.points_ch_n[3]);
      cal_response(buf);
    }
    else if (command == "CAL_READ") {
      // Format: READ:LC1=val;LC2=val;LC3=val;LC4=val;TOT=val
      char buf[100];
      snprintf(buf, sizeof(buf), "READ:%ld;%ld;%ld;%ld;%ld",
               (long)cal_read_cell_10g_units(0, true),
               (long)cal_read_cell_10g_units(1, true),
               (long)cal_read_cell_10g_units(2, true),
               (long)cal_read_cell_10g_units(3, true),
               (long)cal_read_total_10g_units(true));
      cal_response(buf);
    }
    else if (command == "CAL_TARE") {
      Serial.println("[CAL] TARE: collecting samples...");
      current_led_status = LED_CAL_TARE_COLLECTING;
      update_led_status();  // Update LED immediately to show solid color
      bool ok = cal_tare(1500, true);  // 1.5 seconds collection time
      if (ok) {
        current_led_status = LED_CAL_TARE_SUCCESS;
        led_success_start_ms = millis();
      } else {
        current_led_status = LED_CAL_ERROR;
      }
      cal_response(ok ? "TARE:OK" : "TARE:ERR");
    }
    else if (command.startsWith("CAL_ADD_") && !command.startsWith("CAL_ADD_CH_")) {
      // Format: CAL_ADD_<kg>
      float kg = command.substring(8).toFloat();
      if (kg > 0.0f) {
        Serial.printf("[CAL] ADD %.3f kg: collecting...\n", (double)kg);
        current_led_status = LED_CAL_ADD_COLLECTING;
        update_led_status();  // Update LED immediately to show solid color
        bool ok = cal_add_point_total(kg, 1500, true);  // 1.5 seconds collection time
        if (ok) {
          current_led_status = LED_CAL_FIT_RUNNING;
          bool fit_ok = cal_fit_and_save();
          if (fit_ok) {
            current_led_status = LED_CAL_ADD_SUCCESS;
            led_success_start_ms = millis();
          } else {
            current_led_status = LED_CAL_ERROR;
          }
          cal_response(fit_ok ? "ADD:OK" : "ADD:FIT_ERR");
        } else {
          current_led_status = LED_CAL_ERROR;
          cal_response("ADD:ERR");
        }
      } else {
        cal_response("ADD:BAD_KG");
      }
    }
    else if (command.startsWith("CAL_ADD_CH_")) {
      // Format: CAL_ADD_CH_<ch>_<weight>
      // Parse: CAL_ADD_CH_1_20 -> ch=1, kg=20
      String params = command.substring(11);  // "1_20"
      int us = params.indexOf('_');
      if (us > 0) {
        int ch = params.substring(0, us).toInt();
        float kg = params.substring(us + 1).toFloat();
        if (ch >= 1 && ch <= 4 && kg > 0.0f) {
          Serial.printf("[CAL] ADD_CH%d %.3f kg: collecting...\n", ch, (double)kg);
          current_led_status = LED_CAL_ADD_COLLECTING;
          update_led_status();  // Update LED immediately to show solid color
          bool ok = cal_add_point_channel((uint8_t)(ch - 1), kg, 1500, true);  // 1.5 seconds collection time
          if (ok) {
            current_led_status = LED_CAL_FIT_RUNNING;
            bool fit_ok = cal_fit_and_save();
            if (fit_ok) {
              current_led_status = LED_CAL_ADD_SUCCESS;
              led_success_start_ms = millis();
            } else {
              current_led_status = LED_CAL_ERROR;
            }
            cal_response(fit_ok ? "ADD_CH:OK" : "ADD_CH:FIT_ERR");
          } else {
            current_led_status = LED_CAL_ERROR;
            cal_response("ADD_CH:ERR");
          }
        } else {
          cal_response("ADD_CH:BAD_ARGS");
        }
      } else {
        cal_response("ADD_CH:BAD_FMT");
      }
    }
    else if (command == "CAL_FIT") {
      current_led_status = LED_CAL_FIT_RUNNING;
      bool ok = cal_fit_and_save();
      if (ok) {
        current_led_status = LED_CAL_FIT_SUCCESS;
        led_success_start_ms = millis();
      } else {
        current_led_status = LED_CAL_ERROR;
      }
      cal_response(ok ? "FIT:OK" : "FIT:ERR");
    }
    else if (command == "CAL_CLEAR") {
      bool ok = cal_clear();
      if (ok) {
        current_led_status = LED_CAL_CLEAR;
        led_success_start_ms = millis();
      } else {
        current_led_status = LED_CAL_ERROR;
      }
      cal_response(ok ? "CLEAR:OK" : "CLEAR:ERR");
    }
    else if (command == "CAL_POINTS") {
      // Format: PTS:LC1=n;LC2=n;LC3=n;LC4=n
      CalStatus status;
      cal_get_status(&status);
      char buf[50];
      snprintf(buf, sizeof(buf), "PTS:%u;%u;%u;%u",
               (unsigned)status.points_ch_n[0], (unsigned)status.points_ch_n[1],
               (unsigned)status.points_ch_n[2], (unsigned)status.points_ch_n[3]);
      cal_response(buf);
    }
    else if (command == "CAL_READ_RAW") {
      // Same as CAL_READ but with raw (unfiltered) values
      char buf[100];
      snprintf(buf, sizeof(buf), "READ_RAW:%ld;%ld;%ld;%ld;%ld",
               (long)cal_read_cell_10g_units(0, false),
               (long)cal_read_cell_10g_units(1, false),
               (long)cal_read_cell_10g_units(2, false),
               (long)cal_read_cell_10g_units(3, false),
               (long)cal_read_total_10g_units(false));
      cal_response(buf);
    }
    else {
      send_status_response(command.c_str(), "UNKNOWN");
    }
  }
}

void handle_serial_monitor_commands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toUpperCase();
    
    Serial.printf("[T41] Serial Command: %s\n", command.c_str());
    
    if (command == "START") {
      if (start_data_acquisition()) {
        Serial.println("[T41] ✓ Data acquisition started");
      } else {
        Serial.println("[T41] ✗ Failed to start");
      }
    }
    else if (command == "STOP") {
      if (stop_data_acquisition()) {
        Serial.println("[T41] ✓ Data acquisition stopped");
      } else {
        Serial.println("[T41] ✗ Failed to stop");
      }
    }
    else if (command == "RESTART") {
      if (restart_data_acquisition()) {
        Serial.println("[T41] ✓ Data acquisition restarted");
      } else {
        Serial.println("[T41] ✗ Failed to restart");
      }
    }
    else if (command == "RESET") {
      Serial.println("[T41] Resetting in 1 second...");
      delay(1000);
      SCB_AIRCR = 0x05FA0004; // Teensy software reset
    }
    else if (command == "STATUS") {
      const char* state_names[] = {"STOPPED", "STARTING", "RUNNING", "STOPPING", "ERROR"};
      Serial.printf("[T41] State: %s | Frames: %lu | Uptime: %lu ms\n", 
                    state_names[current_state], (unsigned long)st.frames_sent, millis());
    }
    // ========== LED CONTROL COMMANDS ==========
    else if (command == "LED_ON") {
      led_enabled = true;
      update_led_status();
      Serial.println("[T41] ✓ LEDs enabled");
    }
    else if (command == "LED_OFF") {
      led_enabled = false;
      led_setAll(0, 0, 0, 0);
      Serial.println("[T41] ✓ LEDs disabled");
    }
    // ========== MOCK DATA COMMANDS ==========
    else if (command == "MOCK_ON") {
      extern void mock_data_set_enabled(bool);
      mock_data_set_enabled(true);
      Serial.println("[T41] ✓ Mock data generation enabled");
    }
    else if (command == "MOCK_OFF") {
      extern void mock_data_set_enabled(bool);
      mock_data_set_enabled(false);
      Serial.println("[T41] ✓ Mock data generation disabled");
    }
    // ========== NOISE FILTERING COMMANDS ==========
    else if (command == "FILTER_ENABLE") {
      enable_filtering(true);
      Serial.println("[T41] ✓ Noise filtering enabled");
    }
    else if (command == "FILTER_DISABLE") {
      enable_filtering(false);
      Serial.println("[T41] ✓ Noise filtering disabled");
    }
    else if (command == "FILTER_STATUS") {
      show_filter_status();
    }
    else if (command == "FILTER_RESET") {
      reset_filters();
      Serial.println("[T41] ✓ All filters reset");
    }
    else if (command == "FILTER_REALTIME") {
      set_realtime_filtering();
      Serial.println("[T41] ✓ Real-time filtering preset applied");
    }
    else if (command == "FILTER_HIGH_QUALITY") {
      set_high_quality_filtering();
      Serial.println("[T41] ✓ High-quality filtering preset applied");
    }
    else if (command == "FILTER_LOW_NOISE") {
      set_low_noise_filtering();
      Serial.println("[T41] ✓ Low-noise filtering preset applied");
    }
    // ========== TARE OFFSET CONTROL ==========
    else if (command == "TARE_OFFSET_ON") {
      set_tare_offset_enabled(true);
      Serial.println("[T41] ✓ Tare offset applied to stream (values near 0 after tare)");
    }
    else if (command == "TARE_OFFSET_OFF") {
      set_tare_offset_enabled(false);
      Serial.println("[T41] ✓ Tare offset disabled (raw ADC values in stream)");
    }
    else if (command == "TARE_STATUS") {
      Serial.printf("[T41] Tare offset in stream: %s\n", is_tare_offset_enabled() ? "ENABLED" : "DISABLED");
      CalStatus status;
      cal_get_status(&status);
      Serial.printf("[T41] Tare offsets: LC1=%ld, LC2=%ld, LC3=%ld, LC4=%ld\n",
                    (long)status.offsets[0], (long)status.offsets[1], 
                    (long)status.offsets[2], (long)status.offsets[3]);
    }
    else if (command.startsWith("FILTER_TYPE ")) {
      int type = command.substring(12).toInt();
      if (type >= 0 && type <= 7) {
        set_filter_type(type);
        Serial.printf("[T41] ✓ Filter type set to: %d\n", type);
      } else {
        Serial.println("[T41] ✗ Invalid filter type (0-7)");
      }
    }
    else if (command.startsWith("OUTLIER_METHOD ")) {
      int method = command.substring(15).toInt();
      if (method >= 0 && method <= 4) {
        set_outlier_method(method);
        Serial.printf("[T41] ✓ Outlier method set to: %d\n", method);
      } else {
        Serial.println("[T41] ✗ Invalid outlier method (0-4)");
      }
    }
    else if (command.startsWith("GAUSSIAN_SIGMA ")) {
      double sigma = command.substring(15).toFloat();
      if (sigma > 0.0 && sigma <= 5.0) {
        set_gaussian_sigma(sigma);
        Serial.printf("[T41] ✓ Gaussian sigma set to: %.2f\n", sigma);
      } else {
        Serial.println("[T41] ✗ Invalid sigma value (0.1-5.0)");
      }
    }
    else if (command == "FILTER_GAUSSIAN") {
      set_gaussian_filtering();
      Serial.println("[T41] ✓ Gaussian filtering preset applied");
    }
    else if (command == "SHOW_FILTERED") {
      Serial.println("[T41] Raw vs Filtered Load Cell Readings:");
      for (int ch = 0; ch < 4; ch++) {
        FilteredReading reading = get_filtered_reading_with_info(ch);
        Serial.printf("[T41] LC%d: Raw=%ld, Filtered=%ld, Outlier=%s\n",
                      ch + 1, reading.raw, reading.filtered,
                      reading.outlier_detected ? "YES" : "NO");
      }
    }
    else if (command == "PING") {
      Serial.println("[T41] PING received, sending PONG...");
      softSerialTX_println("PONG");
      Serial.println("[T41] PONG sent");
    }
    else if (command == "TX_TEST") {
      Serial.println("[T41] Starting TX test (50 messages)...");
      char buf[20];
      for (int i = 0; i < 50; i++) {
        sprintf(buf, "TX_TEST_%d", i);
        softSerialTX_println(buf);
        Serial.printf("[T41] Sent: %s\n", buf);
        delay(100);
      }
      Serial.println("[T41] TX test complete");
    }
    // ========== CALIBRATION (REGRESSION v2, TEENSY-ONLY) ==========
    // Support both space format (legacy) and underscore format (new, uppercase)
    else if (command == "CAL SHOW" || command == "CAL_SHOW") {
      cal_print_status();
    }
    else if (command == "CAL POINTS" || command == "CAL_POINTS") {
      cal_print_points();
    }
    else if (command == "CAL READ" || command == "CAL_READ") {
      cal_print_reading(true);
    }
    else if (command == "CAL READ RAW" || command == "CAL_READ_RAW") {
      cal_print_reading(false);
    }
    else if (command == "CAL TARE" || command == "CAL_TARE") {
      Serial.println("[CAL] TARE: remove all load and keep still...");
      current_led_status = LED_CAL_TARE_COLLECTING;
      update_led_status();  // Update LED immediately to show solid color
      bool ok = cal_tare(1500, true);  // 1.5 seconds collection time
      if (ok) {
        current_led_status = LED_CAL_TARE_SUCCESS;
        led_success_start_ms = millis();
      } else {
        current_led_status = LED_CAL_ERROR;
      }
      Serial.println(ok ? "[CAL] TARE OK" : "[CAL] TARE ERROR");
    }
    else if (command.startsWith("CAL ADD ") || command.startsWith("CAL_ADD_")) {
      // Format: CAL ADD <kg> or CAL_ADD_<kg>
      float kg = 0.0f;
      if (command.startsWith("CAL ADD ")) {
        kg = command.substring(8).toFloat();
      } else {
        // CAL_ADD_<kg>
        kg = command.substring(8).toFloat();
      }
      Serial.printf("[CAL] ADD(TOTAL): apply %.3f kg and keep still...\n", (double)kg);
      current_led_status = LED_CAL_ADD_COLLECTING;
      update_led_status();  // Update LED immediately to show solid color
      bool ok = cal_add_point_total(kg, 1500, true);  // 1.5 seconds collection time
      if (ok) {
        Serial.println("[CAL] ADD OK -> auto-fitting...");
        current_led_status = LED_CAL_FIT_RUNNING;
        bool fit_ok = cal_fit_and_save();
        if (fit_ok) {
          current_led_status = LED_CAL_ADD_SUCCESS;
          led_success_start_ms = millis();
        } else {
          current_led_status = LED_CAL_ERROR;
        }
        Serial.println(fit_ok ? "[CAL] FIT OK" : "[CAL] FIT ERROR");
      } else {
        current_led_status = LED_CAL_ERROR;
        Serial.println("[CAL] ADD ERROR");
      }
    }
    else if (command.startsWith("CAL ADD_CH ") || command.startsWith("CAL_ADD_CH_")) {
      // Format: CAL ADD_CH <ch> <kg> or CAL_ADD_CH_<ch>_<weight>
      int ch = 0;
      float kg = 0.0f;
      if (command.startsWith("CAL ADD_CH ")) {
        int sp1 = command.indexOf(' ', 11);
        if (sp1 > 0) {
          ch = command.substring(11, sp1).toInt();
          kg = command.substring(sp1 + 1).toFloat();
        } else {
          Serial.println("[CAL] ADD_CH ERROR (usage: CAL ADD_CH <1-4> <kg>)");
          ch = 0;
        }
      } else {
        // CAL_ADD_CH_<ch>_<weight>
        int us1 = command.indexOf('_', 12);
        int us2 = command.indexOf('_', us1 + 1);
        if (us1 > 0 && us2 > 0) {
          ch = command.substring(12, us1).toInt();
          kg = command.substring(us2 + 1).toFloat();
        } else {
          Serial.println("[CAL] ADD_CH ERROR (usage: CAL_ADD_CH_<1-4>_<kg>)");
          ch = 0;
        }
      }
      if (ch > 0 && ch <= 4 && kg > 0.0f) {
        Serial.printf("[CAL] ADD(LC%d): apply %.3f kg mostly on that cell and keep still...\n", ch, (double)kg);
        current_led_status = LED_CAL_ADD_COLLECTING;
        update_led_status();  // Update LED immediately to show solid color
        bool ok = cal_add_point_channel((uint8_t)(ch - 1), kg, 1500, true);  // 1.5 seconds collection time
        if (ok) {
          Serial.println("[CAL] ADD_CH OK -> auto-fitting...");
          current_led_status = LED_CAL_FIT_RUNNING;
          bool fit_ok = cal_fit_and_save();
          if (fit_ok) {
            current_led_status = LED_CAL_ADD_SUCCESS;
            led_success_start_ms = millis();
          } else {
            current_led_status = LED_CAL_ERROR;
          }
          Serial.println(fit_ok ? "[CAL] FIT OK" : "[CAL] FIT ERROR");
        } else {
          current_led_status = LED_CAL_ERROR;
          Serial.println("[CAL] ADD_CH ERROR");
        }
      }
    }
    else if (command == "CAL FIT" || command == "CAL_FIT") {
      current_led_status = LED_CAL_FIT_RUNNING;
      bool ok = cal_fit_and_save();
      if (ok) {
        current_led_status = LED_CAL_FIT_SUCCESS;
        led_success_start_ms = millis();
      } else {
        current_led_status = LED_CAL_ERROR;
      }
      Serial.println(ok ? "[CAL] FIT OK" : "[CAL] FIT ERROR");
    }
    else if (command == "CAL DEBUG" || command == "CAL_DEBUG") {
      // Diagnostic command to debug calibration issues
      Serial.println("[CAL] ===== CALIBRATION DEBUG =====");
      CalStatus status;
      cal_get_status(&status);
      
      for (int ch = 0; ch < 4; ch++) {
        int32_t raw = read_single_channel_fast(ch);
        int32_t raw_filtered = get_raw_filtered_reading(ch);
        int32_t delta = get_filtered_load_cell_reading(ch);
        int32_t calibrated = get_calibrated_load_cell_reading(ch);
        float slope = cal_get_slope(ch);
        
        Serial.printf("[CAL] LC%d:\n", ch + 1);
        Serial.printf("[CAL]   raw_adc=%ld, raw_filt=%ld\n", (long)raw, (long)raw_filtered);
        Serial.printf("[CAL]   offset=%ld, delta=%ld\n", (long)status.offsets[ch], (long)delta);
        Serial.printf("[CAL]   slope=%.9f, polarity=%d\n", (double)slope, (int)status.polarity[ch]);
        Serial.printf("[CAL]   calibrated_10g=%ld (%.2f kg)\n", (long)calibrated, calibrated / 100.0);
      }
      
      // Sum of calibrated values
      int32_t total = 0;
      for (int ch = 0; ch < 4; ch++) {
        total += get_calibrated_load_cell_reading(ch);
      }
      Serial.printf("[CAL] TOTAL: %ld units (%.2f kg)\n", (long)total, total / 100.0);
      Serial.println("[CAL] ================================");
    }
    else if (command == "CAL CLEAR" || command == "CAL_CLEAR") {
      bool ok = cal_clear();
      if (ok) {
        current_led_status = LED_CAL_CLEAR;
        led_success_start_ms = millis();
      } else {
        current_led_status = LED_CAL_ERROR;
      }
      Serial.println(ok ? "[CAL] CLEAR OK" : "[CAL] CLEAR ERROR");
    }
    else if (command == "HELP") {
      Serial.println("[T41] === COMMANDS ===");
      Serial.println("[T41] Data: START, STOP, RESTART, RESET, STATUS");
      Serial.println("[T41] Filter: FILTER_ENABLE/DISABLE/STATUS/RESET");
      Serial.println("[T41] Presets: FILTER_REALTIME/HIGH_QUALITY/LOW_NOISE/GAUSSIAN");
      Serial.println("[T41] Config: FILTER_TYPE <0-7>, OUTLIER_METHOD <0-4>, GAUSSIAN_SIGMA <0.1-5>");
      Serial.println("[T41] Tare: TARE_OFFSET_ON/OFF, TARE_STATUS (control tare in stream)");
      Serial.println("[T41] Debug: SHOW_FILTERED, PING, TX_TEST");
      Serial.println("[T41] Cal: CAL_SHOW, CAL_POINTS, CAL_READ, CAL_READ_RAW, CAL_TARE, CAL_DEBUG");
      Serial.println("[T41]      CAL_ADD_<kg>, CAL_ADD_CH_<1-4>_<kg>, CAL_FIT, CAL_CLEAR");
      Serial.println("[T41] Mock: MOCK_ON, MOCK_OFF (generate test waveforms)");
    }
    else if (command == "") {
      // Empty command, do nothing
    }
    else {
      Serial.printf("[T41] ✗ Unknown command: '%s' (type HELP)\n", command.c_str());
    }
  }
}

// CRC16 function (CCITT-FALSE)
static inline uint16_t crc16_ccitt_false(const uint8_t* d, uint32_t n) {
  uint16_t crc = 0xFFFF;
  while (n--) {
    crc ^= ((uint16_t)*d++) << 8;
    for (int i=0;i<8;i++) crc = (crc & 0x8000) ? ((crc<<1) ^ 0x1021) : (crc<<1);
  }
  return crc;
}

// clamp to signed 24-bit and pack little-endian
static inline void pack_int24_le(uint8_t* p, int32_t v) {
  if (v >  0x7FFFFF) v =  0x7FFFFF;
  if (v < -0x800000) v = -0x800000;
  uint32_t u = (uint32_t)(v & 0xFFFFFF);
  p[0] = (uint8_t)(u & 0xFF);
  p[1] = (uint8_t)((u >> 8) & 0xFF);
  p[2] = (uint8_t)((u >> 16) & 0xFF);
}

void setup() {
  // Set booting state immediately
  current_led_status = LED_BOOTING;
  
  // Initialize LEDs early so booting state is visible
  pinMode(LED_EN1, OUTPUT);
  pinMode(LED_EN2, OUTPUT);
  pinMode(LED_EN3, OUTPUT);
  pinMode(LED_EN4, OUTPUT);
  pinMode(LED_DI1, OUTPUT);
  pinMode(LED_DI2, OUTPUT);
  pinMode(LED_DI3, OUTPUT);
  pinMode(LED_DI4, OUTPUT);
  
  // Hard boot sequence
  led_allEN(false);
  led_allDI_low();
  delay(500);
  
  led_dwt_init();
  
  led_allEN(true);
  delay(500);
  
  // Sync LEDs
  for (int i = 0; i < 5; i++) {
    led_setAll(0, 0, 0, 0);
    delay(20);
  }
  
  // Update LED to show booting state
  update_led_status();
  
  delay(1000);
  Serial.begin(115200);
  Serial.println("[T41] ADS1256 Force Plate System");
  
  // Initialize ADS1256 (SPI0)
  pinMode(ADS_CS_PIN, OUTPUT);
  pinMode(ADS_RDY_PIN, INPUT);
  pinMode(ADS_RST_PIN, OUTPUT);
  SPI.begin();
  initADS();
  update_led_status();  // Update LED during boot
  Serial.println("[T41] ADS1256 initialized");

  // Initialize SPI1 for ESP32 communication
  pinMode(PIN_CS1, OUTPUT);
  digitalWriteFast(PIN_CS1, HIGH);
  SPI1.setSCK(PIN_SCK1);
  SPI1.setMOSI(PIN_MOSI1);
  SPI1.setMISO(PIN_MISO1);
  SPI1.begin();
  update_led_status();  // Update LED during boot
  Serial.println("[T41] SPI1 Master initialized");
  
  // Initialize Serial3 for ESP32 command interface (RX only - Pin 15)
  Serial3.begin(9600);  // Lower baud rate for reliable software TX
  Serial.println("[T41] Serial3 RX initialized (Pin 15) at 9600 baud");
  
  // Initialize Software Serial TX on Pin 16 (PCB fixed wiring)
  softSerialTX_init();
  Serial.println("[T41] Software TX initialized (Pin 16)");

  // Initialize sample buffer
  memset(sample_buffer, 0, sizeof(sample_buffer));
  
  // Initialize in STOPPED state - wait for ESP32 commands
  current_state = STATE_STOPPED;
  state_changed = true;
  
  //determine the conversion factor
  bitToVolt = resolution*Gain/vRef;
  
  // Initialize statistics
  st.start_ms = millis();
  st.last_ms = st.start_ms;
  
  // Initialize noise filtering system
  set_realtime_filtering();  // Start with real-time preset
  enable_filtering(true);    // Enable filtering by default
  Serial.println("[T41] Noise filtering enabled (Real-time preset)");

  // Load calibration (Teensy-only, does not affect frame output format)
  bool cal_ok = cal_init_load();
  Serial.printf("[CAL] EEPROM load: %s\n", cal_ok ? "OK" : "DEFAULTS");
  update_led_status();  // Update LED during boot
  
  Serial.println("[T41] Ready - type HELP for commands");
  
  // Change LED status from booting to idle
  current_led_status = LED_IDLE;
  update_led_status();
  Serial.println("[T41] LEDs initialized");
  
  // Send initial state to ESP32
  send_state_update();
  
  // Request BLE connection status after boot (to set correct LED state)
  // Wait a bit for ESP32 chain to be ready
  delay(500);
  Serial.println("[T41] Requesting BLE connection status...");
  softSerialTX_println("REQUEST_BLE_STATUS");
}

void loop() {
  // Handle ESP32 commands first
  handle_esp32_commands();
  
  // Handle Serial Monitor commands for testing
  handle_serial_monitor_commands();
  
  // Update LED status (every 50ms for smooth animations)
  // Skip LED updates during RUNNING state to avoid timing interference
  // (WS2812 uses noInterrupts() which blocks millis()/timing)
  static uint32_t last_led_update = 0;
  if (current_state != STATE_RUNNING && millis() - last_led_update >= 50) {
    update_led_status();
    last_led_update = millis();
  }
  
  // Send state updates when state changes
  if (state_changed) {
    send_state_update();
  }
  
  // Only perform data acquisition if in RUNNING state
  if (current_state == STATE_RUNNING) {
    // Sample at 1kHz per channel (4kHz total) - rotate through channels every 250μs
    static elapsedMicros sample_timer;
    static uint8_t current_channel = 0;
    static uint32_t sample_count = 0;
    
    if (sample_timer >= 250) {  // 4kHz total sampling rate
      sample_timer = 0;
      
      // Read current channel with CALIBRATION applied (10g units)
      // This gives weight values that fit in int16 and represent actual weight
      int32_t calibrated_value = get_calibrated_load_cell_reading(current_channel);
      
      // Store in buffer
      sample_buffer[current_channel][buffer_index] = calibrated_value;
      
      // Move to next channel
      current_channel++;
      if (current_channel >= CHANNELS) {
        current_channel = 0;
        // All 4 channels sampled, increment buffer index
        buffer_index++;
        
        // When buffer is full (10 samples collected), send frame immediately
        // This guarantees every frame contains exactly 10 fresh, sequential samples
        if (buffer_index >= SAMPLES_PER_FRAME) {
          buffer_index = 0;
          send_frame_to_esp32();
        }
      }
      
      sample_count++;
    }

    // Print statistics every 5 seconds (only when running)
    uint32_t now_ms = millis();
    if (now_ms - st.last_ms >= 5000) {
      print_statistics();
      st.last_ms = now_ms;
      st.min_xfer_us = UINT32_MAX; 
      st.max_xfer_us = 0; // reset window
    }
  } else {
    // When not running, just handle commands and small delay
    delay(10);
  }
}

void send_frame_to_esp32() {
  // Fill frame header
  tx.sync[0] = 0xA5; 
  tx.sync[1] = 0x5A;
  tx.plate_id = 1; 
  tx.proto_ver = 0x01;
  tx.frame_idx = frame_idx++;
  tx.t0_us = micros();

  // Pack 10 samples from each of the 4 channels into the frame
  // Since we send immediately when buffer is full, samples 0-9 are always fresh and in order
  uint8_t* s = tx.samples;
  for (int i = 0; i < SAMPLES_PER_FRAME; i++) {
    // Pack 4 channels × 3 bytes each (24-bit little-endian)
    pack_int24_le(s + 0,  sample_buffer[0][i]); // LC1
    pack_int24_le(s + 3,  sample_buffer[1][i]); // LC2
    pack_int24_le(s + 6,  sample_buffer[2][i]); // LC3
    pack_int24_le(s + 9,  sample_buffer[3][i]); // LC4
    s += (CHANNELS * 3); // 4ch × 3 bytes = 12 bytes per sample
  }

  // Clear padding
  memset(tx.pad, 0, sizeof(tx.pad));

  // Calculate CRC16 over header + samples (bytes 0..129)
  tx.crc16 = 0;
  tx.crc16 = crc16_ccitt_false((uint8_t*)&tx, offsetof(Frame, crc16));

  // SPI transfer to ESP32 at 10 MHz
  SPISettings settings(10000000, MSBFIRST, SPI_MODE0);
  uint32_t t0 = micros();
  SPI1.beginTransaction(settings);
  digitalWriteFast(PIN_CS1, LOW);
  SPI1.transfer((uint8_t*)&tx, FRAME_BYTES);
  digitalWriteFast(PIN_CS1, HIGH);
  SPI1.endTransaction();
  uint32_t dur = micros() - t0;
  
  // Update statistics
  if (dur < st.min_xfer_us) st.min_xfer_us = dur;
  if (dur > st.max_xfer_us) st.max_xfer_us = dur;
  st.frames_sent++;
}

void print_statistics() {
  double secs = (millis() - st.start_ms) / 1000.0;
  double kbps = (st.frames_sent * FRAME_BYTES * 8.0) / secs / 1000.0;
  double total_sps = (st.frames_sent * SAMPLES_PER_FRAME * CHANNELS) / secs; // 4 channels per sample
  double sps_per_channel = total_sps / CHANNELS;

  Serial.printf("[T41] t=%.1fs frames=%lu rate=%.1f kbps sps=%.0f (%.0f/ch) xfer[%lu-%lu us]\n",
    secs, (unsigned long)st.frames_sent, kbps, total_sps, sps_per_channel,
    (unsigned long)(st.min_xfer_us == UINT32_MAX ? 0 : st.min_xfer_us),
    (unsigned long)st.max_xfer_us);
    
  // Performance status
  if (sps_per_channel >= 1000.0) {
    Serial.println("[T41] ✓ 1000+ SPS per channel achieved");
  } else if (sps_per_channel >= 800.0) {
    Serial.println("[T41] ~ Near target SPS");
  }
}
