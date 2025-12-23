// ============================================================================
// MOCK DATA GENERATOR
// ============================================================================
// This file provides mock data generation for testing data rates and reliability
// without requiring physical load cells. Can be easily removed by deleting this file.
//
// Wave types per channel:
//   Channel 0: Square wave
//   Channel 1: Triangle wave
//   Channel 2: Sawtooth wave
//   Channel 3: Sine wave
// ============================================================================

#ifndef MOCK_DATA_GENERATOR_DEFINED
#define MOCK_DATA_GENERATOR_DEFINED

#include <math.h>

// Mock data enable flag
static bool mock_data_enabled = false;

// Wave generation parameters
static const float MOCK_FREQUENCY = 1.0f;  // 1 Hz base frequency

// Output range configuration
// Option 1: -10 to +10 (centered at 0) - CURRENT SETTING
// Option 2: 0 to 20 (offset from 0) - Change MOCK_MIN to 0, MOCK_MAX to 20
static const int32_t MOCK_MIN = -10;  // Minimum output value (int16 compatible)
static const int32_t MOCK_MAX = 10;   // Maximum output value (int16 compatible)
static const int32_t MOCK_RANGE = MOCK_MAX - MOCK_MIN;  // Range: 20

// Time tracking for wave generation (use microsecond precision for smoothness)
static uint32_t mock_start_time_ms = 0;
static uint32_t mock_start_time_us = 0;

// ============================================================================
// WAVE GENERATION FUNCTIONS
// ============================================================================

// Square wave: y = sign(sin(2πft))
// Output: MOCK_MIN or MOCK_MAX
static inline int32_t generate_square_wave(float t, float frequency) {
  float phase = 2.0f * 3.14159265359f * frequency * t;
  float sine_val = sin(phase);
  return (sine_val >= 0.0f) ? MOCK_MAX : MOCK_MIN;
}

// Triangle wave: Linear ramp up then down
// Output: MOCK_MIN to MOCK_MAX (linear ramp)
// Direct calculation for perfect smoothness
static inline int32_t generate_triangle_wave(float t, float frequency) {
  float phase = t * frequency;
  // Get fractional part of phase [0, 1)
  float fractional = phase - floor(phase);
  
  // Generate triangle: 0->1->0 over one period
  float triangle_val;
  if (fractional < 0.5f) {
    // Rising edge: 0 to 0.5 -> output goes from -1 to 0
    triangle_val = fractional * 4.0f - 1.0f;  // [-1, 1]
  } else {
    // Falling edge: 0.5 to 1.0 -> output goes from 1 to -1
    triangle_val = 3.0f - fractional * 4.0f;  // [1, -1]
  }
  
  // Scale from [-1, 1] to [MOCK_MIN, MOCK_MAX]
  float scaled = (triangle_val + 1.0f) / 2.0f;  // [0, 1]
  return MOCK_MIN + (int32_t)roundf(scaled * MOCK_RANGE);
}

// Sawtooth wave: y = 2(tf - floor(tf + 0.5))
// Output: MOCK_MIN to MOCK_MAX (periodic reset)
static inline int32_t generate_sawtooth_wave(float t, float frequency) {
  float phase = t * frequency;
  // Use fractional part for smooth sawtooth
  float fractional = phase - floor(phase);  // [0, 1)
  float sawtooth_val = 2.0f * fractional - 1.0f;  // [-1, 1)
  // Scale from [-1, 1] to [MOCK_MIN, MOCK_MAX]
  float scaled = (sawtooth_val + 1.0f) / 2.0f;  // [0, 1)
  return MOCK_MIN + (int32_t)roundf(scaled * MOCK_RANGE);
}

// Sine wave: y = sin(2πft)
// Output: MOCK_MIN to MOCK_MAX (smooth sinusoidal)
static inline int32_t generate_sine_wave(float t, float frequency) {
  float phase = 2.0f * 3.14159265359f * frequency * t;
  float sine_val = sin(phase);  // [-1, 1]
  // Scale from [-1, 1] to [MOCK_MIN, MOCK_MAX]
  float scaled = (sine_val + 1.0f) / 2.0f;  // [0, 1]
  return MOCK_MIN + (int32_t)roundf(scaled * MOCK_RANGE);
}

// ============================================================================
// PUBLIC API
// ============================================================================

// Enable/disable mock data generation
void mock_data_set_enabled(bool enabled) {
  mock_data_enabled = enabled;
  if (enabled) {
    mock_start_time_ms = millis();
    mock_start_time_us = micros();
    Serial.println("[MOCK] Mock data generation ENABLED");
    Serial.printf("[MOCK] Output range: %d to %d (int16 compatible)\n", MOCK_MIN, MOCK_MAX);
    Serial.println("[MOCK] Ch0: Square, Ch1: Triangle, Ch2: Sawtooth, Ch3: Sine");
  } else {
    Serial.println("[MOCK] Mock data generation DISABLED");
  }
}

bool mock_data_is_enabled() {
  return mock_data_enabled;
}

// Generate mock data for a specific channel
int32_t mock_data_generate(uint8_t channel) {
  if (!mock_data_enabled || channel >= 4) {
    return 0;
  }
  
  // Calculate time in seconds with microsecond precision for smoothness
  // Use micros() for better resolution, but handle overflow
  uint32_t current_us = micros();
  uint32_t elapsed_us;
  
  // Handle micros() overflow (happens every ~70 minutes)
  if (current_us >= mock_start_time_us) {
    elapsed_us = current_us - mock_start_time_us;
  } else {
    // Overflow occurred, calculate from millis() as fallback
    elapsed_us = (millis() - mock_start_time_ms) * 1000UL;
  }
  
  // Convert to seconds with high precision
  float t = elapsed_us / 1000000.0f;
  
  // Generate different wave types for each channel
  int32_t value = 0;
  switch (channel) {
    case 0:
      // Square wave
      value = generate_square_wave(t, MOCK_FREQUENCY);
      break;
      
    case 1:
      // Triangle wave
      value = generate_triangle_wave(t, MOCK_FREQUENCY);
      break;
      
    case 2:
      // Sawtooth wave
      value = generate_sawtooth_wave(t, MOCK_FREQUENCY);
      break;
      
    case 3:
      // Sine wave
      value = generate_sine_wave(t, MOCK_FREQUENCY);
      break;
      
    default:
      value = 0;
      break;
  }
  
  // Value is already in [MOCK_MIN, MOCK_MAX] range
  return value;
}

#endif // MOCK_DATA_GENERATOR_DEFINED


