#pragma once

#include <Arduino.h>

// Calibration + regression (Teensy-only). Keeps streaming/frame output unchanged.
// Units:
// - Output mass is in 10g units (0.01 kg). 1 kg = 100 units.

#ifndef CHANNELS
#define CHANNELS 4
#endif

// Public API (called from main .ino)
bool cal_init_load();
bool cal_clear();

bool cal_tare(uint32_t window_ms = 600, bool use_filtered = true);

// Add a multi-load calibration point using TOTAL load on the plate.
// This creates a point for EACH load cell by distributing the known total load across cells
// proportional to their measured delta counts at that moment.
bool cal_add_point_total(float known_kg, uint32_t window_ms = 800, bool use_filtered = true);

// Optional: add point for a specific channel (requires you to load that cell predominantly).
bool cal_add_point_channel(uint8_t ch, float known_kg, uint32_t window_ms = 800, bool use_filtered = true);

// Fit regression from collected points and save to EEPROM.
bool cal_fit_and_save();

// Read current mass
int32_t cal_read_total_10g_units(bool use_filtered = true);
int32_t cal_read_cell_10g_units(uint8_t ch, bool use_filtered = true);

// Prints
void cal_print_status();
void cal_print_points();
void cal_print_reading(bool use_filtered = true);

// Getters for ESP32 output (access calibration data)
struct CalPoint {
  float x_counts;
  float y_10g;
};
struct CalStatus {
  int32_t offsets[CHANNELS];
  float ch_a_10g_per_count[CHANNELS];
  uint8_t points_ch_n[CHANNELS];
  int8_t polarity[CHANNELS];  // +1 = normal, -1 = inverted wiring (auto-detected)
};
void cal_get_status(CalStatus* out);
bool cal_get_points(uint8_t ch, CalPoint* out_points, uint8_t max_points, uint8_t* out_count);

// Get polarity for a single channel (used by streaming functions)
// Returns +1 for normal wiring, -1 for inverted wiring
int8_t cal_get_polarity(uint8_t ch);

// Get calibration slope for a single channel (used by streaming functions)
// Returns the slope 'a' such that: weight_10g = delta_counts * a
float cal_get_slope(uint8_t ch);


