#include "calibration_regression.h"

#include <EEPROM.h>
#include <math.h>

// External ADC read helpers from existing sketch
extern int32_t read_single_channel_fast(uint8_t channel);
extern int32_t get_raw_filtered_reading(uint8_t channel);  // Filtered but NO tare/polarity

// External LED update function (defined in main .ino file)
extern void update_led_status();

// ---------------- Manual polarity override (no CAL_ADD needed) ----------------
// Per-channel polarity when wiring differs between plates. Edit this array to fix
// channels that show negative when loaded (use -1) or positive when unloaded (use +1).
// 0 = use auto-detected polarity from CAL_ADD; +1 or -1 = force that polarity for the channel.
// Example: {0, 0, -1, 0} forces channel 3 (0-based) inverted; others use auto.
static const int8_t k_polarity_override[CHANNELS] = {-1, -1, -1, -1};

// ---------------- Storage ----------------
static const uint32_t CAL_MAGIC   = 0xCA1BCA1Bu;
static const uint16_t CAL_VERSION = 0x0004u; // v4: added polarity auto-detection
static const int      CAL_EEPROM_ADDR = 0;

// Display-space tare: separate small blob so we don't change main cal layout
static const uint32_t DISPLAY_TARE_MAGIC = 0xD70E0D70u;  // "display tare" in hex
static const int      DISPLAY_TARE_EEPROM_ADDR = 512;
struct DisplayTareBlob {
  uint32_t magic;
  int32_t  display_tare_10g[CHANNELS];
};
static int32_t g_display_tare_10g[CHANNELS];
static bool g_display_tare_loaded = false;

// Keep everything fixed-size: no heap allocations.
static const uint8_t CAL_MAX_POINTS = 10;

// CalPoint is defined in calibration_regression.h

struct CalBlob {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved;

  int32_t offsets[CHANNELS];   // tare offsets in counts

  // Per-channel scale only: y = a*x, x is |delta_counts[ch]|
  float ch_a_10g_per_count[CHANNELS];

  // Per-channel polarity: +1 = normal, -1 = inverted wiring
  // Auto-detected during CAL_ADD based on signal direction
  int8_t polarity[CHANNELS];

  uint8_t points_ch_n[CHANNELS];
#if (CHANNELS % 4) != 0
  uint8_t _pad_ch_n[4 - (CHANNELS % 4)];
#else
  uint8_t _pad_ch_n[0];
#endif
  CalPoint points_ch[CHANNELS][CAL_MAX_POINTS];

  uint32_t crc32;
};

// Alignment check - commented out to avoid compiler issues
// static_assert((offsetof(CalBlob, points_ch) % 4) == 0, "points_ch must be 4-byte aligned");

static CalBlob g_cal;
static bool g_loaded = false;
static bool g_dirty = false;

static uint32_t crc32_sw(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFFu;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; i++) {
      uint32_t mask = -(crc & 1u);
      crc = (crc >> 1) ^ (0xEDB88320u & mask);
    }
  }
  return ~crc;
}

static uint32_t cal_crc(const CalBlob& c) {
  return crc32_sw((const uint8_t*)&c, sizeof(CalBlob) - sizeof(uint32_t));
}

static void cal_defaults() {
  memset(&g_cal, 0, sizeof(g_cal));
  g_cal.magic = CAL_MAGIC;
  g_cal.version = CAL_VERSION;
  g_cal.reserved = 0;
  for (int i = 0; i < CHANNELS; i++) g_cal.offsets[i] = 0;

  for (int i = 0; i < CHANNELS; i++) {
    g_cal.ch_a_10g_per_count[i] = 0.0f;
    g_cal.points_ch_n[i] = 0;
    g_cal.polarity[i] = 1;  // Default: normal polarity (+1)
  }
  g_cal.crc32 = cal_crc(g_cal);
}

static bool cal_load_internal() {
  EEPROM.get(CAL_EEPROM_ADDR, g_cal);
  if (g_cal.magic != CAL_MAGIC || g_cal.version != CAL_VERSION) {
    cal_defaults();
    g_loaded = true;
    return false;
  }
  const uint32_t expect = cal_crc(g_cal);
  if (expect != g_cal.crc32) {
    cal_defaults();
    g_loaded = true;
    return false;
  }
  g_loaded = true;
  return true;
}

static bool cal_save_internal() {
  if (!g_loaded) cal_load_internal();
  g_cal.crc32 = cal_crc(g_cal);
  // Write byte-wise with yields to avoid long blocking flash writes that can look like a reset.
  const uint8_t* src = (const uint8_t*)&g_cal;
  for (unsigned i = 0; i < sizeof(CalBlob); i++) {
    EEPROM.update(CAL_EEPROM_ADDR + (int)i, src[i]);
    if ((i & 0x3F) == 0) yield();
  }
  g_dirty = false;
  return true;
}

static void display_tare_load() {
  if (g_display_tare_loaded) return;
  DisplayTareBlob b;
  EEPROM.get(DISPLAY_TARE_EEPROM_ADDR, b);
  if (b.magic != DISPLAY_TARE_MAGIC) {
    for (int i = 0; i < CHANNELS; i++) g_display_tare_10g[i] = 0;
  } else {
    for (int i = 0; i < CHANNELS; i++) g_display_tare_10g[i] = b.display_tare_10g[i];
  }
  g_display_tare_loaded = true;
}

static bool display_tare_save() {
  DisplayTareBlob b;
  b.magic = DISPLAY_TARE_MAGIC;
  for (int i = 0; i < CHANNELS; i++) b.display_tare_10g[i] = g_display_tare_10g[i];
  const uint8_t* src = (const uint8_t*)&b;
  for (unsigned i = 0; i < sizeof(DisplayTareBlob); i++) {
    EEPROM.update(DISPLAY_TARE_EEPROM_ADDR + (int)i, src[i]);
    if ((i & 0x3F) == 0) yield();
  }
  return true;
}

bool cal_init_load() {
  bool ok = cal_load_internal();
  display_tare_load();
  return ok;
}

bool cal_clear() {
  cal_defaults();
  g_loaded = true;
  g_dirty = true;
  return cal_save_internal();
}

// ---------------- Sampling helpers ----------------
struct CaptureStats {
  int32_t mean[CHANNELS];
  float stddev[CHANNELS];
  uint32_t n;
};

static int32_t read_counts(uint8_t ch, bool use_filtered) {
  // Use get_raw_filtered_reading() which applies filtering but NOT tare offset or polarity
  // This is essential for calibration to work correctly
  return use_filtered ? get_raw_filtered_reading(ch) : read_single_channel_fast(ch);
}

static bool capture_window(uint32_t window_ms, bool use_filtered, CaptureStats* out) {
  if (!out) return false;
  memset(out, 0, sizeof(*out));

  double mean[CHANNELS] = {0,0,0,0};
  double m2[CHANNELS]   = {0,0,0,0};
  uint32_t n = 0;

  const uint32_t t0 = millis();
  uint32_t last_led_update = 0;
  while ((millis() - t0) < window_ms) {
    n++;
    for (uint8_t ch = 0; ch < CHANNELS; ch++) {
      const double x = (double)read_counts(ch, use_filtered);
      const double d = x - mean[ch];
      mean[ch] += d / (double)n;
      const double d2 = x - mean[ch];
      m2[ch] += d * d2;
    }
    delayMicroseconds(300);
    yield();
    
    // Update LED status every 50ms during collection
    uint32_t now = millis();
    if (now - last_led_update >= 50) {
      update_led_status();
      last_led_update = now;
    }
  }

  out->n = n;
  for (uint8_t ch = 0; ch < CHANNELS; ch++) {
    out->mean[ch] = (int32_t)llround(mean[ch]);
    const double var = (n > 1) ? (m2[ch] / (double)(n - 1)) : 0.0;
    out->stddev[ch] = (float)sqrt(var);
  }
  return (n > 5);
}

bool cal_tare(uint32_t window_ms, bool use_filtered) {
  if (!g_loaded) cal_load_internal();

  const uint8_t trials = 3;
  float best = 1e30f;
  CaptureStats best_s = {};
  bool ok = false;

  for (uint8_t i = 0; i < trials; i++) {
    CaptureStats s;
    if (!capture_window(window_ms, use_filtered, &s)) continue;
    ok = true;
    float score = 0.0f;
    for (int ch = 0; ch < CHANNELS; ch++) score += s.stddev[ch];
    if (score < best) { best = score; best_s = s; }
    delay(20);
    yield();
  }
  if (!ok) return false;
  // When no calibration points exist (pre-calibrated cells), use relaxed threshold so tare can succeed
  bool no_cal_points = true;
  for (int ch = 0; ch < CHANNELS && no_cal_points; ch++)
    if (g_cal.points_ch_n[ch] != 0) no_cal_points = false;
  const float stability_limit = no_cal_points ? 100000.0f : 20000.0f;
  if (!(best < stability_limit)) {
    Serial.printf("[CAL] TARE rejected: unstable (sum_std=%.1f, limit=%.0f)\n", (double)best, (double)stability_limit);
    return false;
  }
  for (int ch = 0; ch < CHANNELS; ch++) g_cal.offsets[ch] = best_s.mean[ch];
  g_dirty = true;
  return cal_save_internal();
}

bool cal_tare_force(uint32_t window_ms, bool use_filtered) {
  if (!g_loaded) cal_load_internal();

  const uint8_t trials = 3;
  float best = 1e30f;
  CaptureStats best_s = {};
  bool ok = false;

  for (uint8_t i = 0; i < trials; i++) {
    CaptureStats s;
    if (!capture_window(window_ms, use_filtered, &s)) continue;
    ok = true;
    float score = 0.0f;
    for (int ch = 0; ch < CHANNELS; ch++) score += s.stddev[ch];
    if (score < best) { best = score; best_s = s; }
    delay(20);
    yield();
  }
  if (!ok) return false;
  // Skip stability check - store current mean as offset (use when plate is known unloaded)
  Serial.printf("[CAL] TARE FORCE: storing offsets (sum_std=%.1f, stability not checked)\n", (double)best);
  for (int ch = 0; ch < CHANNELS; ch++) g_cal.offsets[ch] = best_s.mean[ch];
  g_dirty = true;
  return cal_save_internal();
}

// ---------------- Regression ----------------
static bool fit_through_zero(const CalPoint* pts, uint8_t n, float* out_a) {
  // Enforce no intercept: y = a*x (so mass is 0 when delta_counts is 0)
  if (!out_a) return false;
  if (n < 1) return false;

  double sxx = 0.0;
  double sxy = 0.0;
  for (uint8_t i = 0; i < n; i++) {
    const double x = pts[i].x_counts;
    const double y = pts[i].y_10g;
    sxx += x * x;
    sxy += x * y;
  }
  if (sxx < 1e-9) return false;
  const double a = sxy / sxx;
  if (!isfinite(a)) return false;
  *out_a = (float)a;
  return true;
}

bool cal_add_point_total(float known_kg, uint32_t window_ms, bool use_filtered) {
  if (!g_loaded) cal_load_internal();
  if (known_kg <= 0.0f) return false;

  CaptureStats s;
  if (!capture_window(window_ms, use_filtered, &s)) return false;

  // Compute per-cell deltas using GEOMETRY-BASED calibration.
  // When weight is placed at CENTER of the plate, each of the 4 corner load cells
  // carries exactly 25% of the total weight (due to symmetry).
  // This gives each cell its own individual slope to compensate for sensitivity differences.
  double delta[CHANNELS] = {0,0,0,0};
  int valid_channels = 0;
  
  for (int ch = 0; ch < CHANNELS; ch++) {
    const double d = (double)s.mean[ch] - (double)g_cal.offsets[ch];
    const double ad = fabs(d);

    // Skip saturated channels (ADS1256 rails) to avoid corrupting calibration points.
    // 0x7FFFFF is full-scale max; 0x800000 is min (signed 24-bit).
    if (s.mean[ch] >= 0x7FFFF0 || s.mean[ch] <= -0x7FFF00) {
      delta[ch] = 0.0;
      Serial.printf("[CAL] LC%d SATURATED - skipping\n", ch + 1);
      continue;
    }

    // Auto-detect polarity: if delta is negative, the load cell is wired in reverse
    // Only update polarity if we have a significant signal (> 1000 counts)
    if (ad > 1000.0) {
      g_cal.polarity[ch] = (d < 0) ? -1 : 1;
      Serial.printf("[CAL] LC%d polarity auto-detected: %s\n", 
                    ch + 1, (d < 0) ? "INVERTED" : "NORMAL");
    }

    if (ad > 100.0) {  // Minimum threshold for valid reading
      delta[ch] = ad;
      valid_channels++;
    }
  }
  
  if (valid_channels < 1) {
    Serial.println("[CAL] ERROR: No valid channels detected");
    return false;
  }

  // GEOMETRY-BASED CALIBRATION:
  // Each cell carries 25% of the total weight when weight is at center.
  // This creates individual slopes that compensate for sensitivity differences.
  // Result: sum remains constant regardless of where weight is placed on plate.
  const float total_y10g = known_kg * 100.0f;
  const float weight_per_cell = total_y10g / (float)CHANNELS;  // 25% each for 4 cells
  
  Serial.printf("[CAL] Total weight: %.1f kg = %.0f (10g units)\n", (double)known_kg, (double)total_y10g);
  Serial.printf("[CAL] Weight per cell (geometry): %.0f (10g units) = %.2f kg\n", 
                (double)weight_per_cell, (double)(weight_per_cell / 100.0f));
  
  for (int ch = 0; ch < CHANNELS; ch++) {
    if (delta[ch] <= 0.0) continue;
    if (g_cal.points_ch_n[ch] >= CAL_MAX_POINTS) continue;
    
    // Each cell is assigned 25% of total weight (geometry-based)
    // This gives each cell its own slope: slope = weight_per_cell / delta_counts
    g_cal.points_ch[ch][g_cal.points_ch_n[ch]++] = CalPoint{ (float)delta[ch], weight_per_cell };
    
    // Calculate and display the individual slope for this channel
    float slope = weight_per_cell / (float)delta[ch];
    Serial.printf("[CAL] LC%d: counts=%.0f, weight=%.0f, slope=%.9f\n",
                  ch + 1, delta[ch], (double)weight_per_cell, (double)slope);
  }

  g_dirty = true;
  // Do NOT EEPROM-write on every ADD; keep points in RAM and persist on FIT to reduce flash activity.
  return true;
}

bool cal_add_point_channel(uint8_t ch, float known_kg, uint32_t window_ms, bool use_filtered) {
  if (!g_loaded) cal_load_internal();
  if (ch >= CHANNELS) return false;
  if (known_kg <= 0.0f) return false;
  if (g_cal.points_ch_n[ch] >= CAL_MAX_POINTS) return false;

  CaptureStats s;
  if (!capture_window(window_ms, use_filtered, &s)) return false;

  const double d = (double)s.mean[ch] - (double)g_cal.offsets[ch];
  const double ad = fabs(d);
  if (ad < 1.0) return false;

  // Auto-detect polarity: if delta is negative, the load cell is wired in reverse
  // Only update polarity if we have a significant signal (> 1000 counts)
  if (ad > 1000.0) {
    g_cal.polarity[ch] = (d < 0) ? -1 : 1;
    Serial.printf("[CAL] LC%d polarity auto-detected: %s\n", 
                  ch + 1, (d < 0) ? "INVERTED" : "NORMAL");
  }

  const float y10g = known_kg * 100.0f;
  g_cal.points_ch[ch][g_cal.points_ch_n[ch]++] = CalPoint{ (float)ad, y10g };
  g_dirty = true;
  return true;
}

bool cal_fit_and_save() {
  if (!g_loaded) cal_load_internal();

  bool any = false;
  for (int ch = 0; ch < CHANNELS; ch++) {
    const uint8_t n = g_cal.points_ch_n[ch];
    if (n < 1) continue;
    float a = 0.0f;
    if (!fit_through_zero(g_cal.points_ch[ch], n, &a)) continue;
    g_cal.ch_a_10g_per_count[ch] = a;
    any = true;
  }
  if (!any) return false;

  g_dirty = true;
  return cal_save_internal();
}

int32_t cal_read_cell_10g_units(uint8_t ch, bool use_filtered) {
  if (!g_loaded) cal_load_internal();
  if (ch >= CHANNELS) return 0;

  const int32_t counts = read_counts(ch, use_filtered);
  const float dx = (float)fabs((double)((int32_t)(counts - g_cal.offsets[ch])));

  const float a = g_cal.ch_a_10g_per_count[ch];
  if (a == 0.0f) return 0;
  return (int32_t)lroundf(a * dx);
}

int32_t cal_read_total_10g_units(bool use_filtered) {
  if (!g_loaded) cal_load_internal();

  int32_t total = 0;
  for (int ch = 0; ch < CHANNELS; ch++) total += cal_read_cell_10g_units((uint8_t)ch, use_filtered);
  return total;
}

void cal_print_status() {
  if (!g_loaded) cal_load_internal();
  Serial.println("[CAL] ===== Calibration v4 (geometry-based) =====");
  Serial.println("[CAL] Geometry: 22cm x 35cm plate, 4 corner load cells");
  Serial.println("[CAL] Each cell carries 25% of total weight when centered");
  Serial.println();
  
  float sum_slopes = 0.0f;
  float min_slope = 1e9f;
  float max_slope = 0.0f;
  int valid_slopes = 0;
  
  for (int ch = 0; ch < CHANNELS; ch++) {
    float slope = g_cal.ch_a_10g_per_count[ch];
    Serial.printf("[CAL] LC%d: offset=%ld, slope=%.9f, pts=%u, pol=%s\n",
                  ch+1, (long)g_cal.offsets[ch],
                  (double)slope,
                  (unsigned)g_cal.points_ch_n[ch],
                  g_cal.polarity[ch] < 0 ? "INV" : "NRM");
    
    if (slope > 0.0f) {
      sum_slopes += slope;
      if (slope < min_slope) min_slope = slope;
      if (slope > max_slope) max_slope = slope;
      valid_slopes++;
    }
  }
  
  if (valid_slopes > 0) {
    float avg_slope = sum_slopes / (float)valid_slopes;
    float spread_pct = ((max_slope - min_slope) / avg_slope) * 100.0f;
    Serial.println();
    Serial.printf("[CAL] Slope stats: avg=%.9f, min=%.9f, max=%.9f\n",
                  (double)avg_slope, (double)min_slope, (double)max_slope);
    Serial.printf("[CAL] Slope spread: %.2f%% (individual cell sensitivity differences)\n",
                  (double)spread_pct);
  }
  
  Serial.println();
  Serial.println("[CAL] Units: 10g (0.01kg). 1kg = 100 units.");
}

void cal_print_points() {
  if (!g_loaded) cal_load_internal();
  for (int ch = 0; ch < CHANNELS; ch++) {
    Serial.printf("[CAL] Points (LC%d):\n", ch+1);
    for (uint8_t i = 0; i < g_cal.points_ch_n[ch]; i++) {
      Serial.printf("[CAL]  #%u x_counts=%.1f y_10g=%.1f\n",
                    (unsigned)i, (double)g_cal.points_ch[ch][i].x_counts, (double)g_cal.points_ch[ch][i].y_10g);
    }
  }
}

void cal_print_reading(bool use_filtered) {
  for (int ch = 0; ch < CHANNELS; ch++) {
    const int32_t u = cal_read_cell_10g_units((uint8_t)ch, use_filtered);
    Serial.printf("[CAL] LC%d=%ld (10g units)\n", ch + 1, (long)u);
  }
  const int32_t tot = cal_read_total_10g_units(use_filtered);
  Serial.printf("[CAL] TOTAL=%ld (10g units)\n", (long)tot);
}

void cal_get_status(CalStatus* out) {
  if (!g_loaded) cal_load_internal();
  if (out == nullptr) return;
  for (int ch = 0; ch < CHANNELS; ch++) {
    out->offsets[ch] = g_cal.offsets[ch];
    out->ch_a_10g_per_count[ch] = g_cal.ch_a_10g_per_count[ch];
    out->points_ch_n[ch] = g_cal.points_ch_n[ch];
    out->polarity[ch] = g_cal.polarity[ch];
  }
}

// Get polarity for a single channel (used by streaming functions)
// Uses k_polarity_override when set (non-zero); otherwise uses auto-detected polarity from CAL_ADD.
int8_t cal_get_polarity(uint8_t ch) {
  if (ch >= CHANNELS) return 1;
  if (k_polarity_override[ch] != 0)
    return k_polarity_override[ch];
  if (!g_loaded) cal_load_internal();
  return g_cal.polarity[ch];
}

// Get calibration slope for a single channel (used by streaming functions)
// Returns the slope 'a' such that: weight_10g = delta_counts * a
float cal_get_slope(uint8_t ch) {
  if (!g_loaded) cal_load_internal();
  if (ch >= CHANNELS) return 0.0f;
  return g_cal.ch_a_10g_per_count[ch];
}

bool cal_get_points(uint8_t ch, CalPoint* out_points, uint8_t max_points, uint8_t* out_count) {
  if (!g_loaded) cal_load_internal();
  if (ch >= CHANNELS || out_points == nullptr || out_count == nullptr) return false;
  const uint8_t n = g_cal.points_ch_n[ch];
  const uint8_t copy_n = (n < max_points) ? n : max_points;
  for (uint8_t i = 0; i < copy_n; i++) {
    out_points[i].x_counts = g_cal.points_ch[ch][i].x_counts;
    out_points[i].y_10g = g_cal.points_ch[ch][i].y_10g;
  }
  *out_count = copy_n;
  return true;
}

// Display-space tare: zero the displayed value (for pre-calibrated load cells)
int32_t cal_get_display_tare_10g(uint8_t ch) {
  if (ch >= CHANNELS) return 0;
  display_tare_load();
  return g_display_tare_10g[ch];
}

// Set display tare so current displayed values become zero. reading_10g[ch] = current displayed value.
bool cal_set_display_tare_from_current(const int32_t reading_10g[CHANNELS]) {
  if (!reading_10g) return false;
  display_tare_load();
  for (int ch = 0; ch < CHANNELS; ch++)
    g_display_tare_10g[ch] += reading_10g[ch];
  return display_tare_save();
}


