// ============================================================================
// NOISE FILTERING AND OUTLIER REMOVAL SYSTEM
// ============================================================================
// Provides comprehensive filtering for force plate data including:
// - Outlier detection and removal
// - Multiple filter types (SMA, EMA, Median, Kalman)
// - Adaptive filtering based on signal characteristics
// - Real-time processing with minimal latency
// ============================================================================

#include <math.h>

// Include guard to prevent redefinition
#ifndef NOISE_FILTERING_DEFINED
#define NOISE_FILTERING_DEFINED

// ============================================================================
// FILTER CONFIGURATION AND CONSTANTS
// ============================================================================
// Filter types and structures are defined in main file for Arduino IDE compatibility

// Default filter configurations for different use cases
static const FilterConfig FILTER_CONFIG_REALTIME = {
  .filter_type = FILTER_EMA,
  .outlier_method = OUTLIER_ADAPTIVE,
  .window_size = 5,
  .outlier_threshold = 3.0,
  .ema_alpha = 0.1,
  .gaussian_sigma = 1.0,
  .enable_preprocessing = true,
  .enable_postprocessing = false
};

static const FilterConfig FILTER_CONFIG_HIGH_QUALITY = {
  .filter_type = FILTER_COMBINED,
  .outlier_method = OUTLIER_MAD,
  .window_size = 10,
  .outlier_threshold = 2.5,
  .ema_alpha = 0.05,
  .gaussian_sigma = 1.5,
  .enable_preprocessing = true,
  .enable_postprocessing = true
};

static const FilterConfig FILTER_CONFIG_LOW_NOISE = {
  .filter_type = FILTER_MEDIAN,
  .outlier_method = OUTLIER_IQR,
  .window_size = 7,
  .outlier_threshold = 1.5,
  .ema_alpha = 0.2,
  .gaussian_sigma = 0.8,
  .enable_preprocessing = true,
  .enable_postprocessing = false
};

// Gaussian filter specific preset
static const FilterConfig FILTER_CONFIG_GAUSSIAN = {
  .filter_type = FILTER_GAUSSIAN,
  .outlier_method = OUTLIER_ADAPTIVE,
  .window_size = 7,
  .outlier_threshold = 2.0,
  .ema_alpha = 0.1,
  .gaussian_sigma = 1.2,
  .enable_preprocessing = true,
  .enable_postprocessing = false
};

// ============================================================================
// FILTER DATA STRUCTURES
// ============================================================================
// CircularBuffer and ChannelFilter are defined in main file for Arduino IDE compatibility

// Global filter state
static ChannelFilter channel_filters[4];
static FilterConfig current_filter_config = FILTER_CONFIG_REALTIME;
static bool filtering_enabled = true;
static uint32_t outliers_detected[4] = {0, 0, 0, 0};
static uint32_t samples_processed[4] = {0, 0, 0, 0};

// ============================================================================
// OUTLIER DETECTION FUNCTIONS
// ============================================================================

bool is_outlier_zscore(int32_t value, ChannelFilter& filter, double threshold) {
  if (filter.sample_count < 10) return false; // Need minimum samples
  
  double z_score = fabs(value - filter.running_mean) / sqrt(filter.running_variance);
  return z_score > threshold;
}

bool is_outlier_iqr(int32_t value, ChannelFilter& filter, double threshold) {
  if (!filter.raw_buffer.full()) return false;
  
  // Calculate Q1, Q3 for IQR method
  int32_t sorted[20];
  for (size_t i = 0; i < filter.raw_buffer.size(); i++) {
    sorted[i] = filter.raw_buffer.get(i);
  }
  
  // Simple bubble sort for small array
  size_t n = filter.raw_buffer.size();
  for (size_t i = 0; i < n-1; i++) {
    for (size_t j = 0; j < n-i-1; j++) {
      if (sorted[j] > sorted[j+1]) {
        int32_t temp = sorted[j];
        sorted[j] = sorted[j+1];
        sorted[j+1] = temp;
      }
    }
  }
  
  double q1 = sorted[n/4];
  double q3 = sorted[3*n/4];
  double iqr = q3 - q1;
  
  double lower_bound = q1 - threshold * iqr;
  double upper_bound = q3 + threshold * iqr;
  
  return (value < lower_bound) || (value > upper_bound);
}

bool is_outlier_mad(int32_t value, ChannelFilter& filter, double threshold) {
  if (!filter.raw_buffer.full()) return false;
  
  // Calculate median
  int32_t sorted[20];
  for (size_t i = 0; i < filter.raw_buffer.size(); i++) {
    sorted[i] = filter.raw_buffer.get(i);
  }
  
  size_t n = filter.raw_buffer.size();
  for (size_t i = 0; i < n-1; i++) {
    for (size_t j = 0; j < n-i-1; j++) {
      if (sorted[j] > sorted[j+1]) {
        int32_t temp = sorted[j];
        sorted[j] = sorted[j+1];
        sorted[j+1] = temp;
      }
    }
  }
  
  double median = (n % 2 == 0) ? 
    (sorted[n/2-1] + sorted[n/2]) / 2.0 : 
    sorted[n/2];
  
  // Calculate MAD
  double deviations[20];
  for (size_t i = 0; i < n; i++) {
    deviations[i] = fabs(sorted[i] - median);
  }
  
  // Sort deviations
  for (size_t i = 0; i < n-1; i++) {
    for (size_t j = 0; j < n-i-1; j++) {
      if (deviations[j] > deviations[j+1]) {
        double temp = deviations[j];
        deviations[j] = deviations[j+1];
        deviations[j+1] = temp;
      }
    }
  }
  
  double mad = (n % 2 == 0) ? 
    (deviations[n/2-1] + deviations[n/2]) / 2.0 : 
    deviations[n/2];
  
  filter.running_mad = mad;
  
  // Modified Z-score using MAD
  double modified_z = 0.6745 * fabs(value - median) / mad;
  return modified_z > threshold;
}

bool is_outlier_adaptive(int32_t value, ChannelFilter& filter, double threshold) {
  // Update adaptive thresholds based on recent data
  if (filter.sample_count > 0) {
    double alpha = 0.01; // Adaptation rate
    
    if (value > filter.adaptive_threshold_high) {
      filter.adaptive_threshold_high = filter.adaptive_threshold_high * (1 - alpha) + value * alpha;
    }
    if (value < filter.adaptive_threshold_low) {
      filter.adaptive_threshold_low = filter.adaptive_threshold_low * (1 - alpha) + value * alpha;
    }
    
    // Update noise level estimate
    double deviation = fabs(value - filter.running_mean);
    filter.noise_level = filter.noise_level * (1 - alpha) + deviation * alpha;
  }
  
  double dynamic_threshold = filter.noise_level * threshold;
  double deviation = fabs(value - filter.running_mean);
  
  return deviation > dynamic_threshold;
}

bool detect_outlier(int32_t value, uint8_t channel, const FilterConfig& config) {
  if (channel >= 4) return false;
  
  ChannelFilter& filter = channel_filters[channel];
  
  switch (config.outlier_method) {
    case OUTLIER_ZSCORE:
      return is_outlier_zscore(value, filter, config.outlier_threshold);
      
    case OUTLIER_IQR:
      return is_outlier_iqr(value, filter, config.outlier_threshold);
      
    case OUTLIER_MAD:
      return is_outlier_mad(value, filter, config.outlier_threshold);
      
    case OUTLIER_ADAPTIVE:
      return is_outlier_adaptive(value, filter, config.outlier_threshold);
      
    default:
      return false;
  }
}

// ============================================================================
// FILTER IMPLEMENTATIONS
// ============================================================================

double apply_sma_filter(int32_t value, uint8_t channel, uint8_t window_size) {
  if (channel >= 4) return value;
  
  ChannelFilter& filter = channel_filters[channel];
  filter.raw_buffer.push(value);
  
  if (filter.raw_buffer.size() < window_size) {
    return value; // Not enough samples yet
  }
  
  double sum = 0.0;
  size_t count = min(window_size, (uint8_t)filter.raw_buffer.size());
  
  for (size_t i = 0; i < count; i++) {
    sum += filter.raw_buffer.get(filter.raw_buffer.size() - 1 - i);
  }
  
  return sum / count;
}

double apply_ema_filter(int32_t value, uint8_t channel, double alpha) {
  if (channel >= 4) return value;
  
  ChannelFilter& filter = channel_filters[channel];
  
  if (!filter.ema_initialized) {
    filter.ema_value = value;
    filter.ema_initialized = true;
    return value;
  }
  
  filter.ema_value = alpha * value + (1.0 - alpha) * filter.ema_value;
  return filter.ema_value;
}

double apply_median_filter(int32_t value, uint8_t channel, uint8_t window_size) {
  if (channel >= 4) return value;
  
  ChannelFilter& filter = channel_filters[channel];
  filter.raw_buffer.push(value);
  
  size_t count = min(window_size, (uint8_t)filter.raw_buffer.size());
  if (count < 3) return value; // Need minimum samples for median
  
  int32_t sorted[20];
  for (size_t i = 0; i < count; i++) {
    sorted[i] = filter.raw_buffer.get(filter.raw_buffer.size() - 1 - i);
  }
  
  // Sort array
  for (size_t i = 0; i < count-1; i++) {
    for (size_t j = 0; j < count-i-1; j++) {
      if (sorted[j] > sorted[j+1]) {
        int32_t temp = sorted[j];
        sorted[j] = sorted[j+1];
        sorted[j+1] = temp;
      }
    }
  }
  
  // Return median
  return (count % 2 == 0) ? 
    (sorted[count/2-1] + sorted[count/2]) / 2.0 : 
    sorted[count/2];
}

double apply_kalman_filter(int32_t value, uint8_t channel) {
  if (channel >= 4) return value;
  
  ChannelFilter& filter = channel_filters[channel];
  
  if (!filter.kalman_initialized) {
    filter.kalman_x = value;
    filter.kalman_P = 1000.0;
    filter.kalman_initialized = true;
    return value;
  }
  
  // Prediction step
  double x_pred = filter.kalman_x;
  double P_pred = filter.kalman_P + filter.kalman_Q;
  
  // Update step
  double K = P_pred / (P_pred + filter.kalman_R);
  filter.kalman_x = x_pred + K * (value - x_pred);
  filter.kalman_P = (1.0 - K) * P_pred;
  
  return filter.kalman_x;
}

double apply_combined_filter(int32_t value, uint8_t channel, const FilterConfig& config) {
  if (channel >= 4) return value;
  
  // Stage 1: Median filter for spike removal
  double stage1 = apply_median_filter(value, channel, 5);
  
  // Stage 2: EMA for smoothing
  double stage2 = apply_ema_filter((int32_t)stage1, channel, config.ema_alpha);
  
  // Stage 3: Kalman for optimal estimation
  double stage3 = apply_kalman_filter((int32_t)stage2, channel);
  
  return stage3;
}

// Compute Gaussian weights for given sigma and kernel size
void compute_gaussian_weights(double* weights, uint8_t kernel_size, double sigma) {
  if (kernel_size > 11) kernel_size = 11; // Safety limit
  
  double sum = 0.0;
  int center = kernel_size / 2;
  
  // Compute weights
  for (int i = 0; i < kernel_size; i++) {
    int x = i - center;
    weights[i] = exp(-(x * x) / (2.0 * sigma * sigma));
    sum += weights[i];
  }
  
  // Normalize weights
  for (int i = 0; i < kernel_size; i++) {
    weights[i] /= sum;
  }
}

// Apply Gaussian filter to a channel
double apply_gaussian_filter(int32_t value, uint8_t channel) {
  if (channel >= 4) return value;
  
  ChannelFilter& filter = channel_filters[channel];
  
  // Initialize Gaussian weights if needed
  if (!filter.gaussian_initialized || filter.gaussian_kernel_size != current_filter_config.window_size) {
    filter.gaussian_kernel_size = current_filter_config.window_size;
    if (filter.gaussian_kernel_size > 11) filter.gaussian_kernel_size = 11;
    if (filter.gaussian_kernel_size < 3) filter.gaussian_kernel_size = 3;
    if (filter.gaussian_kernel_size % 2 == 0) filter.gaussian_kernel_size++; // Ensure odd size
    
    compute_gaussian_weights(filter.gaussian_weights, filter.gaussian_kernel_size, current_filter_config.gaussian_sigma);
    filter.gaussian_initialized = true;
  }
  
  // Add new value to buffer
  filter.raw_buffer.push(value);
  
  // Need enough samples for filtering
  if (filter.raw_buffer.size() < filter.gaussian_kernel_size) {
    return (double)value;
  }
  
  // Apply Gaussian convolution
  double filtered_value = 0.0;
  size_t buffer_size = filter.raw_buffer.size();
  
  for (int i = 0; i < filter.gaussian_kernel_size; i++) {
    int buffer_index = (int)buffer_size - filter.gaussian_kernel_size + i;
    if (buffer_index >= 0 && buffer_index < (int)buffer_size) {
      double sample = (double)filter.raw_buffer.get(buffer_index);
      filtered_value += sample * filter.gaussian_weights[i];
    }
  }
  
  return filtered_value;
}

// ============================================================================
// MAIN FILTERING FUNCTION
// ============================================================================

double apply_noise_filter(int32_t raw_value, uint8_t channel) {
  if (!filtering_enabled || channel >= 4) {
    return raw_value;
  }
  
  ChannelFilter& filter = channel_filters[channel];
  samples_processed[channel]++;
  
  // Update running statistics
  if (filter.sample_count == 0) {
    filter.running_mean = raw_value;
    filter.running_variance = 0.0;
  } else {
    double delta = raw_value - filter.running_mean;
    filter.running_mean += delta / (filter.sample_count + 1);
    filter.running_variance += delta * (raw_value - filter.running_mean);
    if (filter.sample_count > 1) {
      filter.running_variance /= filter.sample_count;
    }
  }
  filter.sample_count++;
  
  // Outlier detection and replacement
  int32_t processed_value = raw_value;
  if (detect_outlier(raw_value, channel, current_filter_config)) {
    outliers_detected[channel]++;
    
    // Replace outlier with last good value or running mean
    if (filter.filtered_buffer.size() > 0) {
      processed_value = (int32_t)filter.filtered_buffer.newest();
    } else {
      processed_value = (int32_t)filter.running_mean;
    }
  }
  
  // Apply selected filter
  double filtered_value;
  
  switch (current_filter_config.filter_type) {
    case FILTER_SMA:
      filtered_value = apply_sma_filter(processed_value, channel, current_filter_config.window_size);
      break;
      
    case FILTER_EMA:
      filtered_value = apply_ema_filter(processed_value, channel, current_filter_config.ema_alpha);
      break;
      
    case FILTER_MEDIAN:
      filtered_value = apply_median_filter(processed_value, channel, current_filter_config.window_size);
      break;
      
    case FILTER_KALMAN:
      filtered_value = apply_kalman_filter(processed_value, channel);
      break;
      
    case FILTER_COMBINED:
      filtered_value = apply_combined_filter(processed_value, channel, current_filter_config);
      break;
      
    case FILTER_GAUSSIAN:
      filtered_value = apply_gaussian_filter(processed_value, channel);
      break;
      
    case FILTER_ADAPTIVE:
      // Adaptive filter - choose best filter based on signal characteristics
      if (filter.noise_level > 500.0) {
        filtered_value = apply_median_filter(processed_value, channel, 7);
      } else if (filter.noise_level > 100.0) {
        filtered_value = apply_ema_filter(processed_value, channel, 0.1);
      } else {
        filtered_value = apply_kalman_filter(processed_value, channel);
      }
      break;
      
    default:
      filtered_value = processed_value;
      break;
  }
  
  // Store filtered value
  filter.filtered_buffer.push(filtered_value);
  
  return filtered_value;
}

// ============================================================================
// FILTER CONTROL FUNCTIONS
// ============================================================================

void set_filter_config(const FilterConfig& config) {
  current_filter_config = config;
  Serial.printf("[FILTER] Configuration updated: Type=%d, Outlier=%d, Window=%d\n",
                config.filter_type, config.outlier_method, config.window_size);
}

void set_filter_type(int type) {
  current_filter_config.filter_type = (FilterType)type;
  Serial.printf("[FILTER] Filter type set to: %d\n", type);
}

void set_outlier_method(int method) {
  current_filter_config.outlier_method = (OutlierMethod)method;
  Serial.printf("[FILTER] Outlier method set to: %d\n", method);
}

void enable_filtering(bool enable) {
  filtering_enabled = enable;
  Serial.printf("[FILTER] Filtering %s\n", enable ? "ENABLED" : "DISABLED");
}

void reset_filters() {
  for (int i = 0; i < 4; i++) {
    channel_filters[i].reset();
    outliers_detected[i] = 0;
    samples_processed[i] = 0;
  }
  Serial.println("[FILTER] All filters reset");
}

void show_filter_status() {
  Serial.println("[FILTER] ==========================================");
  Serial.println("[FILTER] NOISE FILTERING STATUS");
  Serial.println("[FILTER] ==========================================");
  
  const char* filter_names[] = {"None", "SMA", "EMA", "Median", "Kalman", "Adaptive", "Combined", "Gaussian"};
  const char* outlier_names[] = {"None", "Z-Score", "IQR", "MAD", "Adaptive"};
  
  Serial.printf("[FILTER] Filtering: %s\n", filtering_enabled ? "ENABLED" : "DISABLED");
  Serial.printf("[FILTER] Filter Type: %s\n", filter_names[current_filter_config.filter_type]);
  Serial.printf("[FILTER] Outlier Method: %s\n", outlier_names[current_filter_config.outlier_method]);
  Serial.printf("[FILTER] Window Size: %d\n", current_filter_config.window_size);
  Serial.printf("[FILTER] Outlier Threshold: %.2f\n", current_filter_config.outlier_threshold);
  Serial.printf("[FILTER] EMA Alpha: %.3f\n", current_filter_config.ema_alpha);
  Serial.printf("[FILTER] Gaussian Sigma: %.2f\n", current_filter_config.gaussian_sigma);
  
  Serial.println("[FILTER] ");
  Serial.println("[FILTER] Per-Channel Statistics:");
  for (int i = 0; i < 4; i++) {
    ChannelFilter& filter = channel_filters[i];
    double outlier_rate = (samples_processed[i] > 0) ? 
      (double)outliers_detected[i] / samples_processed[i] * 100.0 : 0.0;
    
    Serial.printf("[FILTER] CH%d: Samples=%lu, Outliers=%lu (%.2f%%), Mean=%.1f, Noise=%.1f\n",
                  i + 1, samples_processed[i], outliers_detected[i], outlier_rate,
                  filter.running_mean, filter.noise_level);
  }
  
  Serial.println("[FILTER] ==========================================");
}

// ============================================================================
// PRESET CONFIGURATIONS
// ============================================================================

void set_realtime_filtering() {
  set_filter_config(FILTER_CONFIG_REALTIME);
  Serial.println("[FILTER] Real-time filtering preset applied");
}

void set_high_quality_filtering() {
  set_filter_config(FILTER_CONFIG_HIGH_QUALITY);
  Serial.println("[FILTER] High-quality filtering preset applied");
}

void set_low_noise_filtering() {
  set_filter_config(FILTER_CONFIG_LOW_NOISE);
  Serial.println("[FILTER] Low-noise filtering preset applied");
}

void set_gaussian_filtering() {
  set_filter_config(FILTER_CONFIG_GAUSSIAN);
  Serial.println("[FILTER] Gaussian filtering preset applied");
}

void set_gaussian_sigma(double sigma) {
  if (sigma > 0.0 && sigma <= 5.0) {
    current_filter_config.gaussian_sigma = sigma;
    
    // Reset Gaussian initialization for all channels to use new sigma
    for (int i = 0; i < 4; i++) {
      channel_filters[i].gaussian_initialized = false;
    }
    
    Serial.printf("[FILTER] Gaussian sigma set to: %.2f\n", sigma);
  } else {
    Serial.println("[FILTER] Invalid sigma value (must be 0.1-5.0)");
  }
}

// ============================================================================
// INTEGRATION WITH EXISTING SYSTEM
// ============================================================================

// Tare offset application flag - when true, streamed values are tare-corrected
static bool apply_tare_offset_to_stream = true;

// External calibration accessors (defined in calibration_regression.cpp)
extern void cal_get_status(CalStatus* out);
extern int8_t cal_get_polarity(uint8_t ch);
extern float cal_get_slope(uint8_t ch);

// Get the tare offset for a channel from calibration system
static int32_t get_tare_offset(uint8_t channel) {
  if (channel >= 4) return 0;
  
  CalStatus status;
  cal_get_status(&status);
  return status.offsets[channel];
}

// Control whether tare offset is applied to streaming data
void set_tare_offset_enabled(bool enable) {
  apply_tare_offset_to_stream = enable;
  Serial.printf("[FILTER] Tare offset application: %s\n", enable ? "ENABLED" : "DISABLED");
}

bool is_tare_offset_enabled() {
  return apply_tare_offset_to_stream;
}

// Get RAW filtered reading WITHOUT tare offset or polarity correction
// This is used by calibration functions that need raw ADC values with optional filtering
int32_t get_raw_filtered_reading(uint8_t channel) {
  if (channel >= 4) return 0;
  
  // Get raw reading from actual hardware
  int32_t raw_value = read_single_channel_fast(channel);
  
  // Apply filtering only (NO tare offset, NO polarity correction)
  double filtered_value = apply_noise_filter(raw_value, channel);
  
  return (int32_t)filtered_value;
}

// Modified function to apply filtering to load cell readings
// Now also subtracts tare offset and applies polarity correction
// so all channels show positive values when weight is applied
// Returns: tare/polarity corrected delta counts (NOT calibrated weight)
int32_t get_filtered_load_cell_reading(uint8_t channel) {
  if (channel >= 4) return 0;
  
  // Check if mock data is enabled (external function from mock_data_generator.ino)
  extern bool mock_data_is_enabled();
  extern int32_t mock_data_generate(uint8_t channel);
  
  if (mock_data_is_enabled()) {
    // Return mock data directly (no filtering needed for mock data).
    // Note: When mock data is enabled, tare has no effect on streamed values (this path
    // bypasses tare offset and polarity). Turn mock mode off when verifying tare behavior.
    return mock_data_generate(channel);
  }
  
  // Get raw reading from actual hardware
  int32_t raw_value = read_single_channel_fast(channel);
  
  // Apply filtering
  double filtered_value = apply_noise_filter(raw_value, channel);
  
  // Apply tare offset to get values near zero after calibration
  // This ensures streamed data reflects tare-corrected readings
  if (apply_tare_offset_to_stream) {
    int32_t offset = get_tare_offset(channel);
    filtered_value -= (double)offset;
    
    // Apply polarity correction (auto-detected during CAL_ADD)
    // This ensures all channels show positive values when weight is applied
    int8_t polarity = cal_get_polarity(channel);
    filtered_value *= (double)polarity;
  }
  
  return (int32_t)filtered_value;
}

// Get CALIBRATED load cell reading in 10g units (1 kg = 100 units)
// This applies the calibration slope to convert delta counts to actual weight
// Returns: weight in 10g units, clamped to int16 range for BLE streaming
int32_t get_calibrated_load_cell_reading(uint8_t channel) {
  if (channel >= 4) return 0;
  
  // Get tare/polarity corrected delta counts
  int32_t delta_counts = get_filtered_load_cell_reading(channel);
  
  // Apply calibration slope to convert counts to 10g units
  // slope 'a' is calculated during CAL_ADD such that: weight_10g = |delta_counts| * a
  float slope = cal_get_slope(channel);
  
  int32_t weight_10g;
  
  // If not calibrated yet (slope = 0), scale down raw counts to fit in int16 range
  if (slope <= 0.0f || slope > 1.0f) {
    // Slope should be a small positive number (typically 0.001 to 0.1)
    // If it's 0, negative, or > 1, calibration is invalid - use fallback scaling
    // Scale raw counts to roughly fit in int16 range for 100kg max total
    // Typical raw delta for full load might be ~500,000 counts
    // We want ~10,000 units max (100kg), so divide by ~50
    weight_10g = delta_counts / 50;
  } else {
    // Apply calibration: weight_10g = |delta_counts| * slope
    // Note: delta_counts is already polarity-corrected, so it should be positive when loaded
    // We use fabs just in case there's any residual negative from noise
    weight_10g = (int32_t)lroundf(fabs((float)delta_counts) * slope);
  }
  
  // Clamp to int16 range to prevent overflow in BLE transmission
  // Max expected: 100kg = 10,000 units, well within int16 range
  // If we're hitting this clamp, calibration is likely incorrect
  if (weight_10g > 32767) {
    weight_10g = 32767;
  } else if (weight_10g < -32768) {
    weight_10g = -32768;
  }
  
  // Display-space tare: subtract so "after tare, displayed = 0" (for pre-calibrated load cells)
  weight_10g -= cal_get_display_tare_10g(channel);

  return weight_10g;
}

// Get both raw and filtered values for comparison
// FilteredReading structure is defined in main file
// Note: raw value is the ADC reading, filtered value has tare offset and polarity applied

FilteredReading get_filtered_reading_with_info(uint8_t channel) {
  FilteredReading result = {0, 0, false};
  
  if (channel >= 4) return result;
  
  // Get raw reading
  result.raw = read_single_channel_fast(channel);
  
  // Check for outlier
  result.outlier_detected = detect_outlier(result.raw, channel, current_filter_config);
  
  // Apply filtering
  double filtered_value = apply_noise_filter(result.raw, channel);
  
  // Apply tare offset and polarity to filtered value for consistency with streaming
  if (apply_tare_offset_to_stream) {
    int32_t offset = get_tare_offset(channel);
    filtered_value -= (double)offset;
    
    // Apply polarity correction (auto-detected during CAL_ADD)
    int8_t polarity = cal_get_polarity(channel);
    filtered_value *= (double)polarity;
  }
  
  result.filtered = (int32_t)filtered_value;
  
  return result;
}

#endif // NOISE_FILTERING_DEFINED
