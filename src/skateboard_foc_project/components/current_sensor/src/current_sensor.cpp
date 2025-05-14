#include "current_sensor.hpp"
#include "driver/adc.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "ESP32_CONT_CS";
static adc_continuous_handle_t cont_handle = NULL;
static esp_adc_cal_characteristics_t adc_chars;
#define SAMPLE_FREQ_HZ 20000
#define CONV_FRAME_SIZE 256
static uint8_t result_buffer[CONV_FRAME_SIZE];

// Add ADC conversion constants for current calculation
#define ADC_MAX_COUNT 4095.0f
#define ADC_REF_VOLTAGE_V 3.3f

// Kconfig-based zero-current voltage reference
#define ZERO_REF_VOLTAGE_V (CONFIG_CURRENT_ZERO_REFERENCE_MV / 1000.0f)

ESP32InlineCurrentSense::ESP32InlineCurrentSense(float shunt_resistor,
                                                 float gain, int pinA_,
                                                 int pinB_, int pinC_)
    : pinA(pinA_), pinB(pinB_), pinC(pinC_) {
  // Apply zero-current offset from SDKconfig if provided
#if CONFIG_CURRENT_ZERO_REFERENCE_MV
  offset_ia = offset_ib = offset_ic = ZERO_REF_VOLTAGE_V;
#else
  offset_ia = offset_ib = offset_ic = 0.0f;
#endif
  // Calculate volts-to-amps ratio from shunt resistor and amplifier gain
  volts_to_amps_ratio = 1.0f / (shunt_resistor * gain);
  gain_a = gain_b = gain_c = volts_to_amps_ratio;
}

ESP32InlineCurrentSense::ESP32InlineCurrentSense(float mVpA, int pinA_,
                                                 int pinB_, int pinC_)
    : pinA(pinA_), pinB(pinB_), pinC(pinC_) {
  // Apply zero-current offset from SDKconfig if provided
#if CONFIG_CURRENT_ZERO_REFERENCE_MV
  offset_ia = offset_ib = offset_ic = ZERO_REF_VOLTAGE_V;
#else
  offset_ia = offset_ib = offset_ic = 0.0f;
#endif
  // Calculate volts-to-amps ratio from mV/A specification
  volts_to_amps_ratio = 1000.0f / mVpA;
  gain_a = gain_b = gain_c = volts_to_amps_ratio;
}

void ESP32InlineCurrentSense::calibrateOffsets() {
  const int num_samples = 64;
  int countA = 0, countB = 0, countC = 0;
  float sumA = 0, sumB = 0, sumC = 0;
  uint32_t ret_num = 0;
  for (int i = 0; i < num_samples; ++i) {
    if (adc_continuous_read(cont_handle, result_buffer, CONV_FRAME_SIZE,
                            &ret_num, 100) != ESP_OK) {
      continue;
    }
    for (int idx = 0; idx < ret_num; idx += SOC_ADC_DIGI_RESULT_BYTES) {
      adc_digi_output_data_t *out =
          (adc_digi_output_data_t *)&result_buffer[idx];
      int ch = out->type2.channel;
      int raw = out->type2.data;
      uint32_t mV = esp_adc_cal_raw_to_voltage(raw, &adc_chars);
      if (ch == pinA) {
        sumA += mV;
        countA++;
      }
      if (ch == pinB) {
        sumB += mV;
        countB++;
      }
      if (ch == pinC) {
        sumC += mV;
        countC++;
      }
    }
  }
  if (countA > 0)
    offset_ia = (sumA / countA) / 1000.0f;
  if (countB > 0)
    offset_ib = (sumB / countB) / 1000.0f;
  if (countC > 0)
    offset_ic = (sumC / countC) / 1000.0f;
  ESP_LOGI(TAG,
           "Current sensor offsets calibrated: A=%.3f V (%d samples), B=%.3f V "
           "(%d samples), C=%.3f V (%d samples)",
           offset_ia, countA, offset_ib, countB, offset_ic, countC);
}

int ESP32InlineCurrentSense::init() {
  // Must have at least two phases configured
  if (pinA < 0 && pinB < 0) {
    ESP_LOGE(TAG, "At least two current sensing pins must be configured");
    return 0;
  }
  // Setup continuous ADC driver once
  if (cont_handle == NULL) {
    // Characterize ADC for calibration
    esp_adc_cal_value_t cal_type = esp_adc_cal_characterize(
        ADC_UNIT_1, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_12, 0, &adc_chars);
    ESP_LOGI(TAG, "ADC Calibration Type: %d", cal_type);
    // Continuous driver handle
    adc_continuous_handle_cfg_t handle_cfg = {
        .max_store_buf_size = CONV_FRAME_SIZE,
        .conv_frame_size = CONV_FRAME_SIZE,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_cfg, &cont_handle));
    // Setup conversion parameters
    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = SAMPLE_FREQ_HZ,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };
    // Build channel patterns
    adc_digi_pattern_config_t patterns[3] = {};
    int patt_count = 0;
    if (pinA >= 0) {
      patterns[patt_count].atten = ADC_ATTEN_DB_0;
      patterns[patt_count].channel = pinA & 0x7;
      patterns[patt_count].unit = ADC_UNIT_1;
      patterns[patt_count].bit_width = ADC_BITWIDTH_12;
      patt_count++;
    }
    if (pinB >= 0) {
      patterns[patt_count].atten = ADC_ATTEN_DB_0;
      patterns[patt_count].channel = pinB & 0x7;
      patterns[patt_count].unit = ADC_UNIT_1;
      patterns[patt_count].bit_width = ADC_BITWIDTH_12;
      patt_count++;
    }
    if (pinC >= 0) {
      patterns[patt_count].atten = ADC_ATTEN_DB_0;
      patterns[patt_count].channel = pinC & 0x7;
      patterns[patt_count].unit = ADC_UNIT_1;
      patterns[patt_count].bit_width = ADC_BITWIDTH_12;
      patt_count++;
    }
    dig_cfg.pattern_num = patt_count;
    dig_cfg.adc_pattern = patterns;
    ESP_ERROR_CHECK(adc_continuous_config(cont_handle, &dig_cfg));
    ESP_ERROR_CHECK(adc_continuous_start(cont_handle));
  }
  // Calibrate zero-offsets using continuous driver
  calibrateOffsets();
  ESP_LOGI(TAG, "InlineCurrentSense initialized for pins A=%d, B=%d, C=%d",
           pinA, pinB, pinC);
  return 1;
}

PhaseCurrent_s ESP32InlineCurrentSense::getPhaseCurrents() {
  PhaseCurrent_s curr = {0, 0, 0};
  uint32_t ret_num = 0;
  // Read a batch of samples
  if (adc_continuous_read(cont_handle, result_buffer, CONV_FRAME_SIZE, &ret_num,
                          100) == ESP_OK) {
    // pick the last values per channel
    for (int idx = ret_num - SOC_ADC_DIGI_RESULT_BYTES; idx >= 0;
         idx -= SOC_ADC_DIGI_RESULT_BYTES) {
      adc_digi_output_data_t *out =
          (adc_digi_output_data_t *)&result_buffer[idx];
      int ch = out->type2.channel;
      int raw = out->type2.data;
      float voltage = (esp_adc_cal_raw_to_voltage(raw, &adc_chars) / 1000.0f);
      if (ch == pinA)
        curr.a = (voltage - offset_ia) * gain_a;
      if (ch == pinB)
        curr.b = (voltage - offset_ib) * gain_b;
      if (ch == pinC) {
        curr.c = (voltage - offset_ic) * gain_c;
        break; // we have all values
      }
    }
    // If C is missing, compute from A and B
    if (pinC < 0 && pinA >= 0 && pinB >= 0) {
      curr.c = -(curr.a + curr.b);
    }
  } else {
    ESP_LOGW(TAG, "Continuous ADC read failed");
  }
  return curr;
}

int ESP32InlineCurrentSense::driverAlign(float align_voltage) {
  // No hardware-specific alignment needed for simple inline sense
  // This could be implemented if needed for specific hardware
  ESP_LOGI(TAG, "Driver align with voltage %.2f V", align_voltage);
  return 1;
}
