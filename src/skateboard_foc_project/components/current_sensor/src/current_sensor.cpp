#include "current_sensor.hpp"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

static const char *TAG = "ESP32_INLINE_CS";
static adc_oneshot_unit_handle_t adc_handle = NULL;

ESP32InlineCurrentSense::ESP32InlineCurrentSense(float shunt_resistor,
                                                 float gain, int pinA_,
                                                 int pinB_, int pinC_)
    : pinA(pinA_), pinB(pinB_), pinC(pinC_) {
  // Calculate volts to amps ratio from shunt resistor and gain values
  volts_to_amps_ratio = 1.0f / (shunt_resistor * gain);
  gain_a = volts_to_amps_ratio;
  gain_b = volts_to_amps_ratio;
  gain_c = volts_to_amps_ratio;
  offset_ia = 0;
  offset_ib = 0;
  offset_ic = 0;
}

ESP32InlineCurrentSense::ESP32InlineCurrentSense(float mVpA, int pinA_,
                                                 int pinB_, int pinC_)
    : pinA(pinA_), pinB(pinB_), pinC(pinC_) {
  // Calculate volts to amps ratio from mV/A specification
  volts_to_amps_ratio = 1000.0f / mVpA;
  gain_a = volts_to_amps_ratio;
  gain_b = volts_to_amps_ratio;
  gain_c = volts_to_amps_ratio;
  offset_ia = 0;
  offset_ib = 0;
  offset_ic = 0;
}

void ESP32InlineCurrentSense::calibrateOffsets() {
  const int num_samples = 64; // Number of samples to average for offset
  int sum_a = 0, sum_b = 0, sum_c = 0;
  int raw = 0;
  int count_a = 0, count_b = 0, count_c = 0;

  // Sample phase A
  if (pinA >= 0) {
    for (int i = 0; i < num_samples; i++) {
      if (adc_oneshot_read(adc_handle, (adc_channel_t)pinA, &raw) == ESP_OK) {
        sum_a += raw;
        count_a++;
      }
    }
    if (count_a > 0) {
      offset_ia = (float)sum_a / count_a;
    }
  }

  // Sample phase B
  if (pinB >= 0) {
    for (int i = 0; i < num_samples; i++) {
      if (adc_oneshot_read(adc_handle, (adc_channel_t)pinB, &raw) == ESP_OK) {
        sum_b += raw;
        count_b++;
      }
    }
    if (count_b > 0) {
      offset_ib = (float)sum_b / count_b;
    }
  }

  // Sample phase C (if available)
  if (pinC >= 0) {
    for (int i = 0; i < num_samples; i++) {
      if (adc_oneshot_read(adc_handle, (adc_channel_t)pinC, &raw) == ESP_OK) {
        sum_c += raw;
        count_c++;
      }
    }
    if (count_c > 0) {
      offset_ic = (float)sum_c / count_c;
    }
  }

  ESP_LOGI(TAG,
           "Current sensor offsets calibrated: A=%.2f (%d samples), B=%.2f (%d "
           "samples), C=%.2f (%d samples)",
           offset_ia, count_a, offset_ib, count_b, offset_ic, count_c);
}

int ESP32InlineCurrentSense::init() {
  // Check if we have at least two pins configured for current sensing
  if (pinA < 0 && pinB < 0) {
    ESP_LOGE(TAG, "At least two current sensing pins must be configured");
    return 0;
  }

  // Initialize ADC if not already done
  if (adc_handle == NULL) {
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));

    // Configure ADC channels
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_0,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    // Configure ADC channels for each phase sensor that is connected
    if (pinA >= 0) {
      ESP_ERROR_CHECK(adc_oneshot_config_channel(
          adc_handle, (adc_channel_t)pinA, &chan_cfg));
    }

    if (pinB >= 0) {
      ESP_ERROR_CHECK(adc_oneshot_config_channel(
          adc_handle, (adc_channel_t)pinB, &chan_cfg));
    }

    if (pinC >= 0) {
      ESP_ERROR_CHECK(adc_oneshot_config_channel(
          adc_handle, (adc_channel_t)pinC, &chan_cfg));
    }
  }

  // Calibrate the offsets by averaging multiple readings
  calibrateOffsets();

  ESP_LOGI(TAG, "InlineCurrentSense initialized for pins A=%d, B=%d, C=%d",
           pinA, pinB, pinC);

  return 1;
}

PhaseCurrent_s ESP32InlineCurrentSense::getPhaseCurrents() {
  PhaseCurrent_s curr = {0, 0, 0};
  int raw = 0;
  esp_err_t err;

  // Read phase A if available
  if (pinA >= 0) {
    err = adc_oneshot_read(adc_handle, (adc_channel_t)pinA, &raw);
    if (err == ESP_OK) {
      curr.a = (raw - offset_ia) * gain_a;
    } else {
      ESP_LOGW(TAG, "Failed to read phase A current: %s", esp_err_to_name(err));
    }
  }

  // Read phase B if available
  if (pinB >= 0) {
    err = adc_oneshot_read(adc_handle, (adc_channel_t)pinB, &raw);
    if (err == ESP_OK) {
      curr.b = (raw - offset_ib) * gain_b;
    } else {
      ESP_LOGW(TAG, "Failed to read phase B current: %s", esp_err_to_name(err));
    }
  }

  // Read phase C if available, otherwise calculate from A and B
  if (pinC >= 0) {
    err = adc_oneshot_read(adc_handle, (adc_channel_t)pinC, &raw);
    if (err == ESP_OK) {
      curr.c = (raw - offset_ic) * gain_c;
    } else {
      ESP_LOGW(TAG, "Failed to read phase C current: %s", esp_err_to_name(err));
      // If phase C reading fails but we have A and B, calculate C
      if (pinA >= 0 && pinB >= 0) {
        curr.c = -(curr.a + curr.b);
      }
    }
  } else if (pinA >= 0 && pinB >= 0) {
    // In a balanced three-phase system, ia + ib + ic = 0
    // Therefore, ic = -(ia + ib)
    curr.c = -(curr.a + curr.b);
  }

  return curr;
}

int ESP32InlineCurrentSense::driverAlign(float align_voltage) {
  // No hardware-specific alignment needed for simple inline sense
  // This could be implemented if needed for specific hardware
  ESP_LOGI(TAG, "Driver align with voltage %.2f V", align_voltage);
  return 1;
}
