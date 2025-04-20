#include "esp_log.h"
#include "hx711.hpp"
#include "pressure_sensor.h"
#include "pressure_sensor.hpp"
#include <cmath> // For NAN

static const char *TAG = "PRESSURE_SENSOR_C";

struct pressure_sensor_t {
  HX711 hx711;
  PressureSensor sensor;

  pressure_sensor_t(gpio_num_t dout_pin, gpio_num_t sck_pin, HX711::Mode mode)
      : hx711(sck_pin, dout_pin, mode), sensor(hx711) {}
};

esp_err_t pressure_sensor_init(const pressure_sensor_config_t *config,
                               hx711_handle_t *handle) {
  if (config == nullptr || handle == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }

  HX711::Mode gain_mode;
  switch (config->gain) {
  case HX711_GAIN_128_A:
    gain_mode = HX711::Mode::kChannelA128;
    break;
  case HX711_GAIN_64_B:
    gain_mode = HX711::Mode::kChannelA64; // Corrected enum
    break;
  case HX711_GAIN_32_A:
    gain_mode = HX711::Mode::kChannelB32; // Corrected enum
    break;
  default:
    return ESP_ERR_INVALID_ARG;
  }

  // Create new pressure sensor instance with error handling
  pressure_sensor_t *sensor = nullptr;

  // Use a safer approach without exceptions
  sensor = new (std::nothrow)
      pressure_sensor_t(config->dout_gpio, config->sck_gpio, gain_mode);
  if (sensor == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate memory for pressure sensor");
    return ESP_ERR_NO_MEM;
  }

  // Apply initial calibration if provided
  if (config->offset != 0.0f || config->scale != 1.0f) {
    sensor->sensor.SetCalibration(config->offset, config->scale);
  }

  *handle = sensor;
  return ESP_OK;
}

float pressure_sensor_get_weight(hx711_handle_t handle) {
  if (handle == nullptr) {
    return NAN;
  }

  return handle->sensor.GetWeight(5); // Use 5 samples by default
}

esp_err_t pressure_sensor_tare(hx711_handle_t handle, int samples) {
  if (handle == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }

  bool success = handle->sensor.Tare(samples);
  return success ? ESP_OK : ESP_FAIL;
}

esp_err_t pressure_sensor_calibrate(hx711_handle_t handle, float known_weight,
                                    int samples) {
  if (handle == nullptr || known_weight <= 0) {
    return ESP_ERR_INVALID_ARG;
  }

  bool success = handle->sensor.Calibrate(known_weight, samples);
  return success ? ESP_OK : ESP_FAIL;
}

esp_err_t pressure_sensor_save_calibration(hx711_handle_t handle,
                                           const char *namespace_name,
                                           const char *key_prefix) {
  if (handle == nullptr || namespace_name == nullptr || key_prefix == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }

  bool success = handle->sensor.SaveCalibration(namespace_name, key_prefix);
  return success ? ESP_OK : ESP_FAIL;
}

esp_err_t pressure_sensor_load_calibration(hx711_handle_t handle,
                                           const char *namespace_name,
                                           const char *key_prefix) {
  if (handle == nullptr || namespace_name == nullptr || key_prefix == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }

  bool success = handle->sensor.LoadCalibration(namespace_name, key_prefix);
  return success ? ESP_OK : ESP_FAIL;
}

esp_err_t pressure_sensor_deinit(hx711_handle_t handle) {
  if (handle == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }

  delete handle;
  return ESP_OK;
}