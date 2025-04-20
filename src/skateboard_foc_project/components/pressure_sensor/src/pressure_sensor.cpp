#include "pressure_sensor.hpp"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <math.h>
#include <string>

static const char *TAG = "PressureSensor";

PressureSensor::PressureSensor(HX711 &hx711)
    : hx711_(hx711), offset_(0.0f), scale_factor_(1.0f) {}

bool PressureSensor::Tare(int samples, TickType_t timeout) {
  int32_t zero_point = ReadAverage(samples, timeout);
  if (zero_point == 0) {
    return false;
  }

  offset_ = zero_point;
  ESP_LOGI(TAG, "Zero point calibration successful. Offset: %.2f", offset_);
  return true;
}

bool PressureSensor::Calibrate(float known_weight, int samples,
                               TickType_t timeout) {
  if (known_weight <= 0) {
    ESP_LOGE(TAG, "Known weight must be greater than zero for calibration");
    return false;
  }

  int32_t measured_value = ReadAverage(samples, timeout);
  if (measured_value == 0) {
    return false;
  }

  float scale = (measured_value - offset_) / known_weight;
  if (scale == 0) {
    ESP_LOGE(TAG, "Calibration error: scale factor is zero");
    return false;
  }

  scale_factor_ = scale;
  ESP_LOGI(TAG, "Calibration successful. Scale factor: %.2f", scale_factor_);
  return true;
}

float PressureSensor::GetWeight(int samples, TickType_t timeout) {
  if (scale_factor_ == 0) {
    ESP_LOGW(TAG, "Scale factor is zero. Weight measurement invalid.");
    return NAN;
  }

  int32_t value = ReadAverage(samples, timeout);
  if (value == 0) {
    return NAN;
  }

  return (value - offset_) / scale_factor_;
}

bool PressureSensor::SaveCalibration(const char *namespace_name,
                                     const char *key_prefix) {
  if (namespace_name == nullptr || key_prefix == nullptr) {
    ESP_LOGE(TAG, "Invalid namespace or key prefix");
    return false;
  }

  nvs_handle_t nvs_handle;
  esp_err_t err = nvs_open(namespace_name, NVS_READWRITE, &nvs_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to open NVS namespace: %s", esp_err_to_name(err));
    return false;
  }

  char offset_key[64];
  char scale_key[64];

  snprintf(offset_key, sizeof(offset_key), "%s_offset", key_prefix);
  snprintf(scale_key, sizeof(scale_key), "%s_scale", key_prefix);

  // Save offset as blob
  err = nvs_set_blob(nvs_handle, offset_key, &offset_, sizeof(float));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to save offset: %s", esp_err_to_name(err));
    nvs_close(nvs_handle);
    return false;
  }

  // Save scale factor as blob
  err = nvs_set_blob(nvs_handle, scale_key, &scale_factor_, sizeof(float));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to save scale factor: %s", esp_err_to_name(err));
    nvs_close(nvs_handle);
    return false;
  }

  err = nvs_commit(nvs_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to commit NVS data: %s", esp_err_to_name(err));
    nvs_close(nvs_handle);
    return false;
  }

  nvs_close(nvs_handle);
  ESP_LOGI(TAG, "Calibration data saved to NVS");
  return true;
}

bool PressureSensor::LoadCalibration(const char *namespace_name,
                                     const char *key_prefix) {
  if (namespace_name == nullptr || key_prefix == nullptr) {
    ESP_LOGE(TAG, "Invalid namespace or key prefix");
    return false;
  }

  nvs_handle_t nvs_handle;
  esp_err_t err = nvs_open(namespace_name, NVS_READONLY, &nvs_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to open NVS namespace: %s", esp_err_to_name(err));
    return false;
  }

  char offset_key[64];
  char scale_key[64];

  snprintf(offset_key, sizeof(offset_key), "%s_offset", key_prefix);
  snprintf(scale_key, sizeof(scale_key), "%s_scale", key_prefix);

  float offset = 0.0f;
  float scale = 0.0f;
  bool success = true;
  size_t size = sizeof(float);

  // Read offset as blob
  err = nvs_get_blob(nvs_handle, offset_key, &offset, &size);
  if (err != ESP_OK || size != sizeof(float)) {
    ESP_LOGW(TAG, "Failed to read offset: %s, checking legacy format",
             esp_err_to_name(err));
    if (!LoadLegacyCalibration(nvs_handle, key_prefix)) {
      success = false;
    }
  } else {
    // Read scale factor as blob
    size = sizeof(float);
    err = nvs_get_blob(nvs_handle, scale_key, &scale, &size);
    if (err != ESP_OK || size != sizeof(float)) {
      ESP_LOGE(TAG, "Failed to read scale factor: %s", esp_err_to_name(err));
      success = false;
    } else {
      // Successfully read both values
      offset_ = offset;
      scale_factor_ = scale;
      ESP_LOGI(TAG, "Loaded calibration - Offset: %.2f, Scale factor: %.2f",
               offset_, scale_factor_);
    }
  }

  nvs_close(nvs_handle);
  return success;
}

bool PressureSensor::LoadLegacyCalibration(nvs_handle_t nvs_handle,
                                           const char *key_prefix) {
  ESP_LOGI(TAG, "Attempting to load legacy calibration format");

  char legacy_key[64];
  snprintf(legacy_key, sizeof(legacy_key), "%s_cal", key_prefix);

  size_t required_size = 0;

  // Check if legacy calibration data exists
  esp_err_t err = nvs_get_blob(nvs_handle, legacy_key, nullptr, &required_size);
  if (err != ESP_OK || required_size != 2 * sizeof(float)) {
    ESP_LOGE(TAG, "Legacy calibration data not found or invalid");
    return false;
  }

  // Load legacy data
  float cal_data[2];
  err = nvs_get_blob(nvs_handle, legacy_key, cal_data, &required_size);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to load legacy calibration data: %s",
             esp_err_to_name(err));
    return false;
  }

  offset_ = cal_data[0];
  scale_factor_ = cal_data[1];

  ESP_LOGI(TAG, "Loaded legacy calibration - Offset: %.2f, Scale factor: %.2f",
           offset_, scale_factor_);

  // Migrate legacy data to new format (in a separate function call to
  // SaveCalibration)
  ESP_LOGI(TAG, "Migrating legacy calibration data to new format");

  return true;
}

int32_t PressureSensor::ReadAverage(int samples, TickType_t timeout) {
  int32_t sum = 0;
  int valid_samples = 0;

  for (int i = 0; i < samples; i++) {
    int32_t value = hx711_.Read(timeout);
    if (value != 0) {
      sum += value;
      valid_samples++;
    }
  }

  if (valid_samples == 0) {
    ESP_LOGE(TAG, "No valid samples read from HX711");
    return 0;
  }

  return sum / valid_samples;
}

void PressureSensor::SetCalibration(float offset, float scale_factor) {
  offset_ = offset;
  scale_factor_ = scale_factor;
}