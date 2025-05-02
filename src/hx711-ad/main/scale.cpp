#include "scale.hpp"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <math.h>

static const char *kTag = "Scale";

Scale::Scale(HX711 &hx711) : hx711_(hx711) {}

bool Scale::Tare(int samples, TickType_t timeout) {
  ESP_LOGI(kTag, "开始零点校准...");

  int32_t value = ReadAverage(samples, timeout);
  if (value == HX711::kUndefined) {
    ESP_LOGE(kTag, "读取零点值失败");
    return false;
  }

  offset_ = static_cast<float>(value);
  ESP_LOGI(kTag, "零点校准完成: offset=%.2f", offset_);
  return true;
}

bool Scale::Calibrate(float known_weight, int samples, TickType_t timeout) {
  if (known_weight == 0.0f) {
    ESP_LOGE(kTag, "标定重量不能为零");
    return false;
  }

  ESP_LOGI(kTag, "开始校准比例因子，标定重量: %.2f", known_weight);

  int32_t value = ReadAverage(samples, timeout);
  if (value == HX711::kUndefined) {
    ESP_LOGE(kTag, "读取校准值失败");
    return false;
  }

  // 计算比例因子
  scale_factor_ = (static_cast<float>(value) - offset_) / known_weight;

  if (scale_factor_ == 0.0f) {
    ESP_LOGE(kTag, "校准失败：比例因子为零");
    return false;
  }

  is_calibrated_ = true;
  ESP_LOGI(kTag, "校准完成: scale_factor=%.2f", scale_factor_);
  return true;
}

float Scale::GetWeight(int samples, TickType_t timeout) {
  int32_t value = ReadAverage(samples, timeout);
  if (value == HX711::kUndefined) {
    return NAN;
  }

  // 如果比例因子为零，返回未定义
  if (scale_factor_ == 0.0f) {
    return NAN;
  }

  float weight = (static_cast<float>(value) - offset_) / scale_factor_;
  return weight;
}

void Scale::SetCalibration(float offset, float scale_factor) {
  offset_ = offset;
  scale_factor_ = scale_factor;
  is_calibrated_ = (scale_factor != 0.0f);
  ESP_LOGI(kTag, "手动设置校准参数: offset=%.2f, scale_factor=%.2f", offset_,
           scale_factor_);
}

bool Scale::SaveCalibration(const char *namespace_name,
                            const char *key_prefix) {
  nvs_handle_t nvs_handle;
  esp_err_t err;
  char offset_key[32];
  char scale_key[32];

  // 构造键名
  snprintf(offset_key, sizeof(offset_key), "%s_offset", key_prefix);
  snprintf(scale_key, sizeof(scale_key), "%s_scale", key_prefix);

  // 打开NVS
  err = nvs_open(namespace_name, NVS_READWRITE, &nvs_handle);
  if (err != ESP_OK) {
    ESP_LOGE(kTag, "Error opening NVS: %s", esp_err_to_name(err));
    return false;
  }

  // 写入校准参数
  err = nvs_set_blob(nvs_handle, offset_key, &offset_, sizeof(float));
  if (err != ESP_OK) {
    ESP_LOGE(kTag, "Error saving offset: %s", esp_err_to_name(err));
    nvs_close(nvs_handle);
    return false;
  }

  err = nvs_set_blob(nvs_handle, scale_key, &scale_factor_, sizeof(float));
  if (err != ESP_OK) {
    ESP_LOGE(kTag, "Error saving scale factor: %s", esp_err_to_name(err));
    nvs_close(nvs_handle);
    return false;
  }

  // 提交更改
  err = nvs_commit(nvs_handle);
  if (err != ESP_OK) {
    ESP_LOGE(kTag, "Error committing to NVS: %s", esp_err_to_name(err));
    nvs_close(nvs_handle);
    return false;
  }

  nvs_close(nvs_handle);
  ESP_LOGI(kTag, "校准参数已保存到NVS");
  return true;
}

bool Scale::LoadCalibration(const char *namespace_name,
                            const char *key_prefix) {
  nvs_handle_t nvs_handle;
  esp_err_t err;
  char offset_key[32];
  char scale_key[32];
  float offset, scale_factor;
  size_t required_size = sizeof(float);

  // 构造键名
  snprintf(offset_key, sizeof(offset_key), "%s_offset", key_prefix);
  snprintf(scale_key, sizeof(scale_key), "%s_scale", key_prefix);

  // 打开NVS
  err = nvs_open(namespace_name, NVS_READONLY, &nvs_handle);
  if (err != ESP_OK) {
    ESP_LOGE(kTag, "Error opening NVS: %s", esp_err_to_name(err));
    return false;
  }

  // 读取零点偏移
  err = nvs_get_blob(nvs_handle, offset_key, &offset, &required_size);
  if (err != ESP_OK) {
    ESP_LOGE(kTag, "Error loading offset: %s", esp_err_to_name(err));
    nvs_close(nvs_handle);
    return false;
  }

  // 读取比例因子
  err = nvs_get_blob(nvs_handle, scale_key, &scale_factor, &required_size);
  if (err != ESP_OK) {
    ESP_LOGE(kTag, "Error loading scale factor: %s", esp_err_to_name(err));
    nvs_close(nvs_handle);
    return false;
  }

  nvs_close(nvs_handle);

  // 设置校准参数
  SetCalibration(offset, scale_factor);
  ESP_LOGI(kTag, "从NVS加载校准参数成功");
  return true;
}

int32_t Scale::ReadAverage(int samples, TickType_t timeout) {
  if (samples <= 0) {
    samples = 1;
  }

  int32_t sum = 0;
  int valid_samples = 0;

  for (int i = 0; i < samples; i++) {
    int32_t value = hx711_.Read(timeout);
    if (value != HX711::kUndefined) {
      sum += value;
      valid_samples++;
    }
  }

  if (valid_samples == 0) {
    ESP_LOGE(kTag, "没有有效的样本");
    return HX711::kUndefined;
  }

  return sum / valid_samples;
}