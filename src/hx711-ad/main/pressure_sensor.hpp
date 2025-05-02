#pragma once

#include "freertos/FreeRTOS.h"
#include "nvs.h"
#include <inttypes.h>

// Forward declaration
class HX711;

class PressureSensor {
public:
  PressureSensor(HX711 &hx711);

  // 设置零点偏移量(空载时的值)
  bool Tare(int samples = 10, TickType_t timeout = pdMS_TO_TICKS(1000));

  // 使用已知重量设置校准系数
  bool Calibrate(float known_weight, int samples = 10,
                 TickType_t timeout = pdMS_TO_TICKS(1000));

  // 读取重量 (单位由校准时使用的重量单位决定)
  float GetWeight(int samples = 1, TickType_t timeout = pdMS_TO_TICKS(1000));

  // 获取当前校准参数
  float GetOffset() const { return offset_; }
  float GetScale() const { return scale_factor_; }

  // 手动设置校准参数
  void SetCalibration(float offset, float scale_factor);

  // 保存校准参数到NVS (非易失性存储)
  bool SaveCalibration(const char *namespace_name, const char *key_prefix);

  // 从NVS加载校准参数
  bool LoadCalibration(const char *namespace_name, const char *key_prefix);

  // 从旧格式加载校准参数（内部使用）
  bool LoadLegacyCalibration(nvs_handle_t nvs_handle, const char *key_prefix);

private:
  // 读取平均原始值
  int32_t ReadAverage(int samples, TickType_t timeout);

  HX711 &hx711_;
  float offset_ = 0.0f;       // 零点偏移量
  float scale_factor_ = 1.0f; // 比例因子 (raw/单位)
};