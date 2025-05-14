#pragma once

#include "freertos/FreeRTOS.h" // for TickType_t and pdMS_TO_TICKS
#include "hx711.hpp"
#include "nvs.h" // Add this for nvs_handle_t type
#include <inttypes.h>
// Forward declaration to avoid including hx711.hpp here and prevent
// redefinition

class PressureSensor {
public:
  // Static functions for global initialization/deinitialization
  static esp_err_t Init(void);
  static esp_err_t Deinit(void);
  static void Task(void *pvParameters);

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

  // 检查校准状态
  bool IsCalibrated() const { return scale_factor_ != 0.0f; }

  // Callback type for reporting pressure data and new target speed
  typedef void (*DataCallback)(float front_pressure, float rear_pressure,
                               float target_speed, bool is_moving);
  // Register a callback to receive pressure updates
  static void RegisterCallback(DataCallback cb);

private:
  // 读取平均原始值
  int32_t ReadAverage(int samples, TickType_t timeout);

  // 加载旧版格式的校准数据
  bool LoadLegacyCalibration(nvs_handle_t nvs_handle, const char *key_prefix);

  HX711 &hx711_;
  float offset_ = 0.0f;       // 零点偏移量
  float scale_factor_ = 1.0f; // 比例因子 (raw/单位)

  // Static callback invoked on each pressure update
  static DataCallback data_callback_;
};