#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 电池状态结构体
typedef struct {
  float voltage;         // 电池电压(V)
  float current;         // 电池电流(A)，正值为放电，负值为充电
  uint8_t percentage;    // 电池电量百分比(0-100)
  float temperature;     // 电池温度(°C)
  uint16_t cycle_count;  // 循环次数
  bool is_charging;      // 是否正在充电
  bool is_fully_charged; // 是否充满
  bool has_error;        // 是否有错误
  uint8_t error_code;    // 错误代码
} battery_status_t;

/**
 * @brief 初始化电池管理模块
 *
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t battery_mgr_init(void);

/**
 * @brief 获取电池状态
 *
 * @param status 指向状态结构体的指针
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t battery_mgr_get_status(battery_status_t *status);

/**
 * @brief 校准电池电量计算
 *
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t battery_mgr_calibrate(void);

/**
 * @brief 注册电池状态变化回调函数
 *
 * @param callback 回调函数
 * @param user_data 用户数据
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t battery_mgr_register_callback(void (*callback)(battery_status_t *,
                                                         void *),
                                        void *user_data);

/**
 * @brief 关闭电池管理模块
 *
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t battery_mgr_deinit(void);

#ifdef __cplusplus
}
#endif