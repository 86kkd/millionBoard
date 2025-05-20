#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>
#include "pa1010d.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Send command to PA1010D GPS module.
 *
 * @param handle PA1010D driver handle.
 * @param command Command string to send (null-terminated).
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t pa1010d_send_command(pa1010d_handle_t handle, const char *command);

/**
 * @brief Initialize PA1010D GPS and I2C interface for FreeRTOS task.
 *
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t pa1010d_gps_init(void);

/**
 * @brief PA1010D GPS FreeRTOS task function.
 *
 * @param pvParameters Task parameters (unused).
 */
void pa1010d_gps_task(void *pvParameters);

// 外部 GPS 信息结构体，仅包含关键字段
typedef struct {
  bool has_fix;         // 是否已定位
  int num_satellites;   // 可见卫星数
  float latitude;       // 纬度
  float longitude;      // 经度
} gps_info_t;

/**
 * @brief 获取最新的 GPS 信息
 * @param info 指向 GPS 信息结构体的指针
 * @return esp_err_t ESP_OK 成功，否则失败
 */
esp_err_t pa1010d_gps_get_info(gps_info_t *info);

#ifdef __cplusplus
}
#endif
