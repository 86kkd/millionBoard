#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// 节点 ID 和 PDO1 CAN ID 定义
#define ANGLE_SENSOR_NODE_ID     0x05
#define ANGLE_SENSOR_PDO1_CAN_ID (0x180 + ANGLE_SENSOR_NODE_ID)

// 角度传感器数据结构体
typedef struct {
  float roll;     // X轴角度(度)
  float pitch;    // Y轴角度(度)，即坡度
  float yaw;      // Z轴角度(度)
  float temp;     // 传感器温度(°C)
  uint8_t status; // 传感器状态
} angle_sensor_data_t;

/**
 * @brief 初始化角度传感器，创建 CAN 接收任务
 *
 * @return esp_err_t ESP_OK 成功，否则失败
 */
esp_err_t angle_sensor_init(void);

/**
 * @brief 读取最新的角度传感器数据
 *
 * @param data 指向数据结构体的指针
 * @return esp_err_t ESP_OK 成功，否则失败
 */
esp_err_t angle_sensor_read(angle_sensor_data_t *data);

#ifdef __cplusplus
}
#endif