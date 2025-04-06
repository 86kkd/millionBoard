#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// 电机方向枚举
typedef enum {
  MOTOR_DIR_FORWARD = 1,
  MOTOR_DIR_BACKWARD = -1,
  MOTOR_DIR_STOP = 0
} motor_direction_t;

// 电机状态结构体
typedef struct {
  float current_speed;         // 当前速度 (RPM)
  float target_speed;          // 目标速度 (RPM)
  float motor_current;         // 电机电流 (A)
  float motor_temp;            // 电机温度 (°C)
  motor_direction_t direction; // 电机方向
} motor_status_t;

/**
 * @brief 初始化FOC电机控制
 *
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t motor_control_init(void);

/**
 * @brief 设置电机速度
 *
 * @param speed 速度值，正值为前进，负值为后退，单位为标准化值(-1.0到1.0)
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t motor_control_set_speed(float speed);

/**
 * @brief 获取电机状态
 *
 * @param status 指向状态结构体的指针
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t motor_control_get_status(motor_status_t *status);

/**
 * @brief 启用电机
 *
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t motor_control_enable(void);

/**
 * @brief 禁用电机
 *
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t motor_control_disable(void);

/**
 * @brief 计算坡度补偿值
 *
 * @param angle 坡度角度(度)，正值为上坡，负值为下坡
 * @return float 补偿值(-1.0到1.0)
 */
float calculate_incline_compensation(float angle);

#ifdef __cplusplus
}
#endif