#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// 电机ID枚举
typedef enum {
  MOTOR_ID_PRIMARY = 0,  // 主电机（默认）
  MOTOR_ID_SECONDARY = 1 // 副电机
} motor_id_t;

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

  // 电流相关信息
  float current_u; // U相电流 (mA)
  float current_v; // V相电流 (mA)
  float current_w; // W相电流 (mA)
  float current_d; // D轴电流 (mA)
  float current_q; // Q轴电流 (mA)
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
 * @param motor_id 电机ID (主电机或副电机)
 * @param speed 速度值，正值为前进，负值为后退，单位为标准化值(-1.0到1.0)
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t motor_control_set_speed(motor_id_t motor_id, float speed);

/**
 * @brief 获取电机状态
 *
 * @param motor_id 电机ID (主电机或副电机)
 * @param status 指向状态结构体的指针
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t motor_control_get_status(motor_id_t motor_id, motor_status_t *status);

/**
 * @brief 启用电机
 *
 * @param motor_id 电机ID (主电机或副电机)
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t motor_control_enable(motor_id_t motor_id);

/**
 * @brief 禁用电机
 *
 * @param motor_id 电机ID (主电机或副电机)
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t motor_control_disable(motor_id_t motor_id);

/**
 * @brief 同时设置两个电机速度
 *
 * @param speed1 主电机速度值，标准化值(-1.0到1.0)
 * @param speed2 副电机速度值，标准化值(-1.0到1.0)
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t motor_control_set_dual_speed(float speed1, float speed2);

/**
 * @brief FOC电机控制周期性更新函数
 *
 * 该函数应在主循环或专用电机控制任务中定期调用，以便更新FOC算法
 * 并处理电机的闭环控制。
 *
 * @return esp_err_t ESP_OK成功，否则失败
 */
esp_err_t motor_control_update(void);

/**
 * @brief 计算坡度补偿值
 *
 * @param angle 坡度角度(度)，正值为上坡，负值为下坡
 * @return float 补偿值(-1.0到1.0)
 */
float calculate_incline_compensation(float angle);

/**
 * @brief 初始化电流传感器
 */
static void init_current_sensor(void);

/**
 * @brief 获取当前电流值
 */
void motor_control_foc_get_current(float *ia1, float *ib1, float *ic1,
                                   float *ia2, float *ib2, float *ic2);

#ifdef __cplusplus
}
#endif