#pragma once

// 移除废弃的ADC驱动
// #include "driver/adc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "motor_control_foc.h"  // 使用motor_control_foc.h中的motor_id_t定义


#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 电流传感器初始化
 * 
 * 初始化电流传感器ADC通道和配置
 * 
 * @return esp_err_t ESP_OK表示成功，否则失败
 */
esp_err_t current_sensor_init(void);

/**
 * @brief 获取指定电机的三相电流
 * 
 * 使用两相传感器测量，并通过 Ia + Ib + Ic = 0 计算第三相电流
 * 
 * @param motor_id 电机ID (主电机或副电机)
 * @param current_u 指针，用于存储U相电流值(mA)
 * @param current_v 指针，用于存储V相电流值(mA)
 * @param current_w 指针，用于存储W相电流值(mA)
 * @return esp_err_t ESP_OK表示成功，否则失败
 */
esp_err_t current_sensor_get_three_phase_current(motor_id_t motor_id, 
                                                float *current_u, 
                                                float *current_v, 
                                                float *current_w);

/**
 * @brief 获取指定电机相电流的Alpha-Beta分量
 * 
 * @param motor_id 电机ID (主电机或副电机)
 * @param current_alpha 指针，用于存储Alpha分量电流值(mA)
 * @param current_beta 指针，用于存储Beta分量电流值(mA)
 * @return esp_err_t ESP_OK表示成功，否则失败
 */
esp_err_t current_sensor_get_alpha_beta_current(motor_id_t motor_id, 
                                               float *current_alpha, 
                                               float *current_beta);

/**
 * @brief 获取指定电机相电流的DQ分量
 * 
 * @param motor_id 电机ID (主电机或副电机)
 * @param angle 电角度(弧度)
 * @param current_d 指针，用于存储D轴电流值(mA)
 * @param current_q 指针，用于存储Q轴电流值(mA)
 * @return esp_err_t ESP_OK表示成功，否则失败
 */
esp_err_t current_sensor_get_dq_current(motor_id_t motor_id, 
                                       float angle, 
                                       float *current_d, 
                                       float *current_q);

/**
 * @brief 校准电流传感器零点
 * 
 * 校准两个相位的电流传感器零点，第三相使用平均值
 * 
 * @param motor_id 电机ID (主电机或副电机)
 * @return esp_err_t ESP_OK表示成功，否则失败
 */
esp_err_t current_sensor_calibrate(motor_id_t motor_id);

#ifdef __cplusplus
}
#endif 