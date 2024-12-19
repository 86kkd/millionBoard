/*
 * SPDX-FileCopyrightText: 2016-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// 头文件包含部分
#include "driver/gpio.h"       // GPIO驱动
#include "esp_log.h"           // ESP32日志系统
#include "esp_simplefoc.h"     // 简单FOC库
#include "esp_timer.h"         // ESP32定时器
#include "foc_knob.h"          // FOC旋钮控制
#include "foc_knob_default.h"  // FOC旋钮默认配置
#include "freertos/FreeRTOS.h" // FreeRTOS操作系统
#include "freertos/task.h"     // FreeRTOS任务管理
#include "iot_button.h"        // 按钮驱动
#include <stdio.h>
#include <string.h>

// 引脚定义
#define SWITCH_BUTTON 0 // 切换按钮的GPIO引脚
#define PHASE_U_GPIO 3  // 电机U相GPIO引脚
#define PHASE_V_GPIO 8  // 电机V相GPIO引脚
#define PHASE_W_GPIO 18 // 电机W相GPIO引脚
#define MOTOR_PP 10     // 电机极对数

// MT6701磁编码器SPI配置
#define MT6701_SPI_HOST SPI2_HOST // 使用SPI2
#define MT6701_SPI_SCLK_GPIO 12   // SPI时钟引脚
#define MT6701_SPI_MISO_GPIO 13   // SPI数据输入引脚
#define MT6701_SPI_MOSI_GPIO -1   // SPI数据输出引脚（未使用）
#define MT6701_SPI_CS_GPIO 11     // SPI片选引脚

// 全局变量定义
static foc_knob_handle_t foc_knob_handle = NULL; // FOC旋钮句柄
static int mode = MOTOR_UNBOUND_NO_DETENTS;      // 当前模式
static bool motor_shake = false;                 // 电机振动标志

// 电机和驱动器对象初始化
BLDCDriver3PWM driver =
    BLDCDriver3PWM(PHASE_U_GPIO, PHASE_V_GPIO, PHASE_W_GPIO); // 三相PWM驱动器
BLDCMotor motor = BLDCMotor(MOTOR_PP);                        // 无刷电机对象

// 磁编码器对象初始化
MT6701 mt6701 =
    MT6701(MT6701_SPI_HOST, (gpio_num_t)MT6701_SPI_SCLK_GPIO,
           (gpio_num_t)MT6701_SPI_MISO_GPIO, (gpio_num_t)MT6701_SPI_MOSI_GPIO,
           (gpio_num_t)MT6701_SPI_CS_GPIO);

// 电机初始化函数
void motor_init(void) {
  SimpleFOCDebug::enable();        // 启用FOC调试
  Serial.begin(115200);            // 初始化串口
  mt6701.init();                   // 初始化编码器
  motor.linkSensor(&mt6701);       // 连接编码器到电机
  driver.voltage_power_supply = 5; // 设置电源电压
  driver.voltage_limit = 5;        // 设置电压限制
  motor.velocity_limit = 10000;    // 设置速度限制

  driver.init(0);                               // 初始化驱动器
  motor.linkDriver(&driver);                    // 连接驱动器到电机
  motor.foc_modulation = SpaceVectorPWM;        // 设置PWM调制方式
  motor.controller = MotionControlType::torque; // 设置控制模式为力矩控制

  // 设置PID参数
  motor.PID_velocity.P = 2.0;
  motor.PID_velocity.I = 0.5;
  motor.PID_velocity.D = 0.01;
  motor.PID_velocity.output_ramp = 10000;
  motor.PID_velocity.limit = 10;

  motor.useMonitoring(Serial); // 启用监控
  motor.init();                // 初始化电机
  motor.initFOC();             // 初始化FOC控制
}

// 按钮回调函数
static void button_press_cb(void *arg, void *data) {
  mode++; // 切换模式
  if (mode >= MOTOR_MAX_MODES) {
    mode = MOTOR_UNBOUND_NO_DETENTS;
  }
  foc_knob_change_mode(foc_knob_handle, mode); // 更改FOC旋钮模式
  motor_shake = true;                          // 触发振动反馈
}

// 电机任务函数
static void motor_task(void *arg) {
  static float torque = 0;
  while (1) {
    motor.loopFOC(); // FOC控制循环
    if (motor_shake) {
      torque = motor_shake_func(2, 4); // 执行振动
    } else {
      torque = foc_knob_run(foc_knob_handle, motor.shaft_velocity,
                            motor.shaft_angle); // 正常运行
    }
    motor.move(torque); // 输出力矩
    vTaskDelay(1);      // 任务延时
  }
}

// 主函数
extern "C" void app_main(void) {
  // 配置按钮
  button_config_t btn_config = {/*按钮配置*/};
  button_handle_t btn = iot_button_create(&btn_config);
  iot_button_register_cb(btn, BUTTON_PRESS_DOWN, button_press_cb, NULL);

  motor_init(); // 初始化电机

  // 配置FOC旋钮
  foc_knob_config_t cfg = {/*FOC旋钮配置*/};
  foc_knob_handle = foc_knob_create(&cfg);

  // 创建电机控制任务
  xTaskCreate(motor_task, "motor_task", 4096, NULL, 5, NULL);
}