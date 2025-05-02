/*
 * SPDX-FileCopyrightText: 2016-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "Sensor.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_simplefoc.h"
#include "esp_timer.h"
#include "foc_knob.h"
#include "foc_knob_default.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "iot_button.h"
#include "sensors/HallSensor.h"
#include "soc/gpio_num.h"
#include <stdio.h>
#include <string.h>
#define SWITCH_BUTTON 2
#define PHASE_U_GPIO 12
#define PHASE_V_GPIO 13
#define PHASE_W_GPIO 14
#define MOTOR_POWER 46
#define MOTOR_PP 10

#define HALL_SENSOR_1 37
#define HALL_SENSOR_2 36
#define HALL_SENSOR_3 35

#define USING_MCPWM 1

#if !USING_MCPWM
#define LEDC_CHAN_0 0
#define LEDC_CHAN_1 1
#define LEDC_CHAN_2 2
#endif

#define TAG "FOC_Knob_Example"

static foc_knob_handle_t foc_knob_handle = NULL;
static int mode = MOTOR_UNBOUND_NO_DETENTS;
static bool motor_shake = false;

/*update motor parameters based on hardware design*/
BLDCDriver3PWM driver =
    BLDCDriver3PWM(PHASE_U_GPIO, PHASE_V_GPIO, PHASE_W_GPIO);
BLDCMotor motor = BLDCMotor(MOTOR_PP);
HallSensor sensor =
    HallSensor(HALL_SENSOR_1, HALL_SENSOR_2, HALL_SENSOR_3, MOTOR_PP);

void doA() { sensor.handleA(); }
void doB() { sensor.handleB(); }
void doC() { sensor.handleC(); }

/*Motor initialization*/
void motor_init(void) {
  SimpleFOCDebug::enable();
  Serial.begin(115200);

  // Configure MOTOR_POWER pin as output and enable it
  gpio_config_t io_conf = {};
  io_conf.pin_bit_mask = (1ULL << MOTOR_POWER);
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&io_conf);
  gpio_set_level((gpio_num_t)MOTOR_POWER, 1); // Enable motor power
  ESP_LOGI(TAG, ":%d Motor Power Enabled", __LINE__);

  ESP_LOGI(TAG, ":%d Motor Initialize Start", __LINE__);
  sensor.pullup = Pullup::USE_EXTERN;
  sensor.init();
  ESP_LOGI(TAG, ":%d Sensor Initialize Successfully", __LINE__);
  // 启用中断，传递包装函数
  sensor.enableInterrupts(doA, doB, doC);
  ESP_LOGI(TAG, ":%d Interrupts Enabled", __LINE__);
  sensor.direction = Direction::CW; // 规格书1.9项"电机转向(出线端视)顺时针"
  ESP_LOGI(TAG, ":%d Direction Set", __LINE__);
  motor.linkSensor(&sensor);
  ESP_LOGI(TAG, ":%d Motor Link Sensor", __LINE__);
  // 电压和电流限制（规格书第4部分）
  driver.voltage_power_supply = 36; // 匹配规格书"额定电压36V"
  driver.voltage_limit = 30;        // 对应"控制器限流值30A±1A"
  motor.current_limit = 8.0;        // 额定电流≤8A
  motor.velocity_limit = 10000;

  // PWM配置优化
  driver.pwm_frequency = 10000;          // 10kHz
  motor.foc_modulation = SpaceVectorPWM; // 匹配正弦波控制器

  ESP_LOGI(TAG, ":%d Driver Initialize Start", __LINE__);
#if USING_MCPWM
  driver.init(0);
#else
  driver.init({LEDC_CHAN_0, LEDC_CHAN_1, LEDC_CHAN_2});
#endif
  ESP_LOGI(TAG, ":%d Driver Initialize Successfully", __LINE__);
  motor.linkDriver(&driver);
  ESP_LOGI(TAG, ":%d Motor Link Driver", __LINE__);
  motor.foc_modulation = SpaceVectorPWM;
  ESP_LOGI(TAG, ":%d Motor FOC Modulation Set", __LINE__);
  motor.controller = MotionControlType::torque;

  // 根据规格书转矩曲线优化PID参数
  motor.PID_velocity.P = 0.12 * MOTOR_PP; // 1.2
  motor.PID_velocity.I = 2.0 * MOTOR_PP;  // 20.0
  motor.PID_velocity.D = 0.04;
  motor.PID_velocity.output_ramp = 10000;
  motor.PID_velocity.limit = 30; // 最大电流限制
  motor.LPF_velocity.Tf = 0.01;  // 速度环滤波
  ESP_LOGI(TAG, ":%d Motor PID Set", __LINE__);
  motor.useMonitoring(Serial);
  ESP_LOGI(TAG, ":%d Motor Monitoring Set", __LINE__);
  motor.init();
  ESP_LOGI(TAG, ":%d Motor Init", __LINE__);
  motor.initFOC(0, Direction::CW);
  ESP_LOGI(TAG, ":%d Motor InitFOC", __LINE__);
  // // 相位自检
  // if (fabs(motor.electricalAngle() - sensor.getMechanicalAngle() * MOTOR_PP)
  //     > 0.1f) {
  //   ESP_LOGE(TAG, ":%d Phase sequence error! Check wiring", __LINE__);
  //   vTaskDelay(1000 / portTICK_PERIOD_MS);
  //   esp_restart();
  // }

  ESP_LOGI(TAG, ":%d Motor Initialize Successfully", __LINE__);
}

/*Button press callback*/
static void button_press_cb(void *arg, void *data) {
  mode++;
  if (mode >= MOTOR_MAX_MODES) {
    mode = MOTOR_UNBOUND_NO_DETENTS;
  }
  foc_knob_change_mode(foc_knob_handle, mode);
  ESP_LOGI(TAG, ":%d Mode Changed to %d", __LINE__, mode);
  motor_shake = true;
}

static void foc_knob_inc_cb(void *arg, void *data) {
  /*!< Do not printf here */
  foc_knob_state_t state;
  foc_knob_get_state(arg, &state);
}

static void foc_knob_dec_cb(void *arg, void *data) {
  /*!< Do not printf here */
  foc_knob_state_t state;
  foc_knob_get_state(arg, &state);
}

static void foc_knob_h_lim_cb(void *arg, void *data) {
  ESP_LOGI(TAG, ":%d foc_knob_h_lim_cb", __LINE__);
}

static void foc_knob_l_lim_cb(void *arg, void *data) {
  ESP_LOGI(TAG, ":%d foc_knob_l_lim_cb", __LINE__);
}

float motor_shake_func(float strength, int delay_cnt) {
  static int time_cnt = 0;
  if (time_cnt < delay_cnt) {
    time_cnt++;
    return strength;
  } else if (time_cnt < 2 * delay_cnt) {
    time_cnt++;
    return -strength;
  } else {
    time_cnt = 0;
    motor_shake = false;
    return 0;
  }
}

static float motor_pid_cb(float P, float D, float limit, float error) {
  motor.PID_velocity.limit = limit;
  motor.PID_velocity.P = P;
  motor.PID_velocity.D = D;
  return motor.PID_velocity(error);
}

static void motor_task(void *arg) {
  static float torque = 0;
  static float current_filtered = 0;
  const float alpha = 0.9;

  while (1) {
    // Feed the watchdog timer to prevent timeout
    vTaskDelay(1 / portTICK_PERIOD_MS);

    motor.loopFOC();
    static int count = 0;
    if (count++ % 1 == 0) {
      // sensor.handleA();
      // sensor.handleB();
      // sensor.handleC();
      static float last_mech_angle = 0;
      float mech_angle = sensor.getMechanicalAngle(); // 机械角度

      // 只在角度变化时输出日志
      if (mech_angle != last_mech_angle) {
        float elec_angle = motor.electricalAngle(); // 电角度
        ESP_LOGI(TAG, ":%d Mech: %.2f°  Elec: %.2f°  Velocity: %.1f RPM",
                 __LINE__,
                 mech_angle * 57.3, // 弧度转角度
                 elec_angle * 57.3, motor.shaft_velocity);

        ESP_LOGI(TAG, ":%d Move torque: %.1f", __LINE__, torque);
        last_mech_angle = mech_angle;

        // Yield to prevent watchdog timeout
        vTaskDelay(1 / portTICK_PERIOD_MS);
      }
    }

    // 电流保护
    current_filtered = alpha * current_filtered + (1 - alpha) * motor.current.q;
    if (current_filtered > 8.0f) {
      motor.move(0);
      ESP_LOGE(TAG, "Overcurrent! Shutdown. Q:%.2fA", current_filtered);
      vTaskDelay(2000 / portTICK_PERIOD_MS);
      continue;
    }

    // 堵转保护
    if (motor.shaft_velocity < 50 && fabs(motor.current.q) > 2.0) {
      motor.move(0);
      ESP_LOGW(TAG, "Stall detected at %.1f RPM", motor.shaft_velocity);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }

    if (motor_shake) {
      torque = motor_shake_func(2, 4);
    } else {
      torque = foc_knob_run(foc_knob_handle, motor.shaft_velocity,
                            motor.shaft_angle);
    }
    motor.move(torque);

    // Increase delay to prevent watchdog timeout - 10ms instead of 1ms
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

extern "C" void app_main(void) {
  button_config_t btn_config = {.type = BUTTON_TYPE_GPIO,
                                .long_press_time = 0,
                                .short_press_time = 0,
                                .gpio_button_config = {
                                    .gpio_num = SWITCH_BUTTON,
                                    .active_level = 0,
                                }};

  button_handle_t btn = iot_button_create(&btn_config);
  iot_button_register_cb(btn, BUTTON_PRESS_DOWN, button_press_cb, NULL);
  motor_init();

  foc_knob_config_t cfg = {
      .param_lists = default_foc_knob_param_lst,
      .param_list_num = MOTOR_MAX_MODES,
      .max_torque_out_limit = 5,
      .max_torque = 5,
      .pid_cb = motor_pid_cb,
  };

  foc_knob_handle = foc_knob_create(&cfg);

  foc_knob_register_cb(foc_knob_handle, FOC_KNOB_INC, foc_knob_inc_cb, NULL);
  foc_knob_register_cb(foc_knob_handle, FOC_KNOB_DEC, foc_knob_dec_cb, NULL);
  foc_knob_register_cb(foc_knob_handle, FOC_KNOB_H_LIM, foc_knob_h_lim_cb,
                       NULL);
  foc_knob_register_cb(foc_knob_handle, FOC_KNOB_L_LIM, foc_knob_l_lim_cb,
                       NULL);

  xTaskCreate(motor_task, "motor_task", 4096, NULL, 5, NULL);
}
