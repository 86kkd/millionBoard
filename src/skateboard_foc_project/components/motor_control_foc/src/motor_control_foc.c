#include "motor_control_foc.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "math.h"
#include "current_sensor.h"  // 添加电流传感器头文件

static const char *TAG = "MOTOR_CONTROL";

// 电机配置
#define MOTOR_PWM_FREQ CONFIG_FOC_PWM_FREQUENCY           // PWM频率
#define MOTOR_PWM_RESOLUTION LEDC_TIMER_10_BIT // 10位分辨率(0-1023)
#define MOTOR_PWM_TIMER LEDC_TIMER_0
#define MOTOR2_PWM_TIMER LEDC_TIMER_1
#define MOTOR_PWM_MODE LEDC_LOW_SPEED_MODE

// 主电机GPIO引脚(在sdkconfig中定义)
#define MOTOR_PWM_U_PIN CONFIG_MOTOR_PWM_U_PIN
#define MOTOR_PWM_V_PIN CONFIG_MOTOR_PWM_V_PIN
#define MOTOR_PWM_W_PIN CONFIG_MOTOR_PWM_W_PIN

// 副电机GPIO引脚(在sdkconfig中定义)
#define MOTOR2_PWM_U_PIN CONFIG_MOTOR2_PWM_U_PIN
#define MOTOR2_PWM_V_PIN CONFIG_MOTOR2_PWM_V_PIN
#define MOTOR2_PWM_W_PIN CONFIG_MOTOR2_PWM_W_PIN

// 电机共用使能引脚
#define MOTOR_ENABLE_PIN CONFIG_MOTOR_ENABLE_PIN

// PWM通道定义
#define MOTOR1_CHANNEL_U LEDC_CHANNEL_0
#define MOTOR1_CHANNEL_V LEDC_CHANNEL_1
#define MOTOR1_CHANNEL_W LEDC_CHANNEL_2
#define MOTOR2_CHANNEL_U LEDC_CHANNEL_3
#define MOTOR2_CHANNEL_V LEDC_CHANNEL_4
#define MOTOR2_CHANNEL_W LEDC_CHANNEL_5

// 电机参数
#define MOTOR_POLE_PAIRS CONFIG_MOTOR_POLE_PAIRS  // 极对数
#define MOTOR_MAX_CURRENT 8.0f     // 最大电流(安培)
#define MOTOR_SUPPLY_VOLTAGE 36.0f // 供电电压(伏特)

// FOC控制状态
typedef struct {
  float angle;         // 电角度
  float voltage_alpha; // Alpha分量电压
  float voltage_beta;  // Beta分量电压
  float duty_a;        // A相占空比
  float duty_b;        // B相占空比
  float duty_c;        // C相占空比
} foc_state_t;

// 单个电机控制状态
typedef struct {
  bool initialized;
  float target_speed;  // 目标速度(-1.0到1.0)
  float current_speed; // 当前速度
  motor_direction_t direction;
  foc_state_t foc;
} motor_control_state_t;

// 双电机控制状态
static struct {
  motor_control_state_t motors[2]; // 0=主电机，1=副电机
  bool enabled;                    // 电机使能状态(共用)
} motor_state = {0};

// 内部函数 - 更新单个电机
static esp_err_t update_motor_pwm(motor_id_t motor_id, float speed) {
  if (motor_id != MOTOR_ID_PRIMARY && motor_id != MOTOR_ID_SECONDARY) {
    return ESP_ERR_INVALID_ARG;
  }

  motor_control_state_t *motor = &motor_state.motors[motor_id];
  if (!motor->initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  ledc_mode_t pwm_mode = MOTOR_PWM_MODE;
  uint8_t channel_u, channel_v, channel_w;
  
  if (motor_id == MOTOR_ID_PRIMARY) {
    channel_u = MOTOR1_CHANNEL_U;
    channel_v = MOTOR1_CHANNEL_V;
    channel_w = MOTOR1_CHANNEL_W;
  } else {
    channel_u = MOTOR2_CHANNEL_U;
    channel_v = MOTOR2_CHANNEL_V;
    channel_w = MOTOR2_CHANNEL_W;
  }

  // 使用FOC算法更新PWM信号
  float angle = motor->foc.angle;
  float amplitude = fabsf(speed);

  // 空间矢量调制(SVM)
  float ua, ub, uc;

  // 为每个相位生成正弦波(相差120度)
  ua = amplitude * sinf(angle);
  ub = amplitude * sinf(angle - 2.0f * M_PI / 3.0f);
  uc = amplitude * sinf(angle - 4.0f * M_PI / 3.0f);

  // 转换为PWM占空比(0-1)
  float duty_a = (ua + 1.0f) * 0.5f;
  float duty_b = (ub + 1.0f) * 0.5f;
  float duty_c = (uc + 1.0f) * 0.5f;

  // 应用占空比
  uint32_t duty_max = (1 << MOTOR_PWM_RESOLUTION) - 1;
  ledc_set_duty(pwm_mode, channel_u, (uint32_t)(duty_a * duty_max));
  ledc_set_duty(pwm_mode, channel_v, (uint32_t)(duty_b * duty_max));
  ledc_set_duty(pwm_mode, channel_w, (uint32_t)(duty_c * duty_max));

  ledc_update_duty(pwm_mode, channel_u);
  ledc_update_duty(pwm_mode, channel_v);
  ledc_update_duty(pwm_mode, channel_w);

  // 存储当前FOC状态
  motor->foc.duty_a = duty_a;
  motor->foc.duty_b = duty_b;
  motor->foc.duty_c = duty_c;

  // 增加角度以供下次更新(模拟电机旋转)
  // 在实际实现中,这将由传感器反馈决定
  motor->foc.angle += 0.1f * motor->target_speed;
  if (motor->foc.angle > 2.0f * M_PI) {
    motor->foc.angle -= 2.0f * M_PI;
  } else if (motor->foc.angle < 0.0f) {
    motor->foc.angle += 2.0f * M_PI;
  }

  return ESP_OK;
}

// 初始化FOC电机控制
esp_err_t motor_control_init(void) {
  // 检查是否已初始化
  if (motor_state.motors[MOTOR_ID_PRIMARY].initialized && 
      motor_state.motors[MOTOR_ID_SECONDARY].initialized) {
    return ESP_OK;
  }

  ESP_LOGI(TAG, "正在初始化FOC双电机控制");

  // 配置主电机LEDC定时器
  ledc_timer_config_t ledc_timer1 = {
      .duty_resolution = MOTOR_PWM_RESOLUTION,
      .freq_hz = MOTOR_PWM_FREQ,
      .speed_mode = MOTOR_PWM_MODE,
      .timer_num = MOTOR_PWM_TIMER,
      .clk_cfg = LEDC_AUTO_CLK,
  };
  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer1));

  // 配置副电机LEDC定时器
  ledc_timer_config_t ledc_timer2 = {
      .duty_resolution = MOTOR_PWM_RESOLUTION,
      .freq_hz = MOTOR_PWM_FREQ,
      .speed_mode = MOTOR_PWM_MODE,
      .timer_num = MOTOR2_PWM_TIMER,
      .clk_cfg = LEDC_AUTO_CLK,
  };
  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer2));

  // 为主电机每个相位配置LEDC通道
  ledc_channel_config_t ledc_channel_m1_u = {
      .channel = MOTOR1_CHANNEL_U,
      .duty = 0,
      .gpio_num = MOTOR_PWM_U_PIN,
      .speed_mode = MOTOR_PWM_MODE,
      .hpoint = 0,
      .timer_sel = MOTOR_PWM_TIMER,
  };
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_m1_u));

  ledc_channel_config_t ledc_channel_m1_v = {
      .channel = MOTOR1_CHANNEL_V,
      .duty = 0,
      .gpio_num = MOTOR_PWM_V_PIN,
      .speed_mode = MOTOR_PWM_MODE,
      .hpoint = 0,
      .timer_sel = MOTOR_PWM_TIMER,
  };
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_m1_v));

  ledc_channel_config_t ledc_channel_m1_w = {
      .channel = MOTOR1_CHANNEL_W,
      .duty = 0,
      .gpio_num = MOTOR_PWM_W_PIN,
      .speed_mode = MOTOR_PWM_MODE,
      .hpoint = 0,
      .timer_sel = MOTOR_PWM_TIMER,
  };
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_m1_w));

  // 为副电机每个相位配置LEDC通道
  ledc_channel_config_t ledc_channel_m2_u = {
      .channel = MOTOR2_CHANNEL_U,
      .duty = 0,
      .gpio_num = MOTOR2_PWM_U_PIN,
      .speed_mode = MOTOR_PWM_MODE,
      .hpoint = 0,
      .timer_sel = MOTOR2_PWM_TIMER,
  };
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_m2_u));

  ledc_channel_config_t ledc_channel_m2_v = {
      .channel = MOTOR2_CHANNEL_V,
      .duty = 0,
      .gpio_num = MOTOR2_PWM_V_PIN,
      .speed_mode = MOTOR_PWM_MODE,
      .hpoint = 0,
      .timer_sel = MOTOR2_PWM_TIMER,
  };
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_m2_v));

  ledc_channel_config_t ledc_channel_m2_w = {
      .channel = MOTOR2_CHANNEL_W,
      .duty = 0,
      .gpio_num = MOTOR2_PWM_W_PIN,
      .speed_mode = MOTOR_PWM_MODE,
      .hpoint = 0,
      .timer_sel = MOTOR2_PWM_TIMER,
  };
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_m2_w));

  // 配置电机使能引脚(共用)
  gpio_config_t io_conf = {
      .intr_type = GPIO_INTR_DISABLE,
      .mode = GPIO_MODE_OUTPUT,
      .pin_bit_mask = (1ULL << MOTOR_ENABLE_PIN),
      .pull_down_en = 0,
      .pull_up_en = 0,
  };
  gpio_config(&io_conf);

  // 初始时禁用电机
  gpio_set_level(MOTOR_ENABLE_PIN, 0);
  motor_state.enabled = false;
  
  // 初始化两个电机的状态
  for (int i = 0; i < 2; i++) {
    motor_state.motors[i].foc.angle = 0.0f;
    motor_state.motors[i].foc.voltage_alpha = 0.0f;
    motor_state.motors[i].foc.voltage_beta = 0.0f;
    motor_state.motors[i].foc.duty_a = 0.0f;
    motor_state.motors[i].foc.duty_b = 0.0f;
    motor_state.motors[i].foc.duty_c = 0.0f;
    motor_state.motors[i].target_speed = 0.0f;
    motor_state.motors[i].current_speed = 0.0f;
    motor_state.motors[i].direction = MOTOR_DIR_STOP;
    motor_state.motors[i].initialized = true;
  }

  ESP_LOGI(TAG, "FOC双电机控制初始化成功");

  return ESP_OK;
}

// 设置单个电机速度
esp_err_t motor_control_set_speed(motor_id_t motor_id, float speed) {
  if (motor_id != MOTOR_ID_PRIMARY && motor_id != MOTOR_ID_SECONDARY) {
    return ESP_ERR_INVALID_ARG;
  }

  motor_control_state_t *motor = &motor_state.motors[motor_id];
  if (!motor->initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  // 将速度限制在有效范围内
  if (speed > 1.0f)
    speed = 1.0f;
  if (speed < -1.0f)
    speed = -1.0f;

  motor->target_speed = speed;

  // 根据速度设置方向
  if (speed > 0.05f) {
    motor->direction = MOTOR_DIR_FORWARD;
  } else if (speed < -0.05f) {
    motor->direction = MOTOR_DIR_BACKWARD;
  } else {
    motor->direction = MOTOR_DIR_STOP;
  }

  // 更新PWM信号
  update_motor_pwm(motor_id, speed);

  ESP_LOGD(TAG, "电机%d速度设置为%.2f,方向%d", motor_id, speed, motor->direction);

  return ESP_OK;
}

// 同时设置两个电机速度
esp_err_t motor_control_set_dual_speed(float speed1, float speed2) {
  esp_err_t ret1 = motor_control_set_speed(MOTOR_ID_PRIMARY, speed1);
  esp_err_t ret2 = motor_control_set_speed(MOTOR_ID_SECONDARY, speed2);

  if (ret1 != ESP_OK || ret2 != ESP_OK) {
    return ESP_FAIL;
  }

  return ESP_OK;
}

// 获取电机状态
esp_err_t motor_control_get_status(motor_id_t motor_id, motor_status_t *status) {
  if (motor_id != MOTOR_ID_PRIMARY && motor_id != MOTOR_ID_SECONDARY) {
    return ESP_ERR_INVALID_ARG;
  }

  motor_control_state_t *motor = &motor_state.motors[motor_id];
  if (!motor->initialized || status == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  status->current_speed = motor->current_speed;
  status->target_speed = motor->target_speed;
  status->direction = motor->direction;

  // 获取三相电流
  current_sensor_get_three_phase_current(motor_id, 
                                        &status->current_u, 
                                        &status->current_v, 
                                        &status->current_w);

  // 获取DQ轴电流
  current_sensor_get_dq_current(motor_id, 
                               motor->foc.angle, 
                               &status->current_d, 
                               &status->current_q);

  // 计算总电流大小（使用Q轴电流作为主要电流值）
  status->motor_current = fabsf(status->current_q) / 1000.0f;  // 转换为安培
  
  // 电机温度目前还没有传感器，使用默认值
  status->motor_temp = 25.0f;   // 在实际实现中将从温度传感器读取

  return ESP_OK;
}

// 使能所有电机
esp_err_t motor_control_enable(motor_id_t motor_id) {
  if (!motor_state.motors[MOTOR_ID_PRIMARY].initialized || 
      !motor_state.motors[MOTOR_ID_SECONDARY].initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  // 忽略motor_id参数，使能所有电机
  if (!motor_state.enabled) {
    ESP_LOGI(TAG, "正在使能所有电机");
    gpio_set_level(MOTOR_ENABLE_PIN, 1);
    motor_state.enabled = true;
  }

  return ESP_OK;
}

// 禁用所有电机
esp_err_t motor_control_disable(motor_id_t motor_id) {
  if (!motor_state.motors[MOTOR_ID_PRIMARY].initialized || 
      !motor_state.motors[MOTOR_ID_SECONDARY].initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  // 忽略motor_id参数，禁用所有电机
  if (motor_state.enabled) {
    ESP_LOGI(TAG, "正在禁用所有电机");
    gpio_set_level(MOTOR_ENABLE_PIN, 0);
    motor_state.enabled = false;
    
    // 将所有PWM通道设置为零
    ledc_set_duty(MOTOR_PWM_MODE, MOTOR1_CHANNEL_U, 0);
    ledc_set_duty(MOTOR_PWM_MODE, MOTOR1_CHANNEL_V, 0);
    ledc_set_duty(MOTOR_PWM_MODE, MOTOR1_CHANNEL_W, 0);
    ledc_update_duty(MOTOR_PWM_MODE, MOTOR1_CHANNEL_U);
    ledc_update_duty(MOTOR_PWM_MODE, MOTOR1_CHANNEL_V);
    ledc_update_duty(MOTOR_PWM_MODE, MOTOR1_CHANNEL_W);
    
    ledc_set_duty(MOTOR_PWM_MODE, MOTOR2_CHANNEL_U, 0);
    ledc_set_duty(MOTOR_PWM_MODE, MOTOR2_CHANNEL_V, 0);
    ledc_set_duty(MOTOR_PWM_MODE, MOTOR2_CHANNEL_W, 0);
    ledc_update_duty(MOTOR_PWM_MODE, MOTOR2_CHANNEL_U);
    ledc_update_duty(MOTOR_PWM_MODE, MOTOR2_CHANNEL_V);
    ledc_update_duty(MOTOR_PWM_MODE, MOTOR2_CHANNEL_W);
  }
  
  return ESP_OK;
}

// 计算倾斜补偿
float calculate_incline_compensation(float angle) {
  // 基于角度的简单线性补偿
  // 正角度=上坡,负角度=下坡
  float comp = angle / 45.0f; // 45度坡给出满补偿
  
  // 限制补偿在有效范围内
  if (comp > 1.0f) comp = 1.0f;
  if (comp < -1.0f) comp = -1.0f;
  
  return comp;
}
