#include "motor_control_foc.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "math.h"

static const char *TAG = "MOTOR_CONTROL";

// 电机配置
#define MOTOR_PWM_FREQ 25000                   // PWM频率
#define MOTOR_PWM_RESOLUTION LEDC_TIMER_10_BIT // 10位分辨率(0-1023)
#define MOTOR_PWM_TIMER LEDC_TIMER_0
#define MOTOR_PWM_MODE LEDC_LOW_SPEED_MODE

// 电机GPIO引脚(在sdkconfig中定义)
#define MOTOR_PWM_U_PIN CONFIG_MOTOR_PWM_U_PIN
#define MOTOR_PWM_V_PIN CONFIG_MOTOR_PWM_V_PIN
#define MOTOR_PWM_W_PIN CONFIG_MOTOR_PWM_W_PIN
#define MOTOR_ENABLE_PIN CONFIG_MOTOR_ENABLE_PIN

// 电机参数
#define MOTOR_POLE_PAIRS 10        // 极对数
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

// 电机控制状态
static struct {
  bool initialized;
  bool enabled;
  float target_speed;  // 目标速度(-1.0到1.0)
  float current_speed; // 当前速度
  motor_direction_t direction;
  foc_state_t foc;
} motor_state = {0};

// 初始化FOC电机控制
esp_err_t motor_control_init(void) {
  if (motor_state.initialized) {
    return ESP_OK;
  }

  ESP_LOGI(TAG, "正在初始化FOC电机控制");

  // 配置LEDC定时器
  ledc_timer_config_t ledc_timer = {
      .duty_resolution = MOTOR_PWM_RESOLUTION,
      .freq_hz = MOTOR_PWM_FREQ,
      .speed_mode = MOTOR_PWM_MODE,
      .timer_num = MOTOR_PWM_TIMER,
      .clk_cfg = LEDC_AUTO_CLK,
  };
  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

  // 为每个电机相位配置LEDC通道
  ledc_channel_config_t ledc_channel_u = {
      .channel = LEDC_CHANNEL_0,
      .duty = 0,
      .gpio_num = MOTOR_PWM_U_PIN,
      .speed_mode = MOTOR_PWM_MODE,
      .hpoint = 0,
      .timer_sel = MOTOR_PWM_TIMER,
  };
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_u));

  ledc_channel_config_t ledc_channel_v = {
      .channel = LEDC_CHANNEL_1,
      .duty = 0,
      .gpio_num = MOTOR_PWM_V_PIN,
      .speed_mode = MOTOR_PWM_MODE,
      .hpoint = 0,
      .timer_sel = MOTOR_PWM_TIMER,
  };
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_v));

  ledc_channel_config_t ledc_channel_w = {
      .channel = LEDC_CHANNEL_2,
      .duty = 0,
      .gpio_num = MOTOR_PWM_W_PIN,
      .speed_mode = MOTOR_PWM_MODE,
      .hpoint = 0,
      .timer_sel = MOTOR_PWM_TIMER,
  };
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_w));

  // 配置使能引脚
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

  // 初始化FOC状态
  motor_state.foc.angle = 0.0f;
  motor_state.foc.voltage_alpha = 0.0f;
  motor_state.foc.voltage_beta = 0.0f;
  motor_state.foc.duty_a = 0.0f;
  motor_state.foc.duty_b = 0.0f;
  motor_state.foc.duty_c = 0.0f;

  // 初始化电机状态
  motor_state.target_speed = 0.0f;
  motor_state.current_speed = 0.0f;
  motor_state.direction = MOTOR_DIR_STOP;

  motor_state.initialized = true;
  ESP_LOGI(TAG, "FOC电机控制初始化成功");

  return ESP_OK;
}

// 设置电机速度
esp_err_t motor_control_set_speed(float speed) {
  if (!motor_state.initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  // 将速度限制在有效范围内
  if (speed > 1.0f)
    speed = 1.0f;
  if (speed < -1.0f)
    speed = -1.0f;

  motor_state.target_speed = speed;

  // 根据速度设置方向
  if (speed > 0.05f) {
    motor_state.direction = MOTOR_DIR_FORWARD;
  } else if (speed < -0.05f) {
    motor_state.direction = MOTOR_DIR_BACKWARD;
  } else {
    motor_state.direction = MOTOR_DIR_STOP;
  }

  // 使用FOC算法更新PWM信号
  float angle = motor_state.foc.angle;
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
  ledc_set_duty(MOTOR_PWM_MODE, LEDC_CHANNEL_0, (uint32_t)(duty_a * duty_max));
  ledc_set_duty(MOTOR_PWM_MODE, LEDC_CHANNEL_1, (uint32_t)(duty_b * duty_max));
  ledc_set_duty(MOTOR_PWM_MODE, LEDC_CHANNEL_2, (uint32_t)(duty_c * duty_max));

  ledc_update_duty(MOTOR_PWM_MODE, LEDC_CHANNEL_0);
  ledc_update_duty(MOTOR_PWM_MODE, LEDC_CHANNEL_1);
  ledc_update_duty(MOTOR_PWM_MODE, LEDC_CHANNEL_2);

  // 存储当前FOC状态
  motor_state.foc.duty_a = duty_a;
  motor_state.foc.duty_b = duty_b;
  motor_state.foc.duty_c = duty_c;

  // 增加角度以供下次更新(模拟电机旋转)
  // 在实际实现中,这将由传感器反馈决定
  motor_state.foc.angle += 0.1f * motor_state.target_speed;
  if (motor_state.foc.angle > 2.0f * M_PI) {
    motor_state.foc.angle -= 2.0f * M_PI;
  } else if (motor_state.foc.angle < 0.0f) {
    motor_state.foc.angle += 2.0f * M_PI;
  }

  ESP_LOGD(TAG, "电机速度设置为%.2f,方向%d", speed, motor_state.direction);

  return ESP_OK;
}

// 获取电机状态
esp_err_t motor_control_get_status(motor_status_t *status) {
  if (!motor_state.initialized || status == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  status->current_speed = motor_state.current_speed;
  status->target_speed = motor_state.target_speed;
  status->motor_current = 0.0f; // 在实际实现中将从电流传感器读取
  status->motor_temp = 25.0f;   // 在实际实现中将从温度传感器读取
  status->direction = motor_state.direction;

  return ESP_OK;
}

// 使能电机
esp_err_t motor_control_enable(void) {
  if (!motor_state.initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  ESP_LOGI(TAG, "正在使能电机");
  gpio_set_level(MOTOR_ENABLE_PIN, 1);
  motor_state.enabled = true;

  return ESP_OK;
}

// 禁用电机
esp_err_t motor_control_disable(void) {
  if (!motor_state.initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  ESP_LOGI(TAG, "正在禁用电机");
  gpio_set_level(MOTOR_ENABLE_PIN, 0);
  motor_state.enabled = false;

  // 将所有PWM通道设置为零
  ledc_set_duty(MOTOR_PWM_MODE, LEDC_CHANNEL_0, 0);
  ledc_set_duty(MOTOR_PWM_MODE, LEDC_CHANNEL_1, 0);
  ledc_set_duty(MOTOR_PWM_MODE, LEDC_CHANNEL_2, 0);
  ledc_update_duty(MOTOR_PWM_MODE, LEDC_CHANNEL_0);
  ledc_update_duty(MOTOR_PWM_MODE, LEDC_CHANNEL_1);
  ledc_update_duty(MOTOR_PWM_MODE, LEDC_CHANNEL_2);

  return ESP_OK;
}

// 计算倾斜补偿
float calculate_incline_compensation(float angle) {
  // 基于角度的简单线性补偿
  // 正角度=上坡,负角度=下坡

  // 将补偿限制在合理值范围内
  const float max_compensation = 0.5f; // 最大50%补偿

  // 根据角度应用补偿
  // 10度倾斜产生约0.17的补偿值
  float compensation = sinf(angle * M_PI / 180.0f) * max_compensation;

  ESP_LOGD(TAG, "倾斜补偿:%.2f,角度%.1f°", compensation, angle);

  return compensation;
}
