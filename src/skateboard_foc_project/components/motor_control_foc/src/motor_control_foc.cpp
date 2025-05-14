#include "motor_control_foc.hpp"
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_log.h"
#include "esp_simplefoc.h"
#include "math.h"
#include "sensors/HallSensor.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "MOTOR_CONTROL";

// 电机配置
#define MOTOR_PWM_FREQ CONFIG_FOC_PWM_FREQUENCY // PWM频率

// 主电机GPIO引脚(在sdkconfig中定义)
#define MOTOR1_PWM_U_PIN CONFIG_MOTOR_PWM_U_PIN
#define MOTOR1_PWM_V_PIN CONFIG_MOTOR_PWM_V_PIN
#define MOTOR1_PWM_W_PIN CONFIG_MOTOR_PWM_W_PIN

// 副电机GPIO引脚(在sdkconfig中定义)
#define MOTOR2_PWM_U_PIN CONFIG_MOTOR2_PWM_U_PIN
#define MOTOR2_PWM_V_PIN CONFIG_MOTOR2_PWM_V_PIN
#define MOTOR2_PWM_W_PIN CONFIG_MOTOR2_PWM_W_PIN

// 电机共用使能引脚
#define MOTOR_ENABLE_PIN CONFIG_MOTOR_ENABLE_PIN

// 电机参数
#define MOTOR_POLE_PAIRS CONFIG_MOTOR_POLE_PAIRS // 极对数
#define MOTOR_SUPPLY_VOLTAGE 36.0f               // 供电电压(伏特)

// Hall传感器引脚
#define MOTOR1_HALL_A CONFIG_MOTOR1_HALL_A
#define MOTOR1_HALL_B CONFIG_MOTOR1_HALL_B
#define MOTOR1_HALL_C CONFIG_MOTOR1_HALL_C
#define MOTOR2_HALL_A CONFIG_MOTOR2_HALL_A
#define MOTOR2_HALL_B CONFIG_MOTOR2_HALL_B
#define MOTOR2_HALL_C CONFIG_MOTOR2_HALL_C

// 电机驱动
BLDCDriver3PWM driver1 =
    BLDCDriver3PWM(MOTOR1_PWM_U_PIN, MOTOR1_PWM_V_PIN, MOTOR1_PWM_W_PIN);
BLDCDriver3PWM driver2 =
    BLDCDriver3PWM(MOTOR2_PWM_U_PIN, MOTOR2_PWM_V_PIN, MOTOR2_PWM_W_PIN);

// 电机和传感器
BLDCMotor motor1 = BLDCMotor(MOTOR_POLE_PAIRS);
BLDCMotor motor2 = BLDCMotor(MOTOR_POLE_PAIRS);
HallSensor sensor1 =
    HallSensor(MOTOR1_HALL_A, MOTOR1_HALL_B, MOTOR1_HALL_C, MOTOR_POLE_PAIRS);
HallSensor sensor2 =
    HallSensor(MOTOR2_HALL_A, MOTOR2_HALL_B, MOTOR2_HALL_C, MOTOR_POLE_PAIRS);

// Internal state
typedef struct {
  bool initialized;
  float target_speed;  // 目标速度(-1.0到1.0)
  float current_speed; // 当前速度
  motor_direction_t direction;
} motor_control_state_t;

// 双电机控制状态
static struct {
  motor_control_state_t motors[2]; // 0=主电机，1=副电机
  bool enabled;                    // 电机使能状态(共用)
} motor_state = {.motors = {{0}}, .enabled = false};

// Hall传感器中断处理函数
void doA1() { sensor1.handleA(); }
void doB1() { sensor1.handleB(); }
void doC1() { sensor1.handleC(); }
void doA2() { sensor2.handleA(); }
void doB2() { sensor2.handleB(); }
void doC2() { sensor2.handleC(); }

// C接口函数实现

// 初始化FOC电机控制
extern "C" esp_err_t motor_control_init(void) {
  ESP_LOGI(TAG, "正在初始化SimpleFOC双电机控制");

  // 初始化电机使能引脚
  gpio_config_t io_conf = {};
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = (1ULL << MOTOR_ENABLE_PIN);
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  gpio_config(&io_conf);

  // 初始时禁用电机
  gpio_set_level((gpio_num_t)MOTOR_ENABLE_PIN, 0);
  motor_state.enabled = false;

  // Current sensor initialization will configure continuous ADC internally

  // 初始化主电机Hall传感器
  sensor1.pullup = Pullup::USE_EXTERN;
  sensor1.init();
  ESP_LOGI(TAG, "主电机Hall传感器初始化成功");

  // 初始化副电机Hall传感器
  sensor2.pullup = Pullup::USE_EXTERN;
  sensor2.init();
  ESP_LOGI(TAG, "副电机Hall传感器初始化成功");

  // 启用中断
  sensor1.enableInterrupts(doA1, doB1, doC1);
  sensor2.enableInterrupts(doA2, doB2, doC2);
  ESP_LOGI(TAG, "传感器中断使能成功");

  // 设置传感器方向
  sensor1.direction = Direction::CW;
  sensor2.direction = Direction::CW;

  // 配置主电机驱动器
  driver1.voltage_power_supply = MOTOR_SUPPLY_VOLTAGE;
  driver1.voltage_limit = MOTOR_SUPPLY_VOLTAGE * 0.95f; // 限制在95%
  driver1.pwm_frequency = MOTOR_PWM_FREQ;
  driver1.init();
  ESP_LOGI(TAG, "主电机驱动器初始化成功");

  // 配置副电机驱动器
  driver2.voltage_power_supply = MOTOR_SUPPLY_VOLTAGE;
  driver2.voltage_limit = MOTOR_SUPPLY_VOLTAGE * 0.95f; // 限制在95%
  driver2.pwm_frequency = MOTOR_PWM_FREQ;
  driver2.init();
  ESP_LOGI(TAG, "副电机驱动器初始化成功");

  // 配置主电机
  motor1.linkSensor(&sensor1);
  motor1.linkDriver((BLDCDriver *)&driver1);

  // 配置副电机
  motor2.linkSensor(&sensor2);
  motor2.linkDriver((BLDCDriver *)&driver2);

  // 初始化电机
  motor1.init();
  motor2.init();
  ESP_LOGI(TAG, "电机初始化成功");

  // 初始化FOC算法
  motor1.initFOC(0, Direction::CW);
  motor2.initFOC(0, Direction::CW);
  ESP_LOGI(TAG, "FOC算法初始化成功");

  // Apply velocity PID and LPF settings from Kconfig
  motor1.PID_velocity.P = CONFIG_VELOCITY_PID_P / 1000.0f;
  motor1.PID_velocity.I = CONFIG_VELOCITY_PID_I / 1000.0f;
  motor1.PID_velocity.D = CONFIG_VELOCITY_PID_D / 1000.0f;
  motor1.LPF_velocity.Tf = CONFIG_VELOCITY_LPF_TIME_CONSTANT / 1000.0f;
  motor2.PID_velocity.P = CONFIG_VELOCITY_PID_P / 1000.0f;
  motor2.PID_velocity.I = CONFIG_VELOCITY_PID_I / 1000.0f;
  motor2.PID_velocity.D = CONFIG_VELOCITY_PID_D / 1000.0f;
  motor2.LPF_velocity.Tf = CONFIG_VELOCITY_LPF_TIME_CONSTANT / 1000.0f;
  ESP_LOGI(TAG, "Velocity PID set (P=%.3f, I=%.3f, D=%.3f)",
           CONFIG_VELOCITY_PID_P / 1000.0f, CONFIG_VELOCITY_PID_I / 1000.0f,
           CONFIG_VELOCITY_PID_D / 1000.0f);

  // 初始化电机状态
  motor_state.motors[MOTOR_ID_PRIMARY].target_speed = 0.0f;
  motor_state.motors[MOTOR_ID_PRIMARY].current_speed = 0.0f;
  motor_state.motors[MOTOR_ID_PRIMARY].direction = MOTOR_DIR_STOP;
  motor_state.motors[MOTOR_ID_PRIMARY].initialized = true;

  motor_state.motors[MOTOR_ID_SECONDARY].target_speed = 0.0f;
  motor_state.motors[MOTOR_ID_SECONDARY].current_speed = 0.0f;
  motor_state.motors[MOTOR_ID_SECONDARY].direction = MOTOR_DIR_STOP;
  motor_state.motors[MOTOR_ID_SECONDARY].initialized = true;

  ESP_LOGI(TAG, "SimpleFOC双电机控制初始化成功");

  return ESP_OK;
}

// 设置单个电机速度
extern "C" esp_err_t motor_control_set_speed(motor_id_t motor_id, float speed) {
  if (motor_id != MOTOR_ID_PRIMARY && motor_id != MOTOR_ID_SECONDARY) {
    return ESP_ERR_INVALID_ARG;
  }

  motor_control_state_t *motor_ctrl = &motor_state.motors[motor_id];
  if (!motor_ctrl->initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  // 将速度限制在有效范围内
  if (speed > 1.0f)
    speed = 1.0f;
  if (speed < -1.0f)
    speed = -1.0f;

  motor_ctrl->target_speed = speed;

  // 根据速度设置方向
  if (speed > 0.05f) {
    motor_ctrl->direction = MOTOR_DIR_FORWARD;
  } else if (speed < -0.05f) {
    motor_ctrl->direction = MOTOR_DIR_BACKWARD;
  } else {
    motor_ctrl->direction = MOTOR_DIR_STOP;
  }

  // 将标准化速度(-1到1)映射到电机所需的RPM或rad/s
  // 使用rad/s作为速度单位
  float max_speed = 30.0f; // 最大速度30 rad/s，约300 RPM
  float target_rad_per_sec = speed * max_speed;

  // 设置电机目标速度
  if (motor_id == MOTOR_ID_PRIMARY) {
    motor1.target = target_rad_per_sec;
  } else {
    motor2.target = target_rad_per_sec;
  }

  ESP_LOGD(TAG, "电机%d速度设置为%.2f,方向%d", motor_id, speed,
           motor_ctrl->direction);

  return ESP_OK;
}

// 同时设置两个电机速度
extern "C" esp_err_t motor_control_set_dual_speed(float speed1, float speed2) {
  esp_err_t ret1 = motor_control_set_speed(MOTOR_ID_PRIMARY, speed1);
  esp_err_t ret2 = motor_control_set_speed(MOTOR_ID_SECONDARY, speed2);

  if (ret1 != ESP_OK || ret2 != ESP_OK) {
    return ESP_FAIL;
  }

  return ESP_OK;
}

// 获取电机状态
extern "C" esp_err_t motor_control_get_status(motor_id_t motor_id,
                                              motor_status_t *status) {
  if (motor_id != MOTOR_ID_PRIMARY && motor_id != MOTOR_ID_SECONDARY) {
    return ESP_ERR_INVALID_ARG;
  }

  motor_control_state_t *motor_ctrl = &motor_state.motors[motor_id];
  if (!motor_ctrl->initialized || status == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  // 获取当前实际速度并转换为标准化值(-1到1)
  float max_speed = 30.0f; // 与设置速度时使用的相同
  float current_rad_per_sec;

  if (motor_id == MOTOR_ID_PRIMARY) {
    current_rad_per_sec = motor1.shaft_velocity;
    // Use internal FOC current estimates
    status->current_u = status->current_v = status->current_w = 0.0f;
    status->current_d = motor1.current.d * 1000.0f;
    status->current_q = motor1.current.q * 1000.0f;
  } else {
    current_rad_per_sec = motor2.shaft_velocity;
    // Use internal FOC current estimates
    status->current_u = status->current_v = status->current_w = 0.0f;
    status->current_d = motor2.current.d * 1000.0f;
    status->current_q = motor2.current.q * 1000.0f;
  }

  status->current_speed = current_rad_per_sec / max_speed;

  // 限制范围
  if (status->current_speed > 1.0f)
    status->current_speed = 1.0f;
  if (status->current_speed < -1.0f)
    status->current_speed = -1.0f;

  // 更新内部状态
  motor_ctrl->current_speed = status->current_speed;

  status->target_speed = motor_ctrl->target_speed;
  status->direction = motor_ctrl->direction;

  // 计算总电流大小（使用Q轴电流作为主要电流值）
  status->motor_current = fabsf(status->current_q) / 1000.0f; // 转换回安培

  // 电机温度目前还没有传感器，使用默认值
  status->motor_temp = 25.0f; // 在实际实现中将从温度传感器读取

  return ESP_OK;
}

// 使能所有电机
extern "C" esp_err_t motor_control_enable(motor_id_t motor_id) {
  if (!motor_state.motors[MOTOR_ID_PRIMARY].initialized ||
      !motor_state.motors[MOTOR_ID_SECONDARY].initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  // 忽略motor_id参数，使能所有电机
  if (!motor_state.enabled) {
    ESP_LOGI(TAG, "正在使能所有电机");
    gpio_set_level((gpio_num_t)MOTOR_ENABLE_PIN, 1);
    motor_state.enabled = true;

    // 使能SimpleFOC电机
    motor1.enable();
    motor2.enable();
  }

  return ESP_OK;
}

// 禁用所有电机
extern "C" esp_err_t motor_control_disable(motor_id_t motor_id) {
  if (!motor_state.motors[MOTOR_ID_PRIMARY].initialized ||
      !motor_state.motors[MOTOR_ID_SECONDARY].initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  // 忽略motor_id参数，禁用所有电机
  if (motor_state.enabled) {
    ESP_LOGI(TAG, "正在禁用所有电机");
    gpio_set_level((gpio_num_t)MOTOR_ENABLE_PIN, 0);
    motor_state.enabled = false;

    // 禁用SimpleFOC电机
    motor1.disable();
    motor2.disable();
  }

  return ESP_OK;
}

// FOC主循环更新函数 - 应在主循环或电机任务中周期性调用
extern "C" esp_err_t motor_control_update(void) {
  if (motor_state.enabled) {
    // FOC loop handles current estimation internally

    // 更新电机FOC算法
    motor1.loopFOC();
    motor1.move();

    motor2.loopFOC();
    motor2.move();
  }

  return ESP_OK;
}

// 计算倾斜补偿
extern "C" float calculate_incline_compensation(float angle) {
  // 基于角度的简单线性补偿
  // 正角度=上坡,负角度=下坡
  float comp = angle / 45.0f; // 45度坡给出满补偿

  // 限制补偿在有效范围内
  if (comp > 1.0f)
    comp = 1.0f;
  if (comp < -1.0f)
    comp = -1.0f;

  return comp;
}

// Monitoring: read motor shaft mechanical and electrical angles
extern "C" float motor_control_get_mechanical_angle(motor_id_t motor_id) {
  if (motor_id == MOTOR_ID_PRIMARY) {
    return sensor1.getMechanicalAngle();
  } else {
    return sensor2.getMechanicalAngle();
  }
}

// C++ wrapper implementation
#ifdef __cplusplus

MotorControlFOC::MotorControlFOC() {
  // Constructor is empty as initialization is done in init()
}

MotorControlFOC::~MotorControlFOC() {
  // Clean up if needed
  this->disable();
}

esp_err_t MotorControlFOC::init() {
  ESP_LOGI(TAG, "正在初始化SimpleFOC双电机控制");

  // 初始化电机使能引脚
  gpio_config_t io_conf = {};
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = (1ULL << MOTOR_ENABLE_PIN);
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  gpio_config(&io_conf);

  // 初始时禁用电机
  gpio_set_level((gpio_num_t)MOTOR_ENABLE_PIN, 0);
  motor_state.enabled = false;

  // Current sensor initialization will configure continuous ADC internally

  // 初始化主电机Hall传感器
  sensor1.pullup = Pullup::USE_EXTERN;
  sensor1.init();
  ESP_LOGI(TAG, "主电机Hall传感器初始化成功");

  // 初始化副电机Hall传感器
  sensor2.pullup = Pullup::USE_EXTERN;
  sensor2.init();
  ESP_LOGI(TAG, "副电机Hall传感器初始化成功");

  // 启用中断
  sensor1.enableInterrupts(doA1, doB1, doC1);
  sensor2.enableInterrupts(doA2, doB2, doC2);
  ESP_LOGI(TAG, "传感器中断使能成功");

  // 设置传感器方向
  sensor1.direction = Direction::CW;
  sensor2.direction = Direction::CW;

  // 配置主电机驱动器
  driver1.voltage_power_supply = MOTOR_SUPPLY_VOLTAGE;
  driver1.voltage_limit = MOTOR_SUPPLY_VOLTAGE * 0.95f;
  driver1.pwm_frequency = MOTOR_PWM_FREQ;
  driver1.init();
  ESP_LOGI(TAG, "主电机驱动器初始化成功");

  // 配置副电机驱动器
  driver2.voltage_power_supply = MOTOR_SUPPLY_VOLTAGE;
  driver2.voltage_limit = MOTOR_SUPPLY_VOLTAGE * 0.95f;
  driver2.pwm_frequency = MOTOR_PWM_FREQ;
  driver2.init();
  ESP_LOGI(TAG, "副电机驱动器初始化成功");

  // 配置主电机
  motor1.linkSensor(&sensor1);
  motor1.linkDriver((BLDCDriver *)&driver1);

  // 配置副电机
  motor2.linkSensor(&sensor2);
  motor2.linkDriver((BLDCDriver *)&driver2);

  // 初始化电机
  motor1.init();
  motor2.init();
  ESP_LOGI(TAG, "电机初始化成功");

  // 初始化FOC算法
  motor1.initFOC(0, Direction::CW);
  motor2.initFOC(0, Direction::CW);
  ESP_LOGI(TAG, "FOC算法初始化成功");

  // Apply velocity PID and LPF settings from Kconfig
  motor1.PID_velocity.P = CONFIG_VELOCITY_PID_P / 1000.0f;
  motor1.PID_velocity.I = CONFIG_VELOCITY_PID_I / 1000.0f;
  motor1.PID_velocity.D = CONFIG_VELOCITY_PID_D / 1000.0f;
  motor1.LPF_velocity.Tf = CONFIG_VELOCITY_LPF_TIME_CONSTANT / 1000.0f;
  motor2.PID_velocity.P = CONFIG_VELOCITY_PID_P / 1000.0f;
  motor2.PID_velocity.I = CONFIG_VELOCITY_PID_I / 1000.0f;
  motor2.PID_velocity.D = CONFIG_VELOCITY_PID_D / 1000.0f;
  motor2.LPF_velocity.Tf = CONFIG_VELOCITY_LPF_TIME_CONSTANT / 1000.0f;
  ESP_LOGI(TAG, "Velocity PID set (P=%.3f, I=%.3f, D=%.3f)",
           CONFIG_VELOCITY_PID_P / 1000.0f, CONFIG_VELOCITY_PID_I / 1000.0f,
           CONFIG_VELOCITY_PID_D / 1000.0f);

  // 设置电机状态
  motor_state.motors[MOTOR_ID_PRIMARY].target_speed = 0.0f;
  motor_state.motors[MOTOR_ID_PRIMARY].current_speed = 0.0f;
  motor_state.motors[MOTOR_ID_PRIMARY].direction = MOTOR_DIR_STOP;
  motor_state.motors[MOTOR_ID_PRIMARY].initialized = true;

  motor_state.motors[MOTOR_ID_SECONDARY].target_speed = 0.0f;
  motor_state.motors[MOTOR_ID_SECONDARY].current_speed = 0.0f;
  motor_state.motors[MOTOR_ID_SECONDARY].direction = MOTOR_DIR_STOP;
  motor_state.motors[MOTOR_ID_SECONDARY].initialized = true;

  ESP_LOGI(TAG, "SimpleFOC双电机控制初始化成功");
  return ESP_OK;
}

esp_err_t MotorControlFOC::enable() {
  esp_err_t ret1 = motor_control_enable(MOTOR_ID_PRIMARY);
  esp_err_t ret2 = motor_control_enable(MOTOR_ID_SECONDARY);

  // Return error if either motor failed to enable
  if (ret1 != ESP_OK)
    return ret1;
  return ret2;
}

esp_err_t MotorControlFOC::disable() {
  motor_control_disable(MOTOR_ID_PRIMARY);
  motor_control_disable(MOTOR_ID_SECONDARY);
  return ESP_OK;
}

esp_err_t MotorControlFOC::setSpeed(motor_id_t motorId, float speed) {
  return motor_control_set_speed(motorId, speed);
}

esp_err_t MotorControlFOC::setDualSpeed(float speed1, float speed2) {
  return motor_control_set_dual_speed(speed1, speed2);
}

esp_err_t MotorControlFOC::getStatus(motor_id_t motorId,
                                     motor_status_t *status) {
  return motor_control_get_status(motorId, status);
}

esp_err_t MotorControlFOC::update() { return motor_control_update(); }

float MotorControlFOC::calculateInclineCompensation(float angle) {
  return calculate_incline_compensation(angle);
}

void MotorControlFOC::getCurrents(float *ia1, float *ib1, float *ic1,
                                  float *ia2, float *ib2, float *ic2) {
  // Retrieve d/q current estimates from FOC internal state, convert to mA
  *ia1 = motor1.current.d * 1000.0f;
  *ib1 = motor1.current.q * 1000.0f;
  *ic1 = 0.0f;
  *ia2 = motor2.current.d * 1000.0f;
  *ib2 = motor2.current.q * 1000.0f;
  *ic2 = 0.0f;
}

float MotorControlFOC::getMechanicalAngle(motor_id_t motorId) {
  return motor_control_get_mechanical_angle(motorId);
}

float MotorControlFOC::getElectricalAngle(motor_id_t motorId) {
  if (motorId == MOTOR_ID_PRIMARY) {
    return motor1.electricalAngle();
  } else {
    return motor2.electricalAngle();
  }
}

#endif // __cplusplus
