#include "motor_control_foc.hpp"
#include "current_sensor.hpp"
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
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

// INA140电流传感器ADC通道
#define MOTOR1_CURRENT_A CONFIG_MOTOR1_CURRENT_A
#define MOTOR1_CURRENT_B CONFIG_MOTOR1_CURRENT_B
#define MOTOR2_CURRENT_A CONFIG_MOTOR2_CURRENT_A
#define MOTOR2_CURRENT_B CONFIG_MOTOR2_CURRENT_B

// 电流传感器参数
#define CURRENT_SHUNT_RESISTOR 0.01f // 10mΩ分流电阻
#define CURRENT_SENSE_GAIN 20.0f     // INA140增益

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

// 电流传感器
ESP32InlineCurrentSense current_sense1 =
    ESP32InlineCurrentSense(CURRENT_SHUNT_RESISTOR, CURRENT_SENSE_GAIN,
                            MOTOR1_CURRENT_A, MOTOR1_CURRENT_B);
ESP32InlineCurrentSense current_sense2 =
    ESP32InlineCurrentSense(CURRENT_SHUNT_RESISTOR, CURRENT_SENSE_GAIN,
                            MOTOR2_CURRENT_A, MOTOR2_CURRENT_B);

// ADC句柄
adc_oneshot_unit_handle_t adc1_handle = NULL;
adc_cali_handle_t adc1_cali_handle = NULL;

// 内部状态
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

// 初始化ADC
static esp_err_t init_adc(void) {
  // 创建ADC单次转换句柄
  adc_oneshot_unit_init_cfg_t init_config = {.unit_id = ADC_UNIT_1,
                                             .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
                                             .ulp_mode = ADC_ULP_MODE_DISABLE};
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));

  // 配置ADC通道
  adc_oneshot_chan_cfg_t chan_config = {.atten = ADC_ATTEN_DB_12, // 0-3.3V范围
                                        .bitwidth = ADC_BITWIDTH_DEFAULT};

  // 配置四个电流传感器通道
  ESP_ERROR_CHECK(adc_oneshot_config_channel(
      adc1_handle, (adc_channel_t)MOTOR1_CURRENT_A, &chan_config));
  ESP_ERROR_CHECK(adc_oneshot_config_channel(
      adc1_handle, (adc_channel_t)MOTOR1_CURRENT_B, &chan_config));
  ESP_ERROR_CHECK(adc_oneshot_config_channel(
      adc1_handle, (adc_channel_t)MOTOR2_CURRENT_A, &chan_config));
  ESP_ERROR_CHECK(adc_oneshot_config_channel(
      adc1_handle, (adc_channel_t)MOTOR2_CURRENT_B, &chan_config));

  // 创建校准方案
  bool do_calibration = false;

// 检查是否支持curve fitting校准
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  ESP_LOGI(TAG, "Calibration scheme supported: Curve Fitting");
  adc_cali_curve_fitting_config_t cali_config = {
      .unit_id = ADC_UNIT_1,
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };
  ESP_ERROR_CHECK(
      adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_handle));
  do_calibration = true;
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
  ESP_LOGI(TAG, "Calibration scheme supported: Line Fitting");
  adc_cali_line_fitting_config_t cali_config = {
      .unit_id = ADC_UNIT_1,
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };
  ESP_ERROR_CHECK(
      adc_cali_create_scheme_line_fitting(&cali_config, &adc1_cali_handle));
  do_calibration = true;
#else
  ESP_LOGW(TAG, "No calibration scheme supported");
#endif

  if (!do_calibration) {
    ESP_LOGW(TAG, "当前芯片不支持ADC校准，将使用未校准的ADC读数");
  } else {
    ESP_LOGI(TAG, "ADC校准方案已创建");
  }

  return ESP_OK;
}

// 校准ADC
static esp_err_t calibrate_adc() {
  if (adc1_cali_handle == NULL) {
    ESP_LOGW(TAG, "ADC校准句柄未初始化，跳过校准");
    return ESP_FAIL;
  }

  // 校准第一个电机电流传感器的A通道
  int raw_iu, voltage_iu;
  ESP_ERROR_CHECK(
      adc_oneshot_read(adc1_handle, (adc_channel_t)MOTOR1_CURRENT_A, &raw_iu));
  ESP_ERROR_CHECK(
      adc_cali_raw_to_voltage(adc1_cali_handle, raw_iu, &voltage_iu));
  ESP_LOGI(TAG, "Motor1 Current A - ADC raw: %d, voltage: %d mV", raw_iu,
           voltage_iu);

  // 校准第一个电机电流传感器的B通道
  int raw_iv, voltage_iv;
  ESP_ERROR_CHECK(
      adc_oneshot_read(adc1_handle, (adc_channel_t)MOTOR1_CURRENT_B, &raw_iv));
  ESP_ERROR_CHECK(
      adc_cali_raw_to_voltage(adc1_cali_handle, raw_iv, &voltage_iv));
  ESP_LOGI(TAG, "Motor1 Current B - ADC raw: %d, voltage: %d mV", raw_iv,
           voltage_iv);

  // 校准第二个电机电流传感器的A通道
  int raw_iu2, voltage_iu2;
  ESP_ERROR_CHECK(
      adc_oneshot_read(adc1_handle, (adc_channel_t)MOTOR2_CURRENT_A, &raw_iu2));
  ESP_ERROR_CHECK(
      adc_cali_raw_to_voltage(adc1_cali_handle, raw_iu2, &voltage_iu2));
  ESP_LOGI(TAG, "Motor2 Current A - ADC raw: %d, voltage: %d mV", raw_iu2,
           voltage_iu2);

  // 校准第二个电机电流传感器的B通道
  int raw_iv2, voltage_iv2;
  ESP_ERROR_CHECK(
      adc_oneshot_read(adc1_handle, (adc_channel_t)MOTOR2_CURRENT_B, &raw_iv2));
  ESP_ERROR_CHECK(
      adc_cali_raw_to_voltage(adc1_cali_handle, raw_iv2, &voltage_iv2));
  ESP_LOGI(TAG, "Motor2 Current B - ADC raw: %d, voltage: %d mV", raw_iv2,
           voltage_iv2);

  return ESP_OK;
}

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

  // 初始化ADC
  init_adc();
  ESP_LOGI(TAG, "ADC初始化成功");

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

  // 初始化电流传感器
  init_current_sensor();
  ESP_LOGI(TAG, "电流传感器初始化成功");

  // 跳过电流传感器对齐过程
  current_sense1.skip_align = true;
  current_sense2.skip_align = true;

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
  // 使用强制类型转换
  motor1.linkDriver((BLDCDriver *)&driver1);
  motor1.linkCurrentSense(&current_sense1);
  motor1.voltage_limit = MOTOR_SUPPLY_VOLTAGE * 0.95f;
  motor1.current_limit = 8.0f; // 8A最大电流限制
  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor1.controller = MotionControlType::velocity;
  motor1.torque_controller = TorqueControlType::foc_current; // 使用电流闭环控制

  // 优化PID参数
  motor1.PID_velocity.P = 0.12f * MOTOR_POLE_PAIRS;
  motor1.PID_velocity.I = 2.0f * MOTOR_POLE_PAIRS;
  motor1.PID_velocity.D = 0.04f;
  motor1.PID_velocity.output_ramp = 10000.0f;
  motor1.PID_velocity.limit = 30.0f;
  motor1.LPF_velocity.Tf = 0.01f;

  // 配置副电机
  motor2.linkSensor(&sensor2);
  // 使用强制类型转换
  motor2.linkDriver((BLDCDriver *)&driver2);
  motor2.linkCurrentSense(&current_sense2);
  motor2.voltage_limit = MOTOR_SUPPLY_VOLTAGE * 0.95f;
  motor2.current_limit = 8.0f; // 8A最大电流限制
  motor2.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor2.controller = MotionControlType::velocity;
  motor2.torque_controller = TorqueControlType::foc_current; // 使用电流闭环控制

  // 优化PID参数(与主电机相同)
  motor2.PID_velocity.P = 0.12f * MOTOR_POLE_PAIRS;
  motor2.PID_velocity.I = 2.0f * MOTOR_POLE_PAIRS;
  motor2.PID_velocity.D = 0.04f;
  motor2.PID_velocity.output_ramp = 10000.0f;
  motor2.PID_velocity.limit = 30.0f;
  motor2.LPF_velocity.Tf = 0.01f;

  // 初始化电机
  motor1.init();
  motor2.init();
  ESP_LOGI(TAG, "电机初始化成功");

  // 初始化FOC算法
  motor1.initFOC(0, Direction::CW);
  motor2.initFOC(0, Direction::CW);
  ESP_LOGI(TAG, "FOC算法初始化成功");

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

    // 获取相电流
    PhaseCurrent_s phase_currents = current_sense1.getPhaseCurrents();
    status->current_u = phase_currents.a * 1000.0f; // 转换为毫安
    status->current_v = phase_currents.b * 1000.0f; // 转换为毫安
    status->current_w = phase_currents.c * 1000.0f; // 转换为毫安

    // 获取DQ轴电流
    DQCurrent_s dq_current =
        current_sense1.getFOCCurrents(motor1.electrical_angle);
    status->current_d = dq_current.d * 1000.0f; // 转换为毫安
    status->current_q = dq_current.q * 1000.0f; // 转换为毫安
  } else {
    current_rad_per_sec = motor2.shaft_velocity;

    // 获取相电流
    PhaseCurrent_s phase_currents = current_sense2.getPhaseCurrents();
    status->current_u = phase_currents.a * 1000.0f; // 转换为毫安
    status->current_v = phase_currents.b * 1000.0f; // 转换为毫安
    status->current_w = phase_currents.c * 1000.0f; // 转换为毫安

    // 获取DQ轴电流
    DQCurrent_s dq_current =
        current_sense2.getFOCCurrents(motor2.electrical_angle);
    status->current_d = dq_current.d * 1000.0f; // 转换为毫安
    status->current_q = dq_current.q * 1000.0f; // 转换为毫安
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
    // 更新电流传感 - InlineCurrentSense类没有update方法，
    // 而是在getPhaseCurrents()中自动更新
    current_sense1.getPhaseCurrents();
    current_sense2.getPhaseCurrents();

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

// 初始化电流传感器
static void init_current_sensor(void) {
  ESP_LOGI(TAG, "Initializing current sensors");

  // 电流传感器已经在全局定义并初始化了，这里不需要重新创建
  // 只需要确保它们已经链接到驱动器
  if (current_sense1.init()) {
    ESP_LOGI(TAG, "Current sensor 1 initialized successfully");
    current_sense1.linkDriver((BLDCDriver *)&driver1);
    current_sense1.skip_align = true; // 跳过电流传感器对齐
    ESP_LOGI(TAG, "Current sensor 1 calibration done!");
  } else {
    ESP_LOGE(TAG, "Failed to initialize current sensor 1");
  }

  if (current_sense2.init()) {
    ESP_LOGI(TAG, "Current sensor 2 initialized successfully");
    current_sense2.linkDriver((BLDCDriver *)&driver2);
    current_sense2.skip_align = true; // 跳过电流传感器对齐
    ESP_LOGI(TAG, "Current sensor 2 calibration done!");
  } else {
    ESP_LOGE(TAG, "Failed to initialize current sensor 2");
  }
}

// 获取当前电流值
void motor_control_foc_get_current(float *ia1, float *ib1, float *ic1,
                                   float *ia2, float *ib2, float *ic2) {
  // 使用current_sense1来获取电流值
  PhaseCurrent_s currents = current_sense1.getPhaseCurrents();
  *ia1 = currents.a;
  *ib1 = currents.b;
  *ic1 = currents.c;
  // 使用current_sense2来获取电流值
  PhaseCurrent_s currents2 = current_sense2.getPhaseCurrents();
  *ia2 = currents2.a;
  *ib2 = currents2.b;
  *ic2 = currents2.c;
}
