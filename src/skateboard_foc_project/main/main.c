#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <math.h>
#include <stdio.h>

// Define UART pins if not already defined in Kconfig
#ifndef CONFIG_UART_TX_PIN
#define CONFIG_UART_TX_PIN 17 // Default TX pin
#endif

#ifndef CONFIG_UART_RX_PIN
#define CONFIG_UART_RX_PIN 16 // Default RX pin
#endif

// 包含各模块的头文件
#include "angle_sensor.h"
#include "battery_mgr.h"
#include "can_comm.h"
#include "debug.h"
#include "gps.h"
#include "i2c_comm.h"
#include "math_utils.h"
#include "motor_control_foc.hpp"
#include "nfc.h"
#include "pressure_sensor.h"
#include "uart_comm.h"

static const char *TAG = "MAIN";

// 定义前后传感器句柄
hx711_handle_t front_sensor = NULL;
hx711_handle_t rear_sensor = NULL;

// 定义全局状态变量
typedef struct {
  float front_pressure;
  float rear_pressure;
  float board_angle;
  float target_speed;
  float current_speed;
  float battery_voltage;
  float battery_current;
  uint8_t battery_percentage;
  bool is_charging;
  bool is_moving;
  bool is_locked;
} skateboard_state_t;

skateboard_state_t board_state = {0};

// 传感器采集任务
void pressure_sensor_task(void *pvParameters) {
  while (1) {
    // 读取前后脚踩压力
    board_state.front_pressure = pressure_sensor_get_weight(front_sensor);
    board_state.rear_pressure = pressure_sensor_get_weight(rear_sensor);

    // 计算重心和目标速度
    float weight_diff = board_state.front_pressure - board_state.rear_pressure;
    float total_weight = board_state.front_pressure + board_state.rear_pressure;

    if (total_weight > 100.0f) { // 确认有人站在板上
      // 计算重心百分比位置 (-1.0 到 1.0)
      float balance_point = weight_diff / total_weight;

      // 根据重心位置计算目标速度
      board_state.target_speed = balance_point * CONFIG_MAX_SPEED;
      board_state.is_moving = (fabs(board_state.target_speed) > 0.5f);
    } else {
      // 没有人站在板上，停止
      board_state.target_speed = 0.0f;
      board_state.is_moving = false;
    }

    ESP_LOGI(TAG, "Pressure - Front: %.1f, Rear: %.1f, Target Speed: %.1f",
             board_state.front_pressure, board_state.rear_pressure,
             board_state.target_speed);

    vTaskDelay(pdMS_TO_TICKS(50)); // 20Hz采样率
  }
}

// 倾角传感器任务
void angle_sensor_task(void *pvParameters) {
  while (1) {
    // 通过CAN读取倾角传感器数据
    angle_sensor_data_t angle_data;
    if (angle_sensor_read(&angle_data) == ESP_OK) {
      board_state.board_angle = angle_data.pitch;

      // 记录日志
      ESP_LOGI(TAG, "Board Angle: %.1f°", board_state.board_angle);
    }

    vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz
  }
}

// 电池监控任务
void battery_monitor_task(void *pvParameters) {
  while (1) {
    // 读取电池状态
    battery_status_t battery_status;
    if (battery_mgr_get_status(&battery_status) == ESP_OK) {
      board_state.battery_voltage = battery_status.voltage;
      board_state.battery_current = battery_status.current;
      board_state.battery_percentage = battery_status.percentage;
      board_state.is_charging = battery_status.is_charging;

      ESP_LOGI(TAG, "Battery: %.1fV, %.1fA, %d%%, %s", battery_status.voltage,
               battery_status.current, battery_status.percentage,
               battery_status.is_charging ? "Charging" : "Discharging");

      // 低电量警告
      if (battery_status.percentage < 15 && !battery_status.is_charging) {
        ESP_LOGW(TAG, "Low battery warning!");
      }
    }

    vTaskDelay(pdMS_TO_TICKS(1000)); // 1Hz
  }
}

// 电机控制任务
void motor_control_task(void *pvParameters) {
  while (1) {
    // 只有解锁状态才运行电机
    if (!board_state.is_locked) {
      // 计算电机控制输出
      float motor_output = board_state.target_speed;

      // 根据倾角补偿
      float incline_compensation =
          calculate_incline_compensation(board_state.board_angle);
      motor_output += incline_compensation;

      // 应用双电机控制 - 同样的速度设置给两个电机
      motor_control_set_dual_speed(motor_output, motor_output);

      ESP_LOGI(TAG, "Motors Speed: %.1f, Compensation: %.1f", motor_output,
               incline_compensation);
    } else {
      // 锁定状态，停止电机
      motor_control_set_dual_speed(0, 0);
    }

    // 运行FOC算法更新
    motor_control_update();

    vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz
  }
}

// NFC任务
void nfc_task(void *pvParameters) {
  while (1) {
    // 检查是否有NFC卡
    uint8_t uid[10];
    uint8_t uid_len;

    if (nfc_read_passive_target(uid, &uid_len) == ESP_OK) {
      ESP_LOGI(TAG, "NFC card detected!");

      // 检查卡是否授权
      if (nfc_check_authorized(uid, uid_len)) {
        // 切换锁定状态
        board_state.is_locked = !board_state.is_locked;
        ESP_LOGI(TAG, "Board %s",
                 board_state.is_locked ? "LOCKED" : "UNLOCKED");
      }
    }

    vTaskDelay(pdMS_TO_TICKS(500)); // 2Hz
  }
}

// GPS任务
void gps_task(void *pvParameters) {
  while (1) {
    // 获取GPS数据
    gps_data_t gps_data;
    if (gps_get_location(&gps_data) == ESP_OK && gps_data.valid) {
      ESP_LOGI(TAG, "GPS: Lat: %.6f, Lon: %.6f, Speed: %.1f km/h",
               gps_data.latitude, gps_data.longitude, gps_data.speed_kmh);
    }

    vTaskDelay(pdMS_TO_TICKS(1000)); // 1Hz
  }
}

// Helper function for component initialization
static bool init_component(const char *name, esp_err_t (*init_func)(void),
                           void (*success_callback)(void)) {
  ESP_LOGI(TAG, "Initializing %s...", name);

  esp_err_t ret = init_func();
  if (ret == ESP_ERR_NOT_SUPPORTED) {
    ESP_LOGW(TAG, "%s disabled, continuing without it", name);
    return false;
  } else if (ret != ESP_OK) {
    ESP_LOGE(TAG, "%s init failed with error %d", name, ret);
    return false;
  }

  ESP_LOGI(TAG, "%s initialized successfully", name);
  if (success_callback) {
    success_callback();
  }
  return true;
}

// Task creation callbacks
static void create_angle_sensor_tasks(void) {
  xTaskCreate(angle_sensor_task, "angle_sensor", 4096, NULL, 5, NULL);
  xTaskCreate(pressure_sensor_task, "pressure_sensor", 4096, NULL, 5, NULL);
}

static void create_nfc_task(void) {
  xTaskCreate(nfc_task, "nfc", 4096, NULL, 4, NULL);
}

static void create_gps_task(void) {
  xTaskCreate(gps_task, "gps", 4096, NULL, 2, NULL);
}

static void create_battery_task(void) {
  xTaskCreate(battery_monitor_task, "battery_monitor", 4096, NULL, 3, NULL);
}

static void create_motor_task(void) {
  xTaskCreate(motor_control_task, "motor_control", 4096, NULL, 10, NULL);
}

// Add this function above app_main
static esp_err_t motor_enable_wrapper(void) {
  return motor_control_enable(MOTOR_ID_PRIMARY);
}

void app_main(void) {
  // 初始化NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_LOGI(TAG, "Skateboard control system initializing...");

  // 初始化各模块

  // 1. 通信模块初始化
  i2c_comm_init();
  can_comm_init();
  // Configure UART parameters
  uart_config_t uart_config = {.baud_rate = 115200,
                               .data_bits = UART_DATA_8_BITS,
                               .parity = UART_PARITY_DISABLE,
                               .stop_bits = UART_STOP_BITS_1,
                               .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
  uart_comm_init(UART_NUM_1, &uart_config, CONFIG_UART_TX_PIN,
                 CONFIG_UART_RX_PIN);

  // 2. 传感器初始化
  pressure_sensor_config_t front_config = {
      .dout_gpio = CONFIG_HX711_FRONT_DOUT_GPIO,
      .sck_gpio = CONFIG_HX711_FRONT_SCK_GPIO,
      .gain = HX711_GAIN_128_A,
  };

  pressure_sensor_config_t rear_config = {
      .dout_gpio = CONFIG_HX711_REAR_DOUT_GPIO,
      .sck_gpio = CONFIG_HX711_REAR_SCK_GPIO,
      .gain = HX711_GAIN_128_A,
  };

  ESP_ERROR_CHECK(pressure_sensor_init(&front_config, &front_sensor));
  ESP_ERROR_CHECK(pressure_sensor_init(&rear_config, &rear_sensor));

  // 3) Immediately pull your saved calibration out of NVS
  ESP_ERROR_CHECK(
      pressure_sensor_load_calibration(front_sensor, CONFIG_HX711_NVS_NAMESPACE,
                                       CONFIG_HX711_NVS_KEY_PREFIX_FRONT));
  ESP_ERROR_CHECK(
      pressure_sensor_load_calibration(rear_sensor, CONFIG_HX711_NVS_NAMESPACE,
                                       CONFIG_HX711_NVS_KEY_PREFIX_REAR));

  // Initialize all components with a clean pattern
  init_component("Angle sensor", angle_sensor_init, create_angle_sensor_tasks);
  init_component("NFC", nfc_init, create_nfc_task);
  init_component("GPS", gps_init, create_gps_task);
  init_component("Battery manager", battery_mgr_init, create_battery_task);

  // For motor control, we have two steps
  if (init_component("Motor control", motor_control_init, NULL)) {
    // Only try to enable if init succeeded
    init_component("Motor enable", motor_enable_wrapper, create_motor_task);
  }

  ESP_LOGI(TAG, "Skateboard control system started!");
}