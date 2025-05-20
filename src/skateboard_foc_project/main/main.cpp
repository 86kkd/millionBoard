#include <math.h>
#include <stdio.h>

// System includes
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

// C++ component includes
#include "motor_control_foc.hpp"
#include "pressure_sensor.hpp"

// C-only component includes
#include "angle_sensor.h"
#include "battery_mgr.h"
#include "can_comm.h"
#include "debug.h"
#include "gps.h"
#include "i2c_comm.h"
#include "math_utils.h"
#include "nfc.h"
#include "uart_comm.h"
#include "olcd.h"

static const char *TAG = "MAIN";

// Global state for skateboard
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

// Initialize with all fields set to zero
skateboard_state_t board_state = {.front_pressure = 0.0f,
                                  .rear_pressure = 0.0f,
                                  .board_angle = 0.0f,
                                  .target_speed = 0.0f,
                                  .current_speed = 0.0f,
                                  .battery_voltage = 0.0f,
                                  .battery_current = 0.0f,
                                  .battery_percentage = 0,
                                  .is_charging = false,
                                  .is_moving = false,
                                  .is_locked = false};

// 全局LVGL显示数据数组：0=Fp,1=Rp,2=Angle,3=GPS Fix(1/0),4=Lock(1/0)
static float lcd_vals[5] = {0};
static olcd_data lcd_data = {0};

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

// We'll need these forward declarations
static void create_board_angle_task(void);
static void create_motor_task(void);
static void create_motor_monitor_task(void);

// Shared pointers for the tasks to use
static MotorControlFOC *g_motor_control = nullptr;

extern "C" void motor_control_task(void *pvParameters) {
  if (!g_motor_control) {
    ESP_LOGE(TAG, "Motor control not initialized!");
    vTaskDelete(NULL);
    return;
  }

  while (1) {
    // Only run motors if the board is unlocked
    if (!board_state.is_locked) {
      // 直接使用已归一化的目标速度并应用坡度补偿
      float norm_target = board_state.target_speed;
      // float comp = g_motor_control->calculateInclineCompensation(board_state.board_angle);
      // norm_target += comp;
      // 限幅到[-1,1]
      if (norm_target > 1.0f) norm_target = 1.0f;
      if (norm_target < -1.0f) norm_target = -1.0f;
      g_motor_control->setDualSpeed(norm_target, norm_target);

      float ia1, ib1, ic1, ia2, ib2, ic2;
      g_motor_control->getCurrents(&ia1, &ib1, &ic1, &ia2, &ib2, &ic2);
    } else {
      // Board is locked, stop motors
      g_motor_control->setDualSpeed(0, 0);
    }

    // Update FOC algorithm
    g_motor_control->update();

    vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz
  }
}

// Motor monitoring task: log mechanical/electrical angles and phase currents
extern "C" void motor_monitor_task(void *pvParameters) {
  if (!g_motor_control) {
    ESP_LOGE(TAG, "Motor control not initialized!");
    vTaskDelete(NULL);
    return;
  }
  float mech, elec;
  float ia1, ib1, ic1, ia2, ib2, ic2;
  motor_status_t status;
  const float RAD2DEG = 180.0f / M_PI;
  while (1) {
    // Read angles
    mech = g_motor_control->getMechanicalAngle(MOTOR_ID_SECONDARY);
    elec = g_motor_control->getElectricalAngle(MOTOR_ID_SECONDARY);
    // Read phase currents
    g_motor_control->getCurrents(&ia1, &ib1, &ic1, &ia2, &ib2, &ic2);
    // Get current speed for monitoring
    if (g_motor_control->getStatus(MOTOR_ID_SECONDARY, &status) == ESP_OK) {
      board_state.current_speed = status.current_speed;
    }
    // Calculate total RMS current in mA
    float i1_total = sqrtf((ia1 * ia1 + ib1 * ib1 + ic1 * ic1) / 3.0f);
    float i2_total = sqrtf((ia2 * ia2 + ib2 * ib2 + ic2 * ic2) / 3.0f);
    vTaskDelay(pdMS_TO_TICKS(200)); // 5Hz
  }
}

// Task to periodically log central system status
static void status_task(void *pvParameters) {
    battery_status_t batt;
    angle_sensor_data_t ang;
    gps_info_t gps;
    motor_status_t mstat;
    float ia1, ib1, ic1, ia2, ib2, ic2;
    float mech1, elec1, mech2, elec2;
    while (1) {
        // Battery status
        if (battery_mgr_get_status(&batt) != ESP_OK) {
            ESP_LOGW(TAG, "Failed to get battery status");
        }
        // Angle sensor
        if (angle_sensor_read(&ang) != ESP_OK) {
            ESP_LOGW(TAG, "Failed to read angle sensor");
        }
        // Motor currents & speed
        g_motor_control->getCurrents(&ia1, &ib1, &ic1, &ia2, &ib2, &ic2);
        if (g_motor_control->getStatus(MOTOR_ID_SECONDARY, &mstat) == ESP_OK) {
            board_state.current_speed = mstat.current_speed;
        }
        // 获取 GPS 信息
        if (pa1010d_gps_get_info(&gps) != ESP_OK) {
            ESP_LOGW(TAG, "Failed to get GPS info");
        }
        // 获取主、副电机角度
        mech1 = g_motor_control->getMechanicalAngle(MOTOR_ID_PRIMARY);
        elec1 = g_motor_control->getElectricalAngle(MOTOR_ID_PRIMARY);
        mech2 = g_motor_control->getMechanicalAngle(MOTOR_ID_SECONDARY);
        elec2 = g_motor_control->getElectricalAngle(MOTOR_ID_SECONDARY);
        // 归一化目标速度
        float norm_target = board_state.target_speed;
        float comp = g_motor_control->calculateInclineCompensation(board_state.board_angle);
        norm_target += comp;
        if (norm_target > 1.0f) norm_target = 1.0f;
        if (norm_target < -1.0f) norm_target = -1.0f;
        // Log all status
        ESP_LOGI(TAG,
            "STATUS | Pressure Fp:%.2f Rp:%.2f \n"
            "Angle AP:%.2f° AR:%.2f° \n"
            "GPS:%s Sats:%d Lat:%.6f Lon:%.6f \n"
            "Bat:%.2fV %.2fA %d%% %s \n"
            "Curr1:[%.2f,%.2f,%.2f] Curr2:[%.2f,%.2f,%.2f] \n"
            "MA1:%.2f° EA1:%.2f° MA2:%.2f° EA2:%.2f° Spd:%.2f Target:%.2f",
            board_state.front_pressure, board_state.rear_pressure,
            ang.pitch, ang.roll,
            gps.has_fix ? "Fix" : "NoFix", gps.num_satellites,
            gps.latitude, gps.longitude,
            batt.voltage, batt.current, batt.percentage,
            batt.is_charging ? "Chg" : "Dsg",
            ia1, ib1, ic1, ia2, ib2, ic2,
            mech1, elec1, mech2, elec2,
            board_state.current_speed, norm_target);
        vTaskDelay(pdMS_TO_TICKS(1000));
        // 更新LCD显示数组：总重量、俯仰、横滚
        lcd_vals[0] = (board_state.front_pressure + board_state.rear_pressure) / 1000.0f;
        lcd_vals[1] = ang.pitch;
        lcd_vals[2] = ang.roll;
        lcd_vals[3] = (gps.has_fix ? 1.0f : 0.0f);
        lcd_vals[4] = (board_state.is_locked ? 1.0f : 0.0f);
    }
}

static void create_motor_task(void) {
  xTaskCreate(motor_control_task, "motor_control", 4096, NULL, 10, NULL);
}

static void create_motor_monitor_task(void) {
  xTaskCreate(motor_monitor_task, "motor_monitor", 4096, NULL, 4, NULL);
}

// Callback from PressureSensor to update shared board_state
static void pressure_data_callback(float front_pressure, float rear_pressure,
                                   float target_speed, bool is_moving) {
  board_state.front_pressure = front_pressure;
  board_state.rear_pressure = rear_pressure;
  board_state.target_speed = target_speed;
  board_state.is_moving = is_moving;
}

extern "C" void app_main(void) {
  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_LOGI(TAG, "Skateboard control system initializing...");

  // 1. Initialize communication modules (using C interfaces)
  i2c_comm_init();

  // 3. Initialize other components using C interfaces

  nfc_init();

  // 4. Initialize motor control (C++ interface)
  g_motor_control = new MotorControlFOC();
  if (g_motor_control->init() == ESP_OK) {
    ESP_LOGI(TAG, "Motor control initialized successfully");
    if (g_motor_control->enable() == ESP_OK) {
      ESP_LOGI(TAG, "Motors enabled successfully");
      create_motor_task();
      create_motor_monitor_task();
    } else {
      ESP_LOGE(TAG, "Failed to enable motors");
    } 
  }

  // 启动 PA1010D I2C GPS 任务
  if (pa1010d_gps_init() == ESP_OK) {
    ESP_LOGI(TAG, "PA1010D GPS I2C 任务启动成功");
  } else {
    ESP_LOGE(TAG, "PA1010D GPS I2C 初始化失败");
  }

  angle_sensor_init();

    // 2. Initialize pressure sensors using the static component API
  // This will create the sensors and start the pressure sensor task
  esp_err_t ret_pressure = PressureSensor::Init();
  if (ret_pressure != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize pressure sensors");
  } else {
    // Register callback to receive pressure updates
    PressureSensor::RegisterCallback(pressure_data_callback);
  }

  // 创建集中式状态打印任务
  xTaskCreate(status_task, "status", 8192, NULL, 2, NULL);

  // 添加 OLCD 显示任务
  {
    // 初始化 OLCD 并创建显示线程
    lv_disp_t *disp = setup_olcd();
    lcd_data.disp = disp;
    lcd_data.data = lcd_vals;
    xTaskCreate(run_olcd, "olcd_task", 4096, &lcd_data, 5, NULL);
  }

  ESP_LOGI(TAG, "Skateboard control system started!");
}