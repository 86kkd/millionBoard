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

static const char *TAG = "MAIN";
// Define UART pins if not already defined in Kconfig
#ifndef CONFIG_UART_TX_PIN
#define CONFIG_UART_TX_PIN 17 // Default TX pin
#endif

#ifndef CONFIG_UART_RX_PIN
#define CONFIG_UART_RX_PIN 16 // Default RX pin
#endif

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
static void create_angle_sensor_tasks(void);
static void create_nfc_task(void);
static void create_gps_task(void);
static void create_battery_task(void);
static void create_motor_task(void);
static void create_motor_monitor_task(void);

// Shared pointers for the tasks to use
static MotorControlFOC *g_motor_control = nullptr;

// Task functions
extern "C" void angle_sensor_task(void *pvParameters) {
  while (1) {
    // Read angle sensor data through the C interface
    angle_sensor_data_t angle_data;
    if (angle_sensor_read(&angle_data) == ESP_OK) {
      board_state.board_angle = angle_data.pitch;
      ESP_LOGI(TAG, "Board Angle: %.1f°", board_state.board_angle);
    }

    vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz
  }
}

extern "C" void battery_monitor_task(void *pvParameters) {
  while (1) {
    // Read battery status using C interface
    battery_status_t battery_status;
    if (battery_mgr_get_status(&battery_status) == ESP_OK) {
      board_state.battery_voltage = battery_status.voltage;
      board_state.battery_current = battery_status.current;
      board_state.battery_percentage = battery_status.percentage;
      board_state.is_charging = battery_status.is_charging;

      ESP_LOGI(TAG, "Battery: %.1fV, %.1fA, %d%%, %s", battery_status.voltage,
               battery_status.current, battery_status.percentage,
               battery_status.is_charging ? "Charging" : "Discharging");

      // Low battery warning
      if (battery_status.percentage < 15 && !battery_status.is_charging) {
        ESP_LOGW(TAG, "Low battery warning!");
      }
    }

    vTaskDelay(pdMS_TO_TICKS(1000)); // 1Hz
  }
}

extern "C" void motor_control_task(void *pvParameters) {
  if (!g_motor_control) {
    ESP_LOGE(TAG, "Motor control not initialized!");
    vTaskDelete(NULL);
    return;
  }

  while (1) {
    // Only run motors if the board is unlocked
    if (!board_state.is_locked) {
      // Calculate motor output
      float motor_output = board_state.target_speed;

      // Apply incline compensation
      float incline_compensation =
          g_motor_control->calculateInclineCompensation(
              board_state.board_angle);
      motor_output += incline_compensation;

      // Set both motors to the same speed
      g_motor_control->setDualSpeed(motor_output, motor_output);

      // ESP_LOGI(TAG, "Motors Speed: %.1f, Compensation: %.1f", motor_output,
      //          incline_compensation);
      float ia1, ib1, ic1, ia2, ib2, ic2;
      g_motor_control->getCurrents(&ia1, &ib1, &ic1, &ia2, &ib2, &ic2);
      // ESP_LOGI(TAG, "Currents: %.1f, %.1f, %.1f, %.1f, %.1f, %.1f", ia1, ib1,
      //          ic1, ia2, ib2, ic2);
    } else {
      // Board is locked, stop motors
      g_motor_control->setDualSpeed(0, 0);
    }

    // Update FOC algorithm
    g_motor_control->update();

    vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz
  }
}

extern "C" void nfc_task(void *pvParameters) {
  while (1) {
    // Check for NFC card using C interface
    uint8_t uid[10];
    uint8_t uid_len;

    if (nfc_read_passive_target(uid, &uid_len) == ESP_OK) {
      ESP_LOGI(TAG, "NFC card detected!");

      // Check if card is authorized
      if (nfc_check_authorized(uid, uid_len)) {
        // Toggle lock state
        board_state.is_locked = !board_state.is_locked;
        ESP_LOGI(TAG, "Board %s",
                 board_state.is_locked ? "LOCKED" : "UNLOCKED");
      }
    }

    vTaskDelay(pdMS_TO_TICKS(500)); // 2Hz
  }
}

extern "C" void gps_task(void *pvParameters) {
  while (1) {
    // Get GPS data using C interface
    gps_data_t gps_data;
    if (gps_get_location(&gps_data) == ESP_OK && gps_data.valid) {
      ESP_LOGI(TAG, "GPS: Lat: %.6f, Lon: %.6f, Speed: %.1f km/h",
               gps_data.latitude, gps_data.longitude, gps_data.speed_kmh);
    }

    vTaskDelay(pdMS_TO_TICKS(1000)); // 1Hz
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
    ESP_LOGI(TAG,
             "Motor Mon: Mech=%.2f°, Elec=%.2f°, I_total=[%.2f, %.2f] mA, "
             "Speed=%.2f",
             mech * RAD2DEG, elec * RAD2DEG, i1_total, i2_total,
             board_state.current_speed);
    vTaskDelay(pdMS_TO_TICKS(200)); // 5Hz
  }
}

// Task creation callbacks
static void create_angle_sensor_tasks(void) {
  xTaskCreate(angle_sensor_task, "angle_sensor", 4096, NULL, 5, NULL);
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
  can_comm_init();

  // Configure UART parameters
  uart_config_t uart_config = {.baud_rate = 115200,
                               .data_bits = UART_DATA_8_BITS,
                               .parity = UART_PARITY_DISABLE,
                               .stop_bits = UART_STOP_BITS_1,
                               .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                               .rx_flow_ctrl_thresh = 0,
                               .source_clk = UART_SCLK_DEFAULT};

  uart_comm_init(UART_NUM_1, &uart_config, CONFIG_UART_TX_PIN,
                 CONFIG_UART_RX_PIN);

  // 2. Initialize pressure sensors using the static component API
  // This will create the sensors and start the pressure sensor task
  esp_err_t ret_pressure = PressureSensor::Init();
  if (ret_pressure != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize pressure sensors");
  } else {
    // Register callback to receive pressure updates
    PressureSensor::RegisterCallback(pressure_data_callback);
  }

  // 3. Initialize other components using C interfaces
  init_component("Angle sensor", angle_sensor_init, create_angle_sensor_tasks);
  init_component("NFC", nfc_init, create_nfc_task);
  init_component("GPS", gps_init, create_gps_task);
  init_component("Battery manager", battery_mgr_init, create_battery_task);

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

  ESP_LOGI(TAG, "Skateboard control system started!");
}