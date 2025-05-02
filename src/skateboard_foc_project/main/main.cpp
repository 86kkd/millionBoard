#include <cmath>
#include <cstdio>

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

// Define pins for HX711 sensors
static const gpio_num_t kFrontScaleClockPin =
    static_cast<gpio_num_t>(CONFIG_HX711_FRONT_SCK_GPIO);
static const gpio_num_t kFrontScaleDataPin =
    static_cast<gpio_num_t>(CONFIG_HX711_FRONT_DOUT_GPIO);
static const gpio_num_t kRearScaleClockPin =
    static_cast<gpio_num_t>(CONFIG_HX711_REAR_SCK_GPIO);
static const gpio_num_t kRearScaleDataPin =
    static_cast<gpio_num_t>(CONFIG_HX711_REAR_DOUT_GPIO);

// Create global C++ objects as pointers
HX711 *hx711_front = nullptr;
HX711 *hx711_rear = nullptr;
PressureSensor *pressure_sensor_front = nullptr;
PressureSensor *pressure_sensor_rear = nullptr;

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

// Forward declare the C++ motor control object
MotorControlFOC *motor_control = nullptr;

// Task functions
extern "C" void pressure_sensor_task(void *pvParameters) {
  while (1) {
    // Read pressures from both sensors
    board_state.front_pressure = pressure_sensor_front->GetWeight(5);
    board_state.rear_pressure = pressure_sensor_rear->GetWeight(5);

    // Calculate center of gravity and target speed
    float weight_diff = board_state.front_pressure - board_state.rear_pressure;
    float total_weight = board_state.front_pressure + board_state.rear_pressure;

    if (total_weight > 100.0f) { // Confirm someone is on the board
      // Calculate balance point percentage (-1.0 to 1.0)
      float balance_point = weight_diff / total_weight;

      // Calculate target speed based on balance
      board_state.target_speed = balance_point * CONFIG_MAX_SPEED;
      board_state.is_moving = (fabs(board_state.target_speed) > 0.5f);
    } else {
      // No one on the board, stop
      board_state.target_speed = 0.0f;
      board_state.is_moving = false;
    }

    ESP_LOGI(TAG, "Pressure - Front: %.1f, Rear: %.1f, Target Speed: %.1f",
             board_state.front_pressure, board_state.rear_pressure,
             board_state.target_speed);

    vTaskDelay(pdMS_TO_TICKS(50)); // 20Hz sampling rate
  }
}

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
  if (!motor_control) {
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
          motor_control->calculateInclineCompensation(board_state.board_angle);
      motor_output += incline_compensation;

      // Set both motors to the same speed
      motor_control->setDualSpeed(motor_output, motor_output);

      ESP_LOGI(TAG, "Motors Speed: %.1f, Compensation: %.1f", motor_output,
               incline_compensation);
    } else {
      // Board is locked, stop motors
      motor_control->setDualSpeed(0, 0);
    }

    // Update FOC algorithm
    motor_control->update();

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

  // 2. Initialize pressure sensors (C++ interface)
  // Add a delay to give hardware time to stabilize
  vTaskDelay(pdMS_TO_TICKS(100));

  // Create HX711 and PressureSensor objects
  ESP_LOGI(TAG, "Initializing HX711 sensors...");
  hx711_front = new HX711(kFrontScaleClockPin, kFrontScaleDataPin,
                          HX711::Mode::kChannelA128);
  hx711_rear = new HX711(kRearScaleClockPin, kRearScaleDataPin,
                         HX711::Mode::kChannelA128);

  ESP_LOGI(TAG, "Creating pressure sensor interfaces...");
  pressure_sensor_front = new PressureSensor(*hx711_front);
  pressure_sensor_rear = new PressureSensor(*hx711_rear);

  // Load HX711 sensor calibration from NVS
  bool front_loaded = pressure_sensor_front->LoadCalibration(
      CONFIG_HX711_NVS_NAMESPACE, CONFIG_HX711_NVS_KEY_PREFIX_FRONT);
  bool rear_loaded = pressure_sensor_rear->LoadCalibration(
      CONFIG_HX711_NVS_NAMESPACE, CONFIG_HX711_NVS_KEY_PREFIX_REAR);

  // Report pressure sensor status
  ESP_LOGI(TAG, "Front pressure sensor: %s, offset=%.2f, scale=%.2f",
           front_loaded ? "calibrated" : "not calibrated",
           pressure_sensor_front->GetOffset(),
           pressure_sensor_front->GetScale());

  ESP_LOGI(TAG, "Rear pressure sensor: %s, offset=%.2f, scale=%.2f",
           rear_loaded ? "calibrated" : "not calibrated",
           pressure_sensor_rear->GetOffset(), pressure_sensor_rear->GetScale());

  // 3. Initialize other components using C interfaces
  init_component("Angle sensor", angle_sensor_init, create_angle_sensor_tasks);
  init_component("NFC", nfc_init, create_nfc_task);
  init_component("GPS", gps_init, create_gps_task);
  init_component("Battery manager", battery_mgr_init, create_battery_task);

  // 4. Initialize motor control (C++ interface)
  motor_control = new MotorControlFOC();
  if (motor_control->init() == ESP_OK) {
    ESP_LOGI(TAG, "Motor control initialized successfully");
    if (motor_control->enable() == ESP_OK) {
      ESP_LOGI(TAG, "Motors enabled successfully");
      create_motor_task();
    } else {
      ESP_LOGE(TAG, "Failed to enable motors");
    }
  } else {
    ESP_LOGE(TAG, "Failed to initialize motor control");
  }

  ESP_LOGI(TAG, "Skateboard control system started!");
}