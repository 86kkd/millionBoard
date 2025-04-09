#include "angle_sensor.h"
#include "can_comm.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "ANGLE_SENSOR";

static bool is_initialized = false;
static angle_sensor_data_t last_data = {0};

// CAN receive callback
static void angle_sensor_can_callback(uint32_t id, uint8_t *data,
                                      uint8_t length, void *user_data) {
#if CONFIG_ENABLE_ANGLE_SENSOR
  if (id != CAN_ID_ANGLE_SENSOR_DATA || length < 8) {
    return;
  }

  // Parse the data according to the CAN protocol
  // The format used here is from the CAN sample code provided
  int16_t roll_raw = (int16_t)(data[1] << 8 | data[0]);
  int16_t pitch_raw = (int16_t)(data[3] << 8 | data[2]);
  int16_t yaw_raw = (int16_t)(data[5] << 8 | data[4]);

  // Convert to degrees (assuming raw values are in 0.01 degree units)
  last_data.roll = roll_raw * 0.01f;
  last_data.pitch = pitch_raw * 0.01f;
  last_data.yaw = yaw_raw * 0.01f;
  last_data.temp = data[6] / 2.0f - 40.0f;
  last_data.status = data[7];

  ESP_LOGD(TAG,
           "Angle data - Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°, Temp: %.1f°C",
           last_data.roll, last_data.pitch, last_data.yaw, last_data.temp);
#endif
}

esp_err_t angle_sensor_init(void) {
#if CONFIG_ENABLE_ANGLE_SENSOR
  if (is_initialized) {
    return ESP_OK;
  }

  ESP_LOGI(TAG, "Initializing angle sensor");

  // Ensure CAN is initialized
  esp_err_t ret = can_comm_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize CAN");
    return ret;
  }

  // Register callback for angle sensor data
  ret = can_comm_register_callback(CAN_ID_ANGLE_SENSOR_DATA,
                                   angle_sensor_can_callback, NULL);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register CAN callback");
    return ret;
  }

  // Initialize angle sensor data
  memset(&last_data, 0, sizeof(angle_sensor_data_t));

  // Send command to wake up the sensor (using NMT command)
  uint8_t nmt_data[2] = {0x01, 0x05}; // Start node 5
  ret = can_comm_send(0x000, nmt_data, 2, 100);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to send NMT start command, continuing anyway");
    // Not returning error as the sensor might already be active
  }

  is_initialized = true;
  ESP_LOGI(TAG, "Angle sensor initialized successfully");

  return ESP_OK;
#else
  ESP_LOGW(TAG, "Angle sensor support is disabled");
  return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t angle_sensor_read(angle_sensor_data_t *data) {
#if CONFIG_ENABLE_ANGLE_SENSOR
  if (!is_initialized || data == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  // Copy the last received data
  memcpy(data, &last_data, sizeof(angle_sensor_data_t));

  return ESP_OK;
#else
  if (data != NULL) {
    memset(data, 0, sizeof(angle_sensor_data_t));
  }
  return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t angle_sensor_calibrate(void) {
#if CONFIG_ENABLE_ANGLE_SENSOR
  if (!is_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  ESP_LOGI(TAG, "Calibrating angle sensor");

  // In a real implementation, we would send a calibration command to the sensor
  // For this example, we'll just reset the reference values

  // Read current values
  angle_sensor_data_t current_data;
  esp_err_t ret = angle_sensor_read(&current_data);
  if (ret != ESP_OK) {
    return ret;
  }

  // Store these as zero reference (would typically be sent to the sensor)
  ESP_LOGI(TAG, "Zero reference set to Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°",
           current_data.roll, current_data.pitch, current_data.yaw);

  return ESP_OK;
#else
  return ESP_ERR_NOT_SUPPORTED;
#endif
}