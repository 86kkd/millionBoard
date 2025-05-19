#include "angle_sensor.h"
#include "can_comm.h"
#include "esp_log.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ANGLE_SENSOR";

static bool is_initialized = false;
static angle_sensor_data_t last_data = {0};

// Task to poll CAN for angle sensor data
static void angle_sensor_task(void *pvParameters) {
    uint32_t id;
    uint8_t data_buf[8];
    uint8_t len;
    while (1) {
        if (can_comm_receive(&id, data_buf, &len, pdMS_TO_TICKS(100)) == ESP_OK) {
            if (id == CAN_ID_ANGLE_SENSOR_DATA && len >= 8) {
                int16_t roll_raw = (int16_t)(data_buf[1] << 8 | data_buf[0]);
                int16_t pitch_raw = (int16_t)(data_buf[3] << 8 | data_buf[2]);
                int16_t yaw_raw = (int16_t)(data_buf[5] << 8 | data_buf[4]);
                last_data.roll = roll_raw * 0.01f;
                last_data.pitch = pitch_raw * 0.01f;
                last_data.yaw = yaw_raw * 0.01f;
                last_data.temp = data_buf[6] / 2.0f - 40.0f;
                last_data.status = data_buf[7];
                ESP_LOGI(TAG, "Angle data - Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°, Temp: %.1f°C", last_data.roll, last_data.pitch, last_data.yaw, last_data.temp);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t angle_sensor_init(void) {
    if (!CONFIG_ENABLE_ANGLE_SENSOR) {
        ESP_LOGW(TAG, "Angle sensor support is disabled");
        return ESP_ERR_NOT_SUPPORTED;
    }
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

    // Send command to wake up the sensor (using NMT)
    uint8_t nmt[2] = {0x01, 0x05};
    if (can_comm_send(0x000, nmt, 2, 100) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send NMT start command");
    }
    // Create task to poll sensor data
    xTaskCreate(angle_sensor_task, "angle_sensor", 4096, NULL, 5, NULL);

    is_initialized = true;
    ESP_LOGI(TAG, "Angle sensor initialized successfully");

    return ESP_OK;
}

esp_err_t angle_sensor_read(angle_sensor_data_t *data) {
    if (!is_initialized || data == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // Copy the last received data
    memcpy(data, &last_data, sizeof(angle_sensor_data_t));

    return ESP_OK;
}

esp_err_t angle_sensor_calibrate(void) {
    if (!is_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Calibrating angle sensor");

    // In a real implementation, we would send a calibration command to the sensor
    // For this example, we'll just reset the reference values

    // Read current values
    angle_sensor_data_t current;
    esp_err_t ret = angle_sensor_read(&current);
    if (ret != ESP_OK) {
        return ret;
    }

    // Store these as zero reference (would typically be sent to the sensor)
    ESP_LOGI(TAG, "Zero reference set to Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°",
             current.roll, current.pitch, current.yaw);

    return ESP_OK;
}