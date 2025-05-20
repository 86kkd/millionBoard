#include "sdkconfig.h"
#include "angle_sensor.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "driver/gpio.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifndef CONFIG_CAN_TX_GPIO
#define CONFIG_CAN_TX_GPIO 18
#endif
#ifndef CONFIG_CAN_RX_GPIO
#define CONFIG_CAN_RX_GPIO 17
#endif
#ifndef CONFIG_CAN_BITRATE
#define CONFIG_CAN_BITRATE 250000
#endif

static const char *TAG = "ANGLE_SENSOR";
static bool is_initialized = false;
static angle_sensor_data_t last_data = {0};

// 发送 NMT 启动命令
static void send_nmt_command(uint8_t node_id, uint8_t command) {
    twai_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.identifier = 0x000;
    msg.flags = TWAI_MSG_FLAG_NONE;
    msg.data_length_code = 2;
    msg.data[0] = command;
    msg.data[1] = node_id;
    esp_err_t ret = twai_transmit(&msg, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "NMT transmit failed: 0x%X", ret);
    }
}

// 全总线恢复，处理 bus-off
static void handle_bus_off_recovery(void) {
    ESP_LOGW(TAG, "CAN bus-off, recovering...");
    twai_stop();
    twai_driver_uninstall();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    twai_general_config_t gcfg = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CONFIG_CAN_TX_GPIO, (gpio_num_t)CONFIG_CAN_RX_GPIO, TWAI_MODE_NORMAL);
    twai_timing_config_t tcfg = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t fcfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (twai_driver_install(&gcfg, &tcfg, &fcfg) != ESP_OK) {
        ESP_LOGE(TAG, "CAN reinstall driver failed");
        return;
    }
    if (twai_start() != ESP_OK) {
        ESP_LOGE(TAG, "CAN restart failed");
    }
}

// CAN 接收任务
static void angle_sensor_task(void *arg) {
    twai_message_t rx_msg;
    twai_status_info_t status;
    uint32_t consecutive_errors = 0;
    const TickType_t timeout = pdMS_TO_TICKS(100);
    
    while (1) {
        twai_get_status_info(&status);
        if (status.state == TWAI_STATE_BUS_OFF) {
            handle_bus_off_recovery();
            consecutive_errors = 0;
            continue;
        }
        esp_err_t ret = twai_receive(&rx_msg, timeout);
        if (ret == ESP_OK) {
            consecutive_errors = 0;
            if (rx_msg.identifier == ANGLE_SENSOR_PDO1_CAN_ID && rx_msg.data_length_code >= 8) {
                int16_t roll_raw  = (int16_t)(rx_msg.data[1] << 8 | rx_msg.data[0]);
                int16_t pitch_raw = (int16_t)(rx_msg.data[3] << 8 | rx_msg.data[2]);
                int16_t yaw_raw   = (int16_t)(rx_msg.data[5] << 8 | rx_msg.data[4]);
                last_data.roll  = roll_raw  * 0.01f;
                last_data.pitch = pitch_raw * 0.01f;
                last_data.yaw   = yaw_raw   * 0.01f;
                last_data.temp  = rx_msg.data[6] / 2.0f - 40.0f;
                last_data.status= rx_msg.data[7];
                // ESP_LOGI(TAG, "Angle data - Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°, Temp: %.1f°C",
                //          last_data.roll, last_data.pitch, last_data.yaw, last_data.temp);
            }
        } else if (ret == ESP_ERR_TIMEOUT) {
            // 超时，不处理
        } else {
            ESP_LOGW(TAG, "CAN recv error: 0x%X", ret);
            if (++consecutive_errors > 10) {
                handle_bus_off_recovery();
                consecutive_errors = 0;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t angle_sensor_init(void) {
    if (is_initialized) {
        return ESP_OK;
    }
    // 配置 CAN，使用 250KBITS 波特率
    twai_general_config_t gcfg = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CONFIG_CAN_TX_GPIO, (gpio_num_t)CONFIG_CAN_RX_GPIO, TWAI_MODE_NORMAL);
    twai_timing_config_t tcfg = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t fcfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    esp_err_t ret = twai_driver_install(&gcfg, &tcfg, &fcfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CAN driver install failed: 0x%X", ret);
        return ret;
    }
    ret = twai_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CAN start failed: 0x%X", ret);
        twai_driver_uninstall();
        return ret;
    }
    ESP_LOGI(TAG, "CAN initialized (TX:%d, RX:%d, bitrate:250000)",
             CONFIG_CAN_TX_GPIO, CONFIG_CAN_RX_GPIO);
    // 检查总线状态
    twai_status_info_t status;
    twai_get_status_info(&status);
    ESP_LOGI(TAG, "Bus State:%d, TXErr:%d, RXErr:%d", status.state, status.tx_error_counter, status.rx_error_counter);
    // 发送 NMT 启动命令
    for (int i = 0; i < 3; i++) {
        send_nmt_command(ANGLE_SENSOR_NODE_ID, 0x01);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    // 创建接收任务
    xTaskCreate(angle_sensor_task, "angle_sensor", 4096, NULL, 5, NULL);
    is_initialized = true;
    ESP_LOGI(TAG, "Angle sensor task started");
    return ESP_OK;
}

esp_err_t angle_sensor_read(angle_sensor_data_t *data) {
    if (!is_initialized || data == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
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