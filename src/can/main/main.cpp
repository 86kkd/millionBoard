#include "driver/gpio.h"
#include "driver/twai.h" // Changed from can.h to twai.h
#include "esp_err.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "CAN_EXAMPLE";

// CAN configuration
#define CAN_TX_GPIO GPIO_NUM_18                     // CTX
#define CAN_RX_GPIO GPIO_NUM_17                     // CRX
#define CAN_BAUD_RATE TWAI_TIMING_CONFIG_250KBITS() // New timig config format

// Sensor configuration
#define NODE_ID 0x05
#define T_PDO1_CAN_ID (0x180 + NODE_ID)

void can_init(void);
void can_receive_task(void *arg);
void parse_pdo1_data(uint8_t *data);

extern "C" void app_main(void) {
  can_init();
  xTaskCreate(can_receive_task, "CAN_RX", 4096, NULL,
              configMAX_PRIORITIES - 3, // 提升至22（FreeRTOS默认25级）
              NULL);
}

// 在app_main中添加NMT网络唤醒命令
void send_nmt_command(uint8_t node_id, uint8_t command) {
  twai_message_t nmt_msg = {
      .flags = TWAI_MSG_FLAG_NONE, // Must come first due to union
      .identifier = 0x000,         // 11-bit identifier
      .data_length_code = 2,       // 2 bytes of data
      .data = {command, node_id}   // Data bytes
  };
  ESP_ERROR_CHECK(twai_transmit(&nmt_msg, portMAX_DELAY));
}

void can_init(void) {
  // Use official configuration macros
  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = CAN_BAUD_RATE;
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // 安装驱动
  esp_err_t ret = twai_driver_install(&g_config, &t_config, &f_config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "%s:%d Driver install failed: 0x%x", __FILE__, __LINE__, ret);
    return;
  }

  // 启动CAN控制器
  if ((ret = twai_start()) != ESP_OK) {
    ESP_LOGE(TAG, "%s:%d Start failed: 0x%x", __FILE__, __LINE__, ret);
    return;
  }
  ESP_LOGI(TAG, "%s:%d CAN Started", __FILE__, __LINE__);

  // Check initial bus status
  twai_status_info_t status_info;
  twai_get_status_info(&status_info);
  ESP_LOGI(TAG,
           "%s:%d Bus Status - State:%d, TX Errs:%d, RX Errs:%d, Msgs Tx:%d, "
           "Msgs Rx:%d",
           __FILE__, __LINE__, status_info.state, status_info.tx_error_counter,
           status_info.rx_error_counter, status_info.tx_failed_count,
           status_info.rx_missed_count);

  // Send NMT command with retry mechanism
  const int MAX_RETRIES = 3;
  for (int retry = 0; retry < MAX_RETRIES; retry++) {
    if (retry > 0) {
      ESP_LOGW(TAG, "%s:%d Retrying NMT command, attempt %d", __FILE__,
               __LINE__, retry + 1);
      vTaskDelay(pdMS_TO_TICKS(100 * (1 << retry))); // Exponential backoff
    }

    send_nmt_command(NODE_ID, 0x01);

    ESP_LOGI(TAG, "%s:%d messsge to tx: %d", __FILE__, __LINE__,
             status_info.msgs_to_tx);
    ESP_LOGI(TAG, "%s:%d messsge to rx: %d", __FILE__, __LINE__,
             status_info.msgs_to_rx);

    // Verify transmission
    twai_status_info_t post_tx_status;
    twai_get_status_info(&post_tx_status);
    if (post_tx_status.tx_failed_count == 0) {
      ESP_LOGI(TAG, "%s:%d NMT command sent successfully", __FILE__, __LINE__);
      break;
    }

    if (retry == MAX_RETRIES - 1) {
      ESP_LOGE(TAG, "%s:%d Failed to send NMT command after %d attempts",
               __FILE__, __LINE__, MAX_RETRIES);
    }
  }
}

// Add new function for bus recovery
void handle_bus_off_recovery(void) {
  ESP_LOGW(TAG, "%s:%d Initiating full bus reset procedure", __FILE__,
           __LINE__);

  // Full cleanup sequence
  twai_stop();
  vTaskDelay(pdMS_TO_TICKS(50));
  twai_driver_uninstall();        // Critical missing step
  vTaskDelay(pdMS_TO_TICKS(150)); // Allow full discharge

  // Reinitialize with fresh config
  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
  ESP_ERROR_CHECK(twai_start());

  ESP_LOGI(TAG, "%s:%d Bus reset complete", __FILE__, __LINE__);
}
// Update receive task with enhanced error handling
void can_receive_task(void *arg) {
  twai_message_t rx_msg;
  twai_status_info_t status;
  uint32_t consecutive_errors = 0;

  while (1) {
    // Monitor bus status periodically
    twai_get_status_info(&status);

    // Check for bus-off condition
    if (status.state == TWAI_STATE_BUS_OFF) {
      ESP_LOGW(TAG, "%s:%d Bus off recovery", __FILE__, __LINE__);
      handle_bus_off_recovery();
      consecutive_errors = 0;
      continue;
    }

    // Monitor error counters
    if (status.tx_error_counter > 16 || status.rx_error_counter > 16) {
      ESP_LOGW(TAG, "%s:%d High error counters - TX:%d, RX:%d", __FILE__,
               __LINE__, status.tx_error_counter, status.rx_error_counter);
    }

    esp_err_t ret = twai_receive(&rx_msg, pdMS_TO_TICKS(1000));
    if (ret == ESP_ERR_TIMEOUT) {
      ESP_LOGE(TAG, "%s:%d ret: twai receive timeout", __FILE__, __LINE__);
    }

    if (ret == ESP_OK) {
      consecutive_errors = 0;
      if (rx_msg.identifier == T_PDO1_CAN_ID && rx_msg.data_length_code == 8) {
        ESP_LOGD(TAG, "%s:%d Received T_PDO1", __FILE__, __LINE__);
        parse_pdo1_data(rx_msg.data);
      }
    } else if (ret != ESP_ERR_TIMEOUT) {
      consecutive_errors++;
      ESP_LOGE(TAG, "%s:%d Receive error: 0x%x (consecutive errors: %d)",
               __FILE__, __LINE__, ret, consecutive_errors);

      if (consecutive_errors > 10) {
        ESP_LOGE(TAG,
                 "%s:%d Too many consecutive errors, attempting bus recovery",
                 __FILE__, __LINE__);
        handle_bus_off_recovery();
        consecutive_errors = 0;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10)); // Prevent task starvation
  }
}

// 解析函数保持不变
void parse_pdo1_data(uint8_t *data) {
  int16_t roll_raw = (int16_t)(data[1] << 8 | data[0]);
  int16_t pitch_raw = (int16_t)(data[3] << 8 | data[2]);
  int16_t yaw_raw = (int16_t)(data[5] << 8 | data[4]);

  float roll = roll_raw * 0.01f;
  float pitch = pitch_raw * 0.01f;
  float yaw = yaw_raw * 0.01f;
  float temp = data[6] / 2.0f - 40.0f;

  ESP_LOGI(TAG,
           "%s:%d Roll:%.2f° Pitch:%.2f° Yaw:%.2f° Temp:%.1fC Status:0x%02X",
           __FILE__, __LINE__, roll, pitch, yaw, temp, data[7]);
}
