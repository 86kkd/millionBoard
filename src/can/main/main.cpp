#include "driver/gpio.h"
#include "driver/twai.h" // Changed from can.h to twai.h
#include "esp_err.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "CAN_EXAMPLE";

// CAN configuration
#define CAN_TX_GPIO GPIO_NUM_5                      // CTX
#define CAN_RX_GPIO GPIO_NUM_4                      // CRX
#define CAN_BAUD_RATE TWAI_TIMING_CONFIG_500KBITS() // New timing config format

// Sensor configuration
#define NODE_ID 0x05
#define T_PDO1_CAN_ID (0x180 + NODE_ID)

void can_init(void);
void can_receive_task(void *arg);
void parse_pdo1_data(uint8_t *data);

extern "C" void app_main(void) {
  can_init();
  xTaskCreate(can_receive_task, "CAN_Receive_Task", 4096, NULL, 5, NULL);
}

void can_init(void) {
  // TWAI配置结构体初始化
  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);

  twai_timing_config_t t_config = CAN_BAUD_RATE; // 使用新的时序配置
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // 安装驱动
  esp_err_t ret = twai_driver_install(&g_config, &t_config, &f_config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Driver install failed: 0x%x", ret);
    return;
  }

  // 启动CAN控制器
  if ((ret = twai_start()) != ESP_OK) {
    ESP_LOGE(TAG, "Start failed: 0x%x", ret);
    return;
  }
  ESP_LOGI(TAG, "CAN Started");
}

void can_receive_task(void *arg) {
  twai_message_t rx_msg; // 改用twai_message_t

  while (1) {
    esp_err_t ret = twai_receive(&rx_msg, pdMS_TO_TICKS(1000));

    if (ret == ESP_OK) {
      if (rx_msg.identifier == T_PDO1_CAN_ID && rx_msg.data_length_code == 8) {
        ESP_LOGI(TAG, "Received T_PDO1");
        parse_pdo1_data(rx_msg.data);
      }
    } else if (ret != ESP_ERR_TIMEOUT) {
      ESP_LOGE(TAG, "Receive error: 0x%x", ret);
    } else {
      ESP_LOGI(TAG, "Receive timeout");
    }
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

  ESP_LOGI(TAG, "Roll:%.2f° Pitch:%.2f° Yaw:%.2f° Temp:%.1fC Status:0x%02X",
           roll, pitch, yaw, temp, data[7]);
}
