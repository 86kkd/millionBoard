#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/twai.h"

static const char *TAG = "CAN_MONITOR";

#define NODE_ID 0

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

extern "C" void app_main(void) {
    // 配置TWAI - 使用100kbps
    twai_general_config_t g_config = {
        .mode = TWAI_MODE_NORMAL,
        .tx_io = GPIO_NUM_3,
        .rx_io = GPIO_NUM_8,
        .clkout_io = TWAI_IO_UNUSED,
        .bus_off_io = TWAI_IO_UNUSED,
        .tx_queue_len = 10,
        .rx_queue_len = 32,
        .alerts_enabled = TWAI_ALERT_ALL,
        .clkout_divider = 0,
    };
    
    // twai_timing_config_t t_config = TWAI_TIMING_CONFIG_100KBITS();
    // twai_timing_config_t t_config = TWAI_TIMING_CONFIG_125KBITS();
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
    // twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
    // 安装TWAI驱动
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "TWAI driver installed and started in listen-only mode at 100kbps");
    
    send_nmt_command(NODE_ID, 0x01);

    // 循环监听总线上的所有消息
    while (1) {
        twai_message_t rx_msg;
        esp_err_t recv_res = twai_receive(&rx_msg, pdMS_TO_TICKS(2000));
        if (recv_res == ESP_OK) {
            ESP_LOGI(TAG, "Message received - ID: 0x%03X, DLC: %d, Data: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
                    rx_msg.identifier,
                    rx_msg.data_length_code,
                    rx_msg.data[0], rx_msg.data[1], rx_msg.data[2], rx_msg.data[3],
                    rx_msg.data[4], rx_msg.data[5], rx_msg.data[6], rx_msg.data[7]);
        }else if (recv_res == ESP_ERR_TIMEOUT) {
            ESP_LOGI(TAG, "Timeout waiting for message");
        }else {
            ESP_LOGE(TAG, "Error receiving message: %s", esp_err_to_name(recv_res));
        }
    }
}


