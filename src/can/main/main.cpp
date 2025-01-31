#include "config.h"
#include "driver/gpio.h"
#include "driver/twai.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/twai_types.h"
#include "sdkconfig.h"
#include "utils.h"
#include <cstring>
#include <inttypes.h>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdio.h>

// CANopen ID定义
#define NODE_ID 0x05
#define PDO1_ID (0x180 + NODE_ID)   // 0x185
#define SDO_TX_ID (0x580 + NODE_ID) // 0x585
#define SDO_RX_ID (0x600 + NODE_ID) // 0x605

#define TAG "main"

// 传感器数据结构
typedef struct {
  float roll;     // X轴角度
  float pitch;    // Y轴角度
  float yaw;      // Z轴角度
  float temp;     // 温度
  uint8_t status; // 状态
} sensor_data_t;

// 完整初始化结构体
sensor_data_t sensor_data = {
    .roll = 0, .pitch = 0, .yaw = 0, .temp = 0, .status = 0};

// 返回TWAI句柄的初始化函数
twai_handle_t twai_init() {
  ESP_LOGI(TAG, "Initializing TWAI...");
  ESP_LOGI(TAG, "TX Pin: %d, RX Pin: %d", TX_GPIO_NUM, RX_GPIO_NUM);

  twai_general_config_t g_config = {.mode = TWAI_MODE_NORMAL,
                                    .tx_io = TX_GPIO_NUM,
                                    .rx_io = RX_GPIO_NUM,
                                    .clkout_io = TWAI_IO_UNUSED,
                                    .bus_off_io = TWAI_IO_UNUSED,
                                    .tx_queue_len = 10,
                                    .rx_queue_len = 10,
                                    .alerts_enabled = TWAI_ALERT_ALL,
                                    .clkout_divider = 0,
                                    .intr_flags = ESP_INTR_FLAG_LEVEL1};

  twai_timing_config_t t_config = {
      .brp = 160,              // 波特率预分频器
      .tseg_1 = 16,            // 时间段1
      .tseg_2 = 8,             // 时间段2
      .sjw = 3,                // 同步跳转宽度
      .triple_sampling = false // 三重采样
  };
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // 首先安装驱动
  twai_handle_t handle = NULL;
  ESP_ERROR_CHECK(
      twai_driver_install_v2(&g_config, &t_config, &f_config, &handle));

  if (handle == NULL) {
    ESP_LOGE(TAG, "Failed to get TWAI handle");
    return NULL;
  }

  // 获取alerts以检查总线状态
  uint32_t alerts_triggered;
  twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(1000));

  if (alerts_triggered & TWAI_ALERT_ABOVE_ERR_WARN) {
    ESP_LOGW(TAG, "Error warning limit reached");
  }
  if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
    ESP_LOGW(TAG, "Error passive state reached");
  }
  if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
    ESP_LOGW(TAG, "Bus error occurred");
  }
  if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
    ESP_LOGW(TAG, "RX queue full");
  }

  // 启动 TWAI - 注意这里需要检查handle是否有效
  if (handle != NULL) {
    ESP_ERROR_CHECK(twai_start_v2(handle));
  } else {
    ESP_LOGE(TAG, "Cannot start TWAI: Invalid handle");
    return NULL;
  }

  // 获取并打印状态信息
  twai_status_info_t status;
  twai_get_status_info(&status);
  ESP_LOGI(TAG, "TWAI Status:");
  ESP_LOGI(TAG, "State: %s",
           (status.state == TWAI_STATE_RUNNING) ? "RUNNING" : "NOT RUNNING");
  ESP_LOGI(TAG, "TX Error Counter: %" PRIu32, status.tx_error_counter);
  ESP_LOGI(TAG, "RX Error Counter: %" PRIu32, status.rx_error_counter);
  ESP_LOGI(TAG, "Msgs To TX: %" PRIu32, status.msgs_to_tx);
  ESP_LOGI(TAG, "Msgs To RX: %" PRIu32, status.msgs_to_rx);

  return handle;
}

void send_test_frame(twai_handle_t handle) {
  twai_message_t msg = {
      .flags = TWAI_MSG_FLAG_NONE,
      .identifier = 0x123,
      .data_length_code = 8,
      .data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}};

  auto is_success = twai_transmit_v2(handle, &msg, pdMS_TO_TICKS(100));

  if (is_success == ESP_OK) {
    ESP_LOGI("TWAI", "Test frame sent success");
  } else {
    ESP_LOGE(TAG, "message %d", is_success);
    ESP_LOGE("TWAI", "Send failed");
  }
}

// 解析PDO1数据（低字节在前）
void parse_pdo1(uint8_t *data, sensor_data_t *sensor) {
  // 添加调试日志
  ESP_LOGI(TAG, "Parsing PDO1 data");

  sensor->roll = (int16_t)(data[0] | (data[1] << 8)) * 0.01f;  // B0-B1
  sensor->pitch = (int16_t)(data[2] | (data[3] << 8)) * 0.01f; // B2-B3
  sensor->yaw = (int16_t)(data[4] | (data[5] << 8)) * 0.01f;   // B4-B5
  sensor->temp = (float)data[6] * 0.5f - 40.0f;                // B6
  sensor->status = data[7];                                    // B7

  // 添加解析后的数据日志
  ESP_LOGI(TAG,
           "Parsed values - Roll: %.2f, Pitch: %.2f, Yaw: %.2f, Temp: %.1f°C, "
           "Status: %d",
           sensor->roll, sensor->pitch, sensor->yaw, sensor->temp,
           sensor->status);
}

esp_err_t send_sdo_command(twai_handle_t handle, uint16_t index,
                           uint8_t subindex, uint32_t data, bool save) {
  twai_message_t tx_msg;
  memset(&tx_msg, 0, sizeof(twai_message_t));

  tx_msg.identifier = SDO_RX_ID;
  tx_msg.data_length_code = 8;
  tx_msg.flags = TWAI_MSG_FLAG_NONE;

  tx_msg.data[0] = 0x23;                  // 写4字节命令
  tx_msg.data[1] = (uint8_t)(index);      // 索引低字节
  tx_msg.data[2] = (uint8_t)(index >> 8); // 索引高字节
  tx_msg.data[3] = subindex;
  tx_msg.data[4] = (uint8_t)(data & 0xFF);
  tx_msg.data[5] = (uint8_t)((data >> 8) & 0xFF);
  tx_msg.data[6] = save ? 0xF0 : 0x00;
  tx_msg.data[7] = 0x00;

  esp_err_t err = twai_transmit_v2(handle, &tx_msg, pdMS_TO_TICKS(100));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to send SDO command: %d", err);
    return err;
  }

  // 等待SDO应答
  twai_message_t rx_msg;
  err = twai_receive_v2(handle, &rx_msg, pdMS_TO_TICKS(1000));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to receive SDO response: %d", err);
    return err;
  }

  // 检查应答
  if (rx_msg.identifier != SDO_TX_ID) {
    std::stringstream ss;
    ss << "Unexpected response ID: 0x" << std::hex << std::setfill('0')
       << std::setw(3) << rx_msg.identifier;
    ESP_LOGE(TAG, "%s", ss.str().c_str());
    return ESP_ERR_INVALID_RESPONSE;
  }

  if (rx_msg.data[0] == 0x60) { // 成功应答
    return ESP_OK;
  } else if (rx_msg.data[0] == 0x80) { // 错误应答
    std::stringstream ss;
    ss << "SDO error response: 0x" << std::hex << std::setfill('0')
       << std::setw(2) << rx_msg.data[7] << std::setw(2) << rx_msg.data[6]
       << std::setw(2) << rx_msg.data[5] << std::setw(2) << rx_msg.data[4];
    ESP_LOGE(TAG, "%s", ss.str().c_str());
    return ESP_ERR_INVALID_RESPONSE;
  }

  return ESP_ERR_INVALID_RESPONSE;
}

void config_sensor(twai_handle_t handle) {
  ESP_LOGI(TAG, "Configuring sensor...");

  // 1. 设置预运行模式
  esp_err_t err = send_sdo_command(handle, 0x0000, 0x00, 0x80, false);
  ESP_LOGI(TAG, "Set pre-operational mode result: %d", err);
  vTaskDelay(pdMS_TO_TICKS(100));

  // 2. 设置波特率
  err = send_sdo_command(handle, 0x1021, 0x00, 0x03, true); // 250kbps
  ESP_LOGI(TAG, "Set baudrate result: %d", err);
  vTaskDelay(pdMS_TO_TICKS(100));

  // 3. 设置数据输出周期 (10ms)
  err = send_sdo_command(handle, 0x1023, 0x00, 0x0A, true);
  ESP_LOGI(TAG, "Set cycle time result: %d", err);
  vTaskDelay(pdMS_TO_TICKS(100));

  // 4. 使能PDO1
  err = send_sdo_command(handle, 0x6000, 0x01, 0x01, true);
  ESP_LOGI(TAG, "Enable PDO1 result: %d", err);
  vTaskDelay(pdMS_TO_TICKS(100));

  // 5. 设置运行模式
  err = send_sdo_command(handle, 0x0000, 0x00, 0x01, false);
  ESP_LOGI(TAG, "Set operational mode result: %d", err);
  vTaskDelay(pdMS_TO_TICKS(100));
}

void print_bus_status() {
  twai_status_info_t status;
  twai_get_status_info(&status);

  std::stringstream ss;
  ss << "\nDetailed TWAI Status:\n"
     << "Bus State: "
     << (status.state == TWAI_STATE_RUNNING      ? "RUNNING"
         : status.state == TWAI_STATE_BUS_OFF    ? "BUS_OFF"
         : status.state == TWAI_STATE_RECOVERING ? "RECOVERING"
                                                 : "STOPPED")
     << "\n"
     << "TX Error Counter: " << status.tx_error_counter << "\n"
     << "RX Error Counter: " << status.rx_error_counter << "\n"
     << "TX Failed Count: " << status.tx_failed_count << "\n"
     << "RX Missed Count: " << status.rx_missed_count << "\n"
     << "RX Overrun Count: " << status.rx_overrun_count << "\n"
     << "Bus-off Count: " << status.bus_error_count << "\n"
     << "Arbitration Lost Count: " << status.arb_lost_count;

  ESP_LOGI(TAG, "%s", ss.str().c_str());
}

// 接收任务参数结构体
typedef struct {
  twai_handle_t handle;
  sensor_data_t *sensor_data;
} rx_task_params_t;

void twai_receive_task(void *arg) {
  rx_task_params_t *params = (rx_task_params_t *)arg;
  twai_message_t rx_msg;

  while (1) {
    auto success =
        twai_receive_v2(params->handle, &rx_msg, pdMS_TO_TICKS(1000));

    if (success == ESP_OK) {
      // 改为INFO级别，确保可以看到消息
      ESP_LOGI(TAG, "Received message ID: 0x%03" PRIx32, rx_msg.identifier);
      ESP_LOGI(TAG, "Expected PDO1_ID: 0x%03X", PDO1_ID);

      // 打印完整的数据内容
      ESP_LOGI(TAG, "Data: 0x%02X %02X %02X %02X %02X %02X %02X %02X",
               rx_msg.data[0], rx_msg.data[1], rx_msg.data[2], rx_msg.data[3],
               rx_msg.data[4], rx_msg.data[5], rx_msg.data[6], rx_msg.data[7]);

      if (rx_msg.identifier == PDO1_ID) {
        ESP_LOGI(TAG, "PDO1 message received!");
        parse_pdo1(rx_msg.data, params->sensor_data);
      }
    } else if (success == ESP_ERR_TIMEOUT) {
      // 减少timeout日志频率，每秒只打印一次
      static uint32_t last_warn = 0;
      uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
      if (now - last_warn > 1000) {
        ESP_LOGW(TAG, "Receive timeout - No messages received for 1 second");
        last_warn = now;
      }
    } else {
      ESP_LOGE(TAG, "Receive error code: %d", success);
    }

    // 添加一个小延时，避免打印太快
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

extern "C" void app_main(void) {

  // 初始化TWAI并获取句柄
  twai_handle_t handle = twai_init();
  ESP_LOGI(TAG, "TWAI initialized");

  // 添加引脚监视
  setup_pin_monitor();
  xTaskCreate(twai_monitor_task, "monitor_task", 4096, NULL, 5, NULL);

  // 获取总线状态
  twai_status_info_t status;
  twai_get_status_info(&status);
  std::stringstream ss;

  // 设置16进制输出
  ss << "Bus state: " << status.state << ", TEC: " << status.tx_error_counter
     << ", REC: " << status.rx_error_counter;
  ESP_LOGI(TAG, "%s", ss.str().c_str());

  // 配置传感器
  config_sensor(handle);
  ESP_LOGI(TAG, "Sensor configured");

  // 创建接收任务参数
  rx_task_params_t rx_params = {.handle = handle, .sensor_data = &sensor_data};

  // 创建接收任务
  xTaskCreate(twai_receive_task, "twai_rx", 4096, &rx_params, RX_TASK_PRIO,
              NULL);
  ESP_LOGI(TAG, "Receive task created");

  // 主循环
  while (1) {
    printf("Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°, Temp: %.1f°C\n",
           sensor_data.roll, sensor_data.pitch, sensor_data.yaw,
           sensor_data.temp);
    print_bus_status(); // 添加状态监控
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}