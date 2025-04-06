#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "unity.h"
#include <stdio.h>

// 包含测试的组件头文件
#include "angle_sensor.h"
#include "battery_mgr.h"
#include "can_comm.h"
#include "hx711.h"
#include "i2c_comm.h"
#include "motor_control_foc.h"
#include "nfc.h"
#include "uart_comm.h"

static const char *TAG = "SYSTEM_TEST";

// 测试系统初始化
TEST_CASE("系统初始化测试", "[system]") {
  ESP_LOGI(TAG, "测试系统各模块初始化");

  // 初始化通信
  esp_err_t ret = i2c_comm_init();
  TEST_ASSERT_EQUAL(ESP_OK, ret);

  ret = can_comm_init();
  TEST_ASSERT_EQUAL(ESP_OK, ret);

  // 初始化传感器
  ret = angle_sensor_init();
  TEST_ASSERT_EQUAL(ESP_OK, ret);

  ret = nfc_init();
  TEST_ASSERT_EQUAL(ESP_OK, ret);

  // 初始化电池管理
  ret = battery_mgr_init();
  TEST_ASSERT_EQUAL(ESP_OK, ret);

  // 初始化电机控制
  ret = motor_control_init();
  TEST_ASSERT_EQUAL(ESP_OK, ret);

  ESP_LOGI(TAG, "所有模块初始化成功");
}

// 测试重心计算
TEST_CASE("重心计算测试", "[system][manual]") {
  ESP_LOGI(TAG, "测试重心计算与电机控制");

  // 初始化 HX711 传感器
  hx711_config_t front_config = {.dout_gpio = CONFIG_HX711_FRONT_DOUT_GPIO,
                                 .sck_gpio = CONFIG_HX711_FRONT_SCK_GPIO,
                                 .gain = HX711_GAIN_128_A,
                                 .offset = 0,
                                 .scale = 1.0f};

  hx711_config_t rear_config = {.dout_gpio = CONFIG_HX711_REAR_DOUT_GPIO,
                                .sck_gpio = CONFIG_HX711_REAR_SCK_GPIO,
                                .gain = HX711_GAIN_128_A,
                                .offset = 0,
                                .scale = 1.0f};

  hx711_handle_t front_sensor = NULL;
  hx711_handle_t rear_sensor = NULL;

  esp_err_t ret = hx711_init(&front_config, &front_sensor);
  TEST_ASSERT_EQUAL(ESP_OK, ret);

  ret = hx711_init(&rear_config, &rear_sensor);
  TEST_ASSERT_EQUAL(ESP_OK, ret);

  // 执行去皮操作
  ret = hx711_tare(front_sensor);
  TEST_ASSERT_EQUAL(ESP_OK, ret);

  ret = hx711_tare(rear_sensor);
  TEST_ASSERT_EQUAL(ESP_OK, ret);

  ESP_LOGI(TAG, "请在滑板上移动重心，测试电机控制...");

  // 读取重心变化并计算电机输出
  for (int i = 0; i < 10; i++) {
    float front_pressure = hx711_get_weight(front_sensor);
    float rear_pressure = hx711_get_weight(rear_sensor);

    float weight_diff = front_pressure - rear_pressure;
    float total_weight = front_pressure + rear_pressure;

    float balance_point = 0.0f;
    float target_speed = 0.0f;

    if (total_weight > 10.0f) {
      balance_point = weight_diff / total_weight;
      target_speed = balance_point * CONFIG_MAX_SPEED / 3600.0f; // 转换为 m/s
    }

    ESP_LOGI(TAG, "重心: %.2f, 目标速度: %.2f m/s", balance_point,
             target_speed);

    // 设置电机速度
    ret = motor_control_set_speed(target_speed);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    vTaskDelay(pdMS_TO_TICKS(500));
  }

  // 停止电机
  ret = motor_control_set_speed(0);
  TEST_ASSERT_EQUAL(ESP_OK, ret);

  // 释放资源
  ret = hx711_deinit(front_sensor);
  TEST_ASSERT_EQUAL(ESP_OK, ret);

  ret = hx711_deinit(rear_sensor);
  TEST_ASSERT_EQUAL(ESP_OK, ret);
}

// 测试倾角补偿
TEST_CASE("倾角补偿测试", "[system]") {
  ESP_LOGI(TAG, "测试倾角补偿");

  // 测试不同角度的补偿值
  float angles[] = {-15.0f, -10.0f, -5.0f, 0.0f, 5.0f, 10.0f, 15.0f};

  for (int i = 0; i < sizeof(angles) / sizeof(angles[0]); i++) {
    float angle = angles[i];
    float compensation = calculate_incline_compensation(angle);

    ESP_LOGI(TAG, "角度: %.1f°, 补偿值: %.3f", angle, compensation);

    // 验证补偿值
    if (angle < 0) {
      TEST_ASSERT_TRUE(compensation < 0); // 下坡应该是负补偿
    } else if (angle > 0) {
      TEST_ASSERT_TRUE(compensation > 0); // 上坡应该是正补偿
    } else {
      TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, compensation); // 平地应该接近零
    }
  }
}

// 测试电池状态读取
TEST_CASE("电池状态读取测试", "[system]") {
  ESP_LOGI(TAG, "测试电池状态读取");

  battery_status_t status;
  esp_err_t ret = battery_mgr_get_status(&status);
  TEST_ASSERT_EQUAL(ESP_OK, ret);

  ESP_LOGI(TAG, "电池电压: %.2fV", status.voltage);
  ESP_LOGI(TAG, "电池电流: %.2fA", status.current);
  ESP_LOGI(TAG, "电池电量: %d%%", status.percentage);
  ESP_LOGI(TAG, "电池温度: %.1f°C", status.temperature);
  ESP_LOGI(TAG, "充电状态: %s", status.is_charging ? "充电中" : "未充电");

  // 验证合理范围
  TEST_ASSERT_TRUE(status.voltage > 20.0f &&
                   status.voltage < 42.0f); // 10S 锂电池电压范围
  TEST_ASSERT_TRUE(status.temperature > -10.0f &&
                   status.temperature < 60.0f); // 温度合理范围
}
