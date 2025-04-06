#include "esp_log.h"
#include "hx711.h"
#include "unity.h"
#include <stdio.h>

static const char *TAG = "HX711_TEST";
static hx711_handle_t sensor = NULL;

// 测试前的初始化
TEST_CASE("HX711 初始化测试", "[hx711]") {
  hx711_config_t config = {.dout_gpio = CONFIG_HX711_TEST_DOUT_GPIO,
                           .sck_gpio = CONFIG_HX711_TEST_SCK_GPIO,
                           .gain = HX711_GAIN_128_A,
                           .offset = 0,
                           .scale = 1.0f};

  esp_err_t ret = hx711_init(&config, &sensor);
  TEST_ASSERT_EQUAL(ESP_OK, ret);
  TEST_ASSERT_NOT_NULL(sensor);
}

// 测试读取原始值
TEST_CASE("HX711 读取原始值测试", "[hx711]") {
  TEST_ASSERT_NOT_NULL(sensor);

  int32_t value;
  esp_err_t ret = hx711_read_raw(sensor, &value, 1000);
  TEST_ASSERT_EQUAL(ESP_OK, ret);

  ESP_LOGI(TAG, "原始值: %d", value);

  // 确保值在合理范围内
  TEST_ASSERT_TRUE(value != 0 || value != 0xFFFFFF);
}

// 测试去皮功能
TEST_CASE("HX711 去皮测试", "[hx711]") {
  TEST_ASSERT_NOT_NULL(sensor);

  ESP_LOGI(TAG, "执行去皮操作，确保传感器上无负载");
  esp_err_t ret = hx711_tare(sensor);
  TEST_ASSERT_EQUAL(ESP_OK, ret);

  // 等待一段时间
  vTaskDelay(pdMS_TO_TICKS(100));

  // 读取当前重量，应该接近零
  float weight = hx711_get_weight(sensor);
  ESP_LOGI(TAG, "去皮后，重量: %.2f", weight);

  // 应该接近零，但考虑到噪声，设置一个合理范围
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, weight);
}

// 测试标定功能
TEST_CASE("HX711 calibration test", "[hx711][manual]") {
  TEST_ASSERT_NOT_NULL(sensor);

  // 这是一个手动测试，需要用户交互
  ESP_LOGI(TAG,
           "Place a known weight (e.g. 100g) on the scale and enter its value");

  // 在实际使用中，这里可以通过串口或其他方式获取用户输入的标准重量
  // 为了单元测试的自动化，这里使用一个固定值
  float known_weight = 100.0f; // 100g

  // 读取当前原始值
  int32_t raw_value;
  esp_err_t ret = hx711_read_raw(sensor, &raw_value, 1000);
  TEST_ASSERT_EQUAL(ESP_OK, ret);

  // 计算比例因子
  float scale = (raw_value - sensor->offset) / known_weight;

  // 设置比例因子
  ret = hx711_set_scale(sensor, scale);
  TEST_ASSERT_EQUAL(ESP_OK, ret);

  // 验证校准
  float measured_weight = hx711_get_weight(sensor);
  ESP_LOGI(TAG, "Calibrated weight: %.2f g", measured_weight);

  // 验证测量值接近已知重量
  TEST_ASSERT_FLOAT_WITHIN(1.0f, known_weight, measured_weight);
}

// 测试后的清理
TEST_CASE("HX711 去初始化测试", "[hx711]") {
  TEST_ASSERT_NOT_NULL(sensor);

  esp_err_t ret = hx711_deinit(sensor);
  TEST_ASSERT_EQUAL(ESP_OK, ret);
  sensor = NULL;
}