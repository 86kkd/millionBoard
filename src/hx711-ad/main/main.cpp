#include <inttypes.h>
#include <math.h>
#include <unistd.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "hx711.hpp"
#include "nvs_flash.h"
#include "scale.hpp"

#define SCALE_1_PIN GPIO_NUM_15
#define SCALE_2_PIN GPIO_NUM_47
#define SCALE_1_DATA_PIN GPIO_NUM_16
#define SCALE_2_DATA_PIN GPIO_NUM_21

// 添加宏定义来指定要校准的传感器: 1 表示 scale_1, 2 表示 scale_2
#define SENSOR_TO_CALIBRATE 1 // 修改此值以选择所需校准的传感器

extern "C" {
void app_main(void);
}

static const char *kTag = "app";

// NVS 名称空间和键前缀
static const char *kNvsNamespace = "scale_cal";
static const char *kScalePrefix1 = "scale1";
static const char *kScalePrefix2 = "scale2";

static const gpio_num_t kClockPin_1 = SCALE_1_PIN;
static const gpio_num_t kDataPin_1 = SCALE_1_DATA_PIN;
static const gpio_num_t kClockPin_2 = SCALE_2_PIN;
static const gpio_num_t kDataPin_2 = SCALE_2_DATA_PIN;

// 校准状态
enum CalibrationState {
  kNone,        // 未校准
  kTare,        // 正在进行零点校准
  kCalibration, // 正在进行比例系数校准
  kMeasurement  // 校准完成，正在进行测量
};

// 已知重量（克）- 用于校准
static const float kKnownWeight = 1000.0f; // 1kg 标准砝码

void app_main(void) {
  ESP_LOGI(kTag, "Starting App");

  // 初始化NVS
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
      err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    // 擦除并重新初始化
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);

  HX711 hx711_1(kClockPin_1, kDataPin_1, HX711::Mode::kChannelA128);
  HX711 hx711_2(kClockPin_2, kDataPin_2, HX711::Mode::kChannelA128);

  Scale scale_1(hx711_1);
  Scale scale_2(hx711_2);

  // 尝试从NVS加载校准参数
  bool loaded_1 = scale_1.LoadCalibration(kNvsNamespace, kScalePrefix1);
  bool loaded_2 = scale_2.LoadCalibration(kNvsNamespace, kScalePrefix2);
  // 根据宏选择是否已加载要校准的传感器参数
  bool loaded_selected = (SENSOR_TO_CALIBRATE == 1 ? loaded_1 : loaded_2);

  // // 根据是否成功加载参数决定初始状态
  CalibrationState state;
  if (loaded_selected) {
    ESP_LOGI(kTag, "已从NVS加载第 %d 号传感器校准参数，直接进入测量模式",
             SENSOR_TO_CALIBRATE);
    state = kMeasurement;
  } else {
    ESP_LOGI(kTag, "需要对第 %d 号传感器进行校准", SENSOR_TO_CALIBRATE);
    state = kNone;
  }

  // state = kNone;

  float weight_1, weight_2; // 将变量声明移到switch外部

  while (true) {
    switch (state) {
    case kNone:
      // 首先进行零点校准
      ESP_LOGI(kTag, "请确保秤上没有任何物体，5秒后进行零点校准...");
      vTaskDelay(pdMS_TO_TICKS(5000));
      state = kTare;
      break;

    case kTare:
// 仅对指定传感器进行零点校准
#if SENSOR_TO_CALIBRATE == 1
      if (scale_1.Tare(10)) {
#else
      if (scale_2.Tare(10)) {
#endif
        ESP_LOGI(kTag,
                 "传感器 %d 零点校准完成，请在秤上放置 %.1f "
                 "克标准砝码，5秒后进行校准...",
                 SENSOR_TO_CALIBRATE, kKnownWeight);
        vTaskDelay(pdMS_TO_TICKS(5000));
        state = kCalibration;
      } else {
        ESP_LOGE(kTag, "传感器 %d 零点校准失败，5秒后重试...",
                 SENSOR_TO_CALIBRATE);
        vTaskDelay(pdMS_TO_TICKS(5000));
      }
      break;

    case kCalibration:
// 仅对指定传感器进行比例系数校准
#if SENSOR_TO_CALIBRATE == 1
      if (scale_1.Calibrate(kKnownWeight, 10)) {
        scale_1.SaveCalibration(kNvsNamespace, kScalePrefix1);
#else
      if (scale_2.Calibrate(kKnownWeight, 10)) {
        scale_2.SaveCalibration(kNvsNamespace, kScalePrefix2);
#endif
        ESP_LOGI(kTag, "传感器 %d 校准完成，进入测量模式...",
                 SENSOR_TO_CALIBRATE);
        state = kMeasurement;
      } else {
        ESP_LOGE(kTag, "传感器 %d 校准失败，5秒后重试...", SENSOR_TO_CALIBRATE);
        vTaskDelay(pdMS_TO_TICKS(5000));
        state = kTare; // 返回零点校准
      }
      break;

    case kMeasurement:
      // 读取重量并显示
      weight_1 = scale_1.GetWeight(5);
      weight_2 = scale_2.GetWeight(5);

      // 检查测量值是否有效
      if (!isnan(weight_1) && !isnan(weight_2)) {
        ESP_LOGI(kTag, "重量: 1=%.2f克, 2=%.2f克, 总重=%.2f克", weight_1,
                 weight_2, weight_1 + weight_2);
      } else {
        ESP_LOGE(kTag, "测量失败，获得了无效的重量值");
      }

      // 校准参数信息
      ESP_LOGI(kTag,
               "校准参数: 1(零点=%.2f, 系数=%.2f), 2(零点=%.2f, 系数=%.2f)",
               scale_1.GetOffset(), scale_1.GetScale(), scale_2.GetOffset(),
               scale_2.GetScale());

      break;

    default:
      // 处理未知状态
      ESP_LOGE(kTag, "未知的校准状态，重置为初始状态");
      state = kNone;
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
