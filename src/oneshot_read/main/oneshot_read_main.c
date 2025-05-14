/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/adc_types.h"
#include "soc/soc_caps.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

const static char *TAG = "EXAMPLE";

/*---------------------------------------------------------------
        ADC General Macros
---------------------------------------------------------------*/
// ADC1 Channels
#if CONFIG_IDF_TARGET_ESP32
#define EXAMPLE_ADC1_CHAN0 ADC_CHANNEL_5
#define EXAMPLE_ADC1_CHAN1 ADC_CHANNEL_6
#else
#define EXAMPLE_ADC1_CHAN0 ADC_CHANNEL_4
#define EXAMPLE_ADC1_CHAN1 ADC_CHANNEL_6
#endif

#if (SOC_ADC_PERIPH_NUM >= 2) && !CONFIG_IDF_TARGET_ESP32C3
/**
 * On ESP32C3, ADC2 is no longer supported, due to its HW limitation.
 * Search for errata on espressif website for more details.
 */
#define EXAMPLE_USE_ADC2 0
#endif

#if EXAMPLE_USE_ADC2
// ADC2 Channels
#if CONFIG_IDF_TARGET_ESP32
#define EXAMPLE_ADC2_CHAN0 ADC_CHANNEL_0
#else
#define EXAMPLE_ADC2_CHAN0 ADC_CHANNEL_0
#endif
#endif // #if EXAMPLE_USE_ADC2

// Using a more appropriate attenuation for 0-1.5V range
#define EXAMPLE_ADC_ATTEN ADC_ATTEN_DB_2_5

// Number of samples to average
#define N_SAMPLES 1

// Define the maximum number of channels in ADC1
#define ADC1_CHANNEL_MAX 10

static int adc_raw[2][10];
static int voltage[2][10];

// Function prototypes
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel,
                                         adc_atten_t atten,
                                         adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);

// Function to read ADC with multiple samples and averaging
static esp_err_t read_adc_with_average(adc_oneshot_unit_handle_t handle,
                                       adc_channel_t channel, int *out_raw) {
  uint32_t sum = 0;
  int raw = 0;
  int samples[N_SAMPLES];

  // 1. 采集所有样本
  for (int i = 0; i < N_SAMPLES; i++) {
    esp_err_t ret = adc_oneshot_read(handle, channel, &raw);
    if (ret != ESP_OK) {
      return ret;
    }
    samples[i] = raw;
    // Small delay between samples for better results
    vTaskDelay(1);
  }

  // // 2. 重排序样本 - 解决I2S采样时左右声道错位问题
  // if (N_SAMPLES >= 4) { // 只有当样本数足够时才进行重排序
  //   for (int i = 0; i < N_SAMPLES - 1; i += 2) {
  //     // 交换相邻样本以修正左右声道错位
  //     int temp = samples[i];
  //     samples[i] = samples[i + 1];
  //     samples[i + 1] = temp;
  //   }
  // }

  // 3. 计算平均值
  for (int i = 0; i < N_SAMPLES; i++) {
    sum += samples[i];
  }

  *out_raw = sum / N_SAMPLES;
  return ESP_OK;
}

void app_main(void) {
  //-------------ADC1 Init---------------//
  adc_oneshot_unit_handle_t adc1_handle;
  adc_oneshot_unit_init_cfg_t init_config1 = {
      .unit_id = ADC_UNIT_1,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

  //-------------ADC1 Config---------------//
  adc_oneshot_chan_cfg_t config = {
      .atten = EXAMPLE_ADC_ATTEN,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };

  // Configure all available channels in ADC1
  adc_cali_handle_t adc1_cali_handles[ADC1_CHANNEL_MAX] = {NULL};
  bool do_calibration1[ADC1_CHANNEL_MAX] = {false};

  // Initialize all ADC1 channels
  for (int channel = 0; channel < ADC1_CHANNEL_MAX; channel++) {
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, channel, &config));
    do_calibration1[channel] = example_adc_calibration_init(
        ADC_UNIT_1, channel, EXAMPLE_ADC_ATTEN, &adc1_cali_handles[channel]);
  }

#if EXAMPLE_USE_ADC2
  //-------------ADC2 Init---------------//
  adc_oneshot_unit_handle_t adc2_handle;
  adc_oneshot_unit_init_cfg_t init_config2 = {
      .unit_id = ADC_UNIT_2,
      .ulp_mode = ADC_ULP_MODE_DISABLE,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config2, &adc2_handle));

  //-------------ADC2 Calibration Init---------------//
  adc_cali_handle_t adc2_cali_handle = NULL;
  bool do_calibration2 = example_adc_calibration_init(
      ADC_UNIT_2, EXAMPLE_ADC2_CHAN0, EXAMPLE_ADC_ATTEN, &adc2_cali_handle);

  //-------------ADC2 Config---------------//
  ESP_ERROR_CHECK(
      adc_oneshot_config_channel(adc2_handle, EXAMPLE_ADC2_CHAN0, &config));
#endif // #if EXAMPLE_USE_ADC2

  while (1) {
    // Read and display all ADC1 channels
    for (int channel = 0; channel < ADC1_CHANNEL_MAX; channel++) {
      ESP_ERROR_CHECK(
          read_adc_with_average(adc1_handle, channel, &adc_raw[0][channel]));
      // ESP_LOGI(TAG, "ADC1 Channel[%d] Raw Data: %d", channel,
      //          adc_raw[0][channel]);

      if (do_calibration1[channel]) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handles[channel],
                                                adc_raw[0][channel],
                                                &voltage[0][channel]));
        //   ESP_LOGI(TAG, "ADC1 Channel[%d] Cali Voltage: %d mV", channel,
        //            voltage[0][channel]);
      }
    }

    // Print a summary of all raw values
    ESP_LOGI(TAG,
             "ADC1 All Channels Raw: [0]:%d, [1]:%d, [2]:%d, [3]:%d, [4]:%d, "
             "[5]:%d, [6]:%d, [7]:%d, [8]:%d, [9]:%d",
             adc_raw[0][0], adc_raw[0][1], adc_raw[0][2], adc_raw[0][3],
             adc_raw[0][4], adc_raw[0][5], adc_raw[0][6], adc_raw[0][7],
             adc_raw[0][8], adc_raw[0][9]);
    ESP_LOGI(TAG,
             "ADC1 cali voltage: [0]:%d, [1]:%d, [2]:%d, [3]:%d, "
             "[4]:%d, [5]:%d, [6]:%d, [7]:%d, [8]:%d, [9]:%d",
             voltage[0][0], voltage[0][1], voltage[0][2], voltage[0][3],
             voltage[0][4], voltage[0][5], voltage[0][6], voltage[0][7],
             voltage[0][8], voltage[0][9]);
    ESP_LOGI(TAG, "--------------------------------");

    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  // Tear Down
  ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));

  // Free calibration handles
  for (int channel = 0; channel < ADC1_CHANNEL_MAX; channel++) {
    if (do_calibration1[channel]) {
      example_adc_calibration_deinit(adc1_cali_handles[channel]);
    }
  }

#if EXAMPLE_USE_ADC2
  ESP_ERROR_CHECK(adc_oneshot_del_unit(adc2_handle));
  if (do_calibration2) {
    example_adc_calibration_deinit(adc2_cali_handle);
  }
#endif // #if EXAMPLE_USE_ADC2
}

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel,
                                         adc_atten_t atten,
                                         adc_cali_handle_t *out_handle) {
  adc_cali_handle_t handle = NULL;
  esp_err_t ret = ESP_FAIL;
  bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  if (!calibrated) {
    ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = unit,
        .chan = channel,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
    if (ret == ESP_OK) {
      calibrated = true;
    }
  }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
  if (!calibrated) {
    ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = unit,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
    if (ret == ESP_OK) {
      calibrated = true;
    }
  }
#endif

  *out_handle = handle;
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "Calibration Success");
  } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
    ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
  } else {
    ESP_LOGE(TAG, "Invalid arg or no memory");
  }

  return calibrated;
}

static void example_adc_calibration_deinit(adc_cali_handle_t handle) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
  ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
  ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
  ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}
