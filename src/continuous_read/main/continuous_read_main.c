/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "driver/uart.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#define EXAMPLE_ADC_UNIT ADC_UNIT_1
#define _EXAMPLE_ADC_UNIT_STR(unit) #unit
#define EXAMPLE_ADC_UNIT_STR(unit) _EXAMPLE_ADC_UNIT_STR(unit)
#define EXAMPLE_ADC_CONV_MODE ADC_CONV_SINGLE_UNIT_1
#define EXAMPLE_ADC_ATTEN ADC_ATTEN_DB_2_5
#define EXAMPLE_ADC_BIT_WIDTH SOC_ADC_DIGI_MAX_BITWIDTH

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
#define EXAMPLE_ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define EXAMPLE_ADC_GET_CHANNEL(p_data) ((p_data)->type1.channel)
#define EXAMPLE_ADC_GET_DATA(p_data) ((p_data)->type1.data)
#else
#define EXAMPLE_ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE2
#define EXAMPLE_ADC_GET_CHANNEL(p_data) ((p_data)->type2.channel)
#define EXAMPLE_ADC_GET_DATA(p_data) ((p_data)->type2.data)
#endif

#define EXAMPLE_READ_LEN 256

#if CONFIG_IDF_TARGET_ESP32
static adc_channel_t channel[2] = {ADC_CHANNEL_5, ADC_CHANNEL_6};
#else
// Initialize all 10 channels for ESP32-S3
#if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
static adc_channel_t channel[10] = {
    ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3, ADC_CHANNEL_4,
    ADC_CHANNEL_5, ADC_CHANNEL_6, ADC_CHANNEL_7, ADC_CHANNEL_8, ADC_CHANNEL_9};
#else
// For other ESP32 variants
static adc_channel_t channel[2] = {ADC_CHANNEL_5, ADC_CHANNEL_6};
#endif
#endif

static TaskHandle_t s_task_handle;
static const char *TAG = "EXAMPLE";
static esp_adc_cal_characteristics_t adc_chars;

// Arrays to store the latest values for all ADC channels
#if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
#define MAX_ADC1_CHANNELS 10
#elif CONFIG_IDF_TARGET_ESP32
#define MAX_ADC1_CHANNELS 8
#elif CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C2 ||                \
    CONFIG_IDF_TARGET_ESP32H2
#define MAX_ADC1_CHANNELS 5
#else
#define MAX_ADC1_CHANNELS 10 // Default to maximum
#endif

static uint32_t adc_raw_values[MAX_ADC1_CHANNELS] = {0};
static uint32_t adc_voltage_values[MAX_ADC1_CHANNELS] = {0};
static bool adc_channel_updated[MAX_ADC1_CHANNELS] = {0};

// Flag to control data output mode
#define OUTPUT_MODE_SUMMARY 0
#define OUTPUT_MODE_CSV 1
static uint8_t output_mode = OUTPUT_MODE_SUMMARY;

// Counter for CSV output
static uint32_t csv_output_counter = 0;

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle,
                                     const adc_continuous_evt_data_t *edata,
                                     void *user_data) {
  BaseType_t mustYield = pdFALSE;
  // Notify that ADC continuous driver has done enough number of conversions
  vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

  return (mustYield == pdTRUE);
}

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num,
                                adc_continuous_handle_t *out_handle) {
  adc_continuous_handle_t handle = NULL;

  adc_continuous_handle_cfg_t adc_config = {
      .max_store_buf_size = 1024,
      .conv_frame_size = EXAMPLE_READ_LEN,
  };
  ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

  adc_continuous_config_t dig_cfg = {
#if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
      // Increase sampling frequency for multiple channels
      .sample_freq_hz = 40 * 1000,
#else
      .sample_freq_hz = 20 * 1000,
#endif
      .conv_mode = EXAMPLE_ADC_CONV_MODE,
      .format = EXAMPLE_ADC_OUTPUT_TYPE,
  };

  adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
  dig_cfg.pattern_num = channel_num;
  for (int i = 0; i < channel_num; i++) {
    adc_pattern[i].atten = EXAMPLE_ADC_ATTEN;
    adc_pattern[i].channel = channel[i] & 0x7;
    adc_pattern[i].unit = EXAMPLE_ADC_UNIT;
    adc_pattern[i].bit_width = EXAMPLE_ADC_BIT_WIDTH;

    ESP_LOGI(TAG, "adc_pattern[%d].atten is :%" PRIx8, i, adc_pattern[i].atten);
    ESP_LOGI(TAG, "adc_pattern[%d].channel is :%" PRIx8, i,
             adc_pattern[i].channel);
    ESP_LOGI(TAG, "adc_pattern[%d].unit is :%" PRIx8, i, adc_pattern[i].unit);
  }
  dig_cfg.adc_pattern = adc_pattern;
  ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

  *out_handle = handle;
}

static void adc_calibration_init(void) {
  esp_adc_cal_value_t val_type =
      esp_adc_cal_characterize(EXAMPLE_ADC_UNIT,      // ADC unit
                               EXAMPLE_ADC_ATTEN,     // Attenuation
                               EXAMPLE_ADC_BIT_WIDTH, // Resolution
                               0, // Default reference voltage
                               &adc_chars);

  const char *cal_type_str;
  switch (val_type) {
  case ESP_ADC_CAL_VAL_EFUSE_VREF:
    cal_type_str = "eFuse Vref";
    break;
  case ESP_ADC_CAL_VAL_EFUSE_TP:
    cal_type_str = "Two Point";
    break;
  case ESP_ADC_CAL_VAL_DEFAULT_VREF:
    cal_type_str = "Default Vref";
    break;
  default:
    cal_type_str = "Unknown";
  }
  ESP_LOGI(TAG, "ADC Calibration: %s", cal_type_str);
}

static void display_adc_channels(void) {
  ESP_LOGI(TAG, "Available ADC channels in Unit %s:",
           EXAMPLE_ADC_UNIT_STR(EXAMPLE_ADC_UNIT));

#if CONFIG_IDF_TARGET_ESP32
  ESP_LOGI(TAG, "ESP32 ADC1 channels: 0, 1, 2, 3, 4, 5, 6, 7");
  // ADC1 channel mapping to GPIO:
  ESP_LOGI(TAG, "ADC1: CH0(GPIO36), CH1(GPIO37), CH2(GPIO38), CH3(GPIO39), "
                "CH4(GPIO32), CH5(GPIO33), CH6(GPIO34), CH7(GPIO35)");
#elif CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
  ESP_LOGI(TAG, "ESP32-S2/S3 ADC1 channels: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9");
  // ADC1 channel mapping to GPIO for S2/S3:
  ESP_LOGI(TAG,
           "ADC1: CH0(GPIO1), CH1(GPIO2), CH2(GPIO3), CH3(GPIO4), CH4(GPIO5), "
           "CH5(GPIO6), CH6(GPIO7), CH7(GPIO8), CH8(GPIO9), CH9(GPIO10)");
#elif CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C2
  ESP_LOGI(TAG, "ESP32-C3/C2 ADC1 channels: 0, 1, 2, 3, 4");
  // ADC1 channel mapping to GPIO for C3/C2:
  ESP_LOGI(TAG,
           "ADC1: CH0(GPIO0), CH1(GPIO1), CH2(GPIO2), CH3(GPIO3), CH4(GPIO4)");
#elif CONFIG_IDF_TARGET_ESP32H2
  ESP_LOGI(TAG, "ESP32-H2 ADC1 channels: 0, 1, 2, 3, 4");
  // ADC1 channel mapping to GPIO for H2:
  ESP_LOGI(TAG,
           "ADC1: CH0(GPIO1), CH1(GPIO2), CH2(GPIO3), CH3(GPIO4), CH4(GPIO5)");
#else
  ESP_LOGI(TAG, "Unknown ESP target, cannot display channel information");
#endif

  ESP_LOGI(TAG, "Currently configured channels: ");
  for (int i = 0; i < sizeof(channel) / sizeof(adc_channel_t); i++) {
    ESP_LOGI(TAG, "  Channel %d", channel[i]);
  }
  ESP_LOGI(TAG, "Attenuation: %d", EXAMPLE_ADC_ATTEN);
}

static void display_adc_summary(void) {
#if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
  // ESP_LOGI(TAG,
  //          "ADC1 All Channels Raw: [0]:%d, [1]:%d, [2]:%d, [3]:%d, [4]:%d, "
  //          "[5]:%d, [6]:%d, [7]:%d, [8]:%d, [9]:%d",
  //          adc_raw_values[0], adc_raw_values[1], adc_raw_values[2],
  //          adc_raw_values[3], adc_raw_values[4], adc_raw_values[5],
  //          adc_raw_values[6], adc_raw_values[7], adc_raw_values[8],
  //          adc_raw_values[9]);
  ESP_LOGI(TAG,
           "ADC1 Calibrated Voltage (mV): [0]:%d, [1]:%d, [2]:%d, [3]:%d, "
           "[4]:%d, [5]:%d, [6]:%d, [7]:%d, [8]:%d, [9]:%d",
           adc_voltage_values[0], adc_voltage_values[1], adc_voltage_values[2],
           adc_voltage_values[3], adc_voltage_values[4], adc_voltage_values[5],
           adc_voltage_values[6], adc_voltage_values[7], adc_voltage_values[8],
           adc_voltage_values[9]);
#elif CONFIG_IDF_TARGET_ESP32
  ESP_LOGI(TAG,
           "ADC1 All Channels Raw: [0]:%d, [1]:%d, [2]:%d, [3]:%d, [4]:%d, "
           "[5]:%d, [6]:%d, [7]:%d",
           adc_raw_values[0], adc_raw_values[1], adc_raw_values[2],
           adc_raw_values[3], adc_raw_values[4], adc_raw_values[5],
           adc_raw_values[6], adc_raw_values[7]);
  ESP_LOGI(TAG,
           "ADC1 Calibrated Voltage (mV): [0]:%d, [1]:%d, [2]:%d, [3]:%d, "
           "[4]:%d, [5]:%d, [6]:%d, [7]:%d",
           adc_voltage_values[0], adc_voltage_values[1], adc_voltage_values[2],
           adc_voltage_values[3], adc_voltage_values[4], adc_voltage_values[5],
           adc_voltage_values[6], adc_voltage_values[7]);
#else
  ESP_LOGI(TAG, "ADC1 All Channels Raw: [0]:%d, [1]:%d, [2]:%d, [3]:%d, [4]:%d",
           adc_raw_values[0], adc_raw_values[1], adc_raw_values[2],
           adc_raw_values[3], adc_raw_values[4]);
  ESP_LOGI(
      TAG,
      "ADC1 Calibrated Voltage (mV): [0]:%d, [1]:%d, [2]:%d, [3]:%d, [4]:%d",
      adc_voltage_values[0], adc_voltage_values[1], adc_voltage_values[2],
      adc_voltage_values[3], adc_voltage_values[4]);
#endif
  // ESP_LOGI(TAG, "--------------------------------");
}

static void output_channels_csv(void) {
  // Output in CSV format: timestamp,ch3,ch4,ch5,ch6
  uint32_t timestamp = esp_log_timestamp();

  // Print header every 100 samples
  if (csv_output_counter % 100 == 0) {
    printf("timestamp,ch3_raw,ch4_raw,ch5_raw,ch6_raw,ch3_mv,ch4_mv,ch5_mv,ch6_"
           "mv\n");
  }

  // Print data in CSV format using PRIu32 for uint32_t values
  printf("%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32
         ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 "\n",
         timestamp, adc_raw_values[3], adc_raw_values[4], adc_raw_values[5],
         adc_raw_values[6], adc_voltage_values[3], adc_voltage_values[4],
         adc_voltage_values[5], adc_voltage_values[6]);

  csv_output_counter++;
}

void app_main(void) {
  esp_err_t ret;
  uint32_t ret_num = 0;
  uint8_t result[EXAMPLE_READ_LEN] = {0};
  memset(result, 0xcc, EXAMPLE_READ_LEN);

  s_task_handle = xTaskGetCurrentTaskHandle();

  // Display available ADC channels
  display_adc_channels();

  // Initialize ADC calibration
  adc_calibration_init();

  adc_continuous_handle_t handle = NULL;
  continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t),
                      &handle);

  adc_continuous_evt_cbs_t cbs = {
      .on_conv_done = s_conv_done_cb,
  };
  ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
  ESP_ERROR_CHECK(adc_continuous_start(handle));

  // Print instructions for changing output mode
  ESP_LOGI(TAG, "Press 's' for summary mode, 'c' for CSV mode");

  while (1) {
    // Check if there's data in UART
    char c;
    if (uart_read_bytes(CONFIG_ESP_CONSOLE_UART_NUM, (uint8_t *)&c, 1, 0) ==
        1) {
      if (c == 's') {
        output_mode = OUTPUT_MODE_SUMMARY;
        ESP_LOGI(TAG, "Switched to summary mode");
      } else if (c == 'c') {
        output_mode = OUTPUT_MODE_CSV;
        csv_output_counter = 0;
        ESP_LOGI(TAG, "Switched to CSV mode");
      }
    }

    /**
     * This is to show you the way to use the ADC continuous mode driver event
     * callback. This `ulTaskNotifyTake` will block when the data processing in
     * the task is fast. However in this example, the data processing (print) is
     * slow, so you barely block here.
     *
     * Without using this event callback (to notify this task), you can still
     * just call `adc_continuous_read()` here in a loop, with/without a certain
     * block timeout.
     */
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    char unit[] = EXAMPLE_ADC_UNIT_STR(EXAMPLE_ADC_UNIT);

    while (1) {
      ret = adc_continuous_read(handle, result, EXAMPLE_READ_LEN, &ret_num, 0);
      if (ret == ESP_OK) {
        // ESP_LOGI("TASK", "ret is %x, ret_num is %" PRIu32 " bytes", ret,
        //          ret_num);
        for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
          adc_digi_output_data_t *p = (adc_digi_output_data_t *)&result[i];
          uint32_t chan_num = EXAMPLE_ADC_GET_CHANNEL(p);
          uint32_t data = EXAMPLE_ADC_GET_DATA(p);
          /* Check the channel number validation, the data is invalid if the
           * channel num exceed the maximum channel */
          if (chan_num < SOC_ADC_CHANNEL_NUM(EXAMPLE_ADC_UNIT)) {
            // Convert raw to voltage in mV
            uint32_t voltage = esp_adc_cal_raw_to_voltage(data, &adc_chars);

            // Store values for summary display
            if (chan_num < MAX_ADC1_CHANNELS) {
              adc_raw_values[chan_num] = data;
              adc_voltage_values[chan_num] = voltage;
              adc_channel_updated[chan_num] = true;
            }

            // Print decimal format
            // ESP_LOGI(TAG, "Unit: %s, Channel: %u, Raw: %u, Voltage: %u mV",
            //          unit, chan_num, data, voltage);

            // Uncomment to see hexadecimal format
            // ESP_LOGI(TAG, "Unit: %s, Channel: %u, Raw: 0x%X, Voltage: %u mV",
            //          unit, chan_num, data, voltage);

            // Uncomment to see byte-by-byte hex dump
            // for (int j = 0; j < SOC_ADC_DIGI_RESULT_BYTES; ++j) {
            //     ESP_LOGI(TAG, "byte[%d]=%02X", j, ((uint8_t*)&result[i])[j]);
            // }
          } else {
            ESP_LOGW(TAG, "Invalid data [%s_%u_0x%X]", unit, chan_num, data);
          }
        }

        // Display data according to selected mode
        if (output_mode == OUTPUT_MODE_SUMMARY) {
          display_adc_summary();
        } else if (output_mode == OUTPUT_MODE_CSV) {
          output_channels_csv();
        }

        /**
         * Because printing is slow, so every time you call `ulTaskNotifyTake`,
         * it will immediately return. To avoid a task watchdog timeout, add a
         * delay here. When you replace the way you process the data, usually
         * you don't need this delay (as this task will block for a while).
         */
        vTaskDelay(1);
      } else if (ret == ESP_ERR_TIMEOUT) {
        // We try to read `EXAMPLE_READ_LEN` until API returns timeout, which
        // means there's no available data
        break;
      }
    }
  }

  ESP_ERROR_CHECK(adc_continuous_stop(handle));
  ESP_ERROR_CHECK(adc_continuous_deinit(handle));
}
