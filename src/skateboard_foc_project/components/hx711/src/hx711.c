#include "hx711.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "HX711";

struct hx711_dev_t {
  gpio_num_t dout_gpio;
  gpio_num_t sck_gpio;
  hx711_gain_t gain;
  int32_t offset;
  float scale;
  int32_t last_raw_reading;
};

esp_err_t hx711_init(const hx711_config_t *config, hx711_handle_t *handle) {
#if CONFIG_ENABLE_HX711
  if (config == NULL || handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  // Allocate device structure
  hx711_handle_t dev = calloc(1, sizeof(struct hx711_dev_t));
  if (dev == NULL) {
    return ESP_ERR_NO_MEM;
  }

  // Copy config
  dev->dout_gpio = config->dout_gpio;
  dev->sck_gpio = config->sck_gpio;
  dev->gain = config->gain;
  dev->offset = config->offset;
  dev->scale = config->scale;

  // Configure GPIO pins
  gpio_config_t io_conf = {.intr_type = GPIO_INTR_DISABLE,
                           .mode = GPIO_MODE_OUTPUT,
                           .pin_bit_mask = (1ULL << dev->sck_gpio),
                           .pull_down_en = 0,
                           .pull_up_en = 0};
  gpio_config(&io_conf);

  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pin_bit_mask = (1ULL << dev->dout_gpio);
  io_conf.pull_up_en = 0;
  gpio_config(&io_conf);

  // Set initial pin states
  gpio_set_level(dev->sck_gpio, 0);

  // Power up and wait for stabilization
  hx711_power_up(dev);
  vTaskDelay(pdMS_TO_TICKS(100));

  *handle = dev;
  ESP_LOGI(TAG, "HX711 initialized on pins DOUT:%d, SCK:%d", dev->dout_gpio,
           dev->sck_gpio);

  return ESP_OK;
#else
  ESP_LOGW(TAG, "HX711 support is disabled");
  return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t hx711_deinit(hx711_handle_t handle) {
#if CONFIG_ENABLE_HX711
  if (handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  // Power down the device
  hx711_power_down(handle);

  // Free memory
  free(handle);

  return ESP_OK;
#else
  return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t hx711_read_raw(hx711_handle_t handle, int32_t *value,
                         uint32_t timeout_ms) {
#if CONFIG_ENABLE_HX711
  if (handle == NULL || value == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  // Wait for data to be ready (DOUT goes low)
  uint64_t start_time = esp_timer_get_time() / 1000;
  while (gpio_get_level(handle->dout_gpio) == 1) {
    if ((esp_timer_get_time() / 1000) - start_time > timeout_ms) {
      return ESP_ERR_TIMEOUT;
    }
    vTaskDelay(1);
  }

  // Read 24 bits of data
  int32_t data = 0;
  for (int i = 0; i < 24; i++) {
    gpio_set_level(handle->sck_gpio, 1);
    esp_rom_delay_us(1); // Short delay
    data = (data << 1) | gpio_get_level(handle->dout_gpio);
    gpio_set_level(handle->sck_gpio, 0);
    esp_rom_delay_us(1); // Short delay
  }

  // Set gain for next reading
  for (int i = 0; i < handle->gain; i++) {
    gpio_set_level(handle->sck_gpio, 1);
    esp_rom_delay_us(1);
    gpio_set_level(handle->sck_gpio, 0);
    esp_rom_delay_us(1);
  }

  // Convert to signed value (2's complement)
  if (data & 0x800000) {
    data |= 0xFF000000;
  }

  *value = data;
  handle->last_raw_reading = data;

  return ESP_OK;
#else
  return ESP_ERR_NOT_SUPPORTED;
#endif
}

float hx711_get_weight(hx711_handle_t handle) {
#if CONFIG_ENABLE_HX711
  if (handle == NULL) {
    return 0.0f;
  }

  int32_t raw_value;
  if (hx711_read_raw(handle, &raw_value, 1000) != ESP_OK) {
    ESP_LOGW(TAG, "Failed to read HX711");
    return 0.0f;
  }

  // Calculate weight based on raw value, offset, and scale
  return (raw_value - handle->offset) / handle->scale;
#else
  return 0.0f;
#endif
}

esp_err_t hx711_tare(hx711_handle_t handle) {
#if CONFIG_ENABLE_HX711
  if (handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  // Take multiple readings and average them for better tare accuracy
  int32_t sum = 0;
  int32_t sample;
  const int num_samples = 10;

  for (int i = 0; i < num_samples; i++) {
    if (hx711_read_raw(handle, &sample, 1000) != ESP_OK) {
      return ESP_FAIL;
    }
    sum += sample;
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  handle->offset = sum / num_samples;
  ESP_LOGI(TAG, "Tare offset set to %d", handle->offset);

  return ESP_OK;
#else
  return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t hx711_set_scale(hx711_handle_t handle, float scale) {
#if CONFIG_ENABLE_HX711
  if (handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  handle->scale = scale;
  ESP_LOGI(TAG, "Scale set to %.4f", scale);

  return ESP_OK;
#else
  return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t hx711_power_down(hx711_handle_t handle) {
#if CONFIG_ENABLE_HX711
  if (handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  gpio_set_level(handle->sck_gpio, 1);
  vTaskDelay(pdMS_TO_TICKS(60)); // Hold SCK high for >60ms to enter power down

  return ESP_OK;
#else
  return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t hx711_power_up(hx711_handle_t handle) {
#if CONFIG_ENABLE_HX711
  if (handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  gpio_set_level(handle->sck_gpio, 0);
  vTaskDelay(pdMS_TO_TICKS(1)); // Need at least 1ms to power up

  return ESP_OK;
#else
  return ESP_ERR_NOT_SUPPORTED;
#endif
}