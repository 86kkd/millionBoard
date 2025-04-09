#pragma once

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct hx711_dev_t *hx711_handle_t;

typedef enum {
  HX711_GAIN_128_A = 1, // Channel A, gain 128
  HX711_GAIN_32_B,      // Channel B, gain 32
  HX711_GAIN_64_A       // Channel A, gain 64
} hx711_gain_t;

typedef struct {
  gpio_num_t dout_gpio; // HX711 DOUT pin
  gpio_num_t sck_gpio;  // HX711 SCK pin
  hx711_gain_t gain;    // Input channel and gain
  int32_t offset;       // Tare offset
  float scale;          // Scale factor for converting to weight
} hx711_config_t;

/**
 * @brief Initialize HX711 device
 *
 * @param config Pointer to HX711 configuration
 * @param handle Pointer to handle that will receive the HX711 instance
 * @return esp_err_t ESP_OK on success
 */
esp_err_t hx711_init(const hx711_config_t *config, hx711_handle_t *handle);

/**
 * @brief Deinitialize HX711 device
 *
 * @param handle HX711 handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t hx711_deinit(hx711_handle_t handle);

/**
 * @brief Read raw value from HX711
 *
 * @param handle HX711 handle
 * @param value Pointer to store raw value
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT on timeout
 */
esp_err_t hx711_read_raw(hx711_handle_t handle, int32_t *value,
                         uint32_t timeout_ms);

/**
 * @brief Get weight reading in configured units
 *
 * @param handle HX711 handle
 * @return float Weight value
 */
float hx711_get_weight(hx711_handle_t handle);

/**
 * @brief Set tare offset (zero scale)
 *
 * @param handle HX711 handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t hx711_tare(hx711_handle_t handle);

/**
 * @brief Set scale factor for converting to weight units
 *
 * @param handle HX711 handle
 * @param scale Scale factor
 * @return esp_err_t ESP_OK on success
 */
esp_err_t hx711_set_scale(hx711_handle_t handle, float scale);

/**
 * @brief Power down the HX711
 *
 * @param handle HX711 handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t hx711_power_down(hx711_handle_t handle);

/**
 * @brief Power up the HX711
 *
 * @param handle HX711 handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t hx711_power_up(hx711_handle_t handle);

#ifdef __cplusplus
}
#endif