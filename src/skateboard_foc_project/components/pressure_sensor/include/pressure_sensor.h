#pragma once

#include "driver/gpio.h"
#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct pressure_sensor_t *hx711_handle_t;

typedef enum {
  HX711_GAIN_128_A = 1, // Channel A, gain factor 128
  HX711_GAIN_64_B,      // Channel B, gain factor 64
  HX711_GAIN_32_A       // Channel A, gain factor 32
} hx711_gain_t;

typedef struct {
  gpio_num_t dout_gpio; // GPIO pin for data out
  gpio_num_t sck_gpio;  // GPIO pin for clock
  hx711_gain_t gain;    // Amplifier gain setting
  float offset;         // Zero offset (tare)
  float scale;          // Scale factor for weight conversion
} pressure_sensor_config_t;

/**
 * @brief Initialize the pressure sensor with the given configuration
 *
 * @param config Pointer to pressure sensor configuration
 * @param handle Pointer to handle that will be set to the sensor instance
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pressure_sensor_init(const pressure_sensor_config_t *config,
                               hx711_handle_t *handle);

/**
 * @brief Get the current weight reading from the sensor
 *
 * @param handle Sensor handle
 * @return float Weight value or NAN on error
 */
float pressure_sensor_get_weight(hx711_handle_t handle);

/**
 * @brief Perform zero calibration (tare)
 *
 * @param handle Sensor handle
 * @param samples Number of samples to average
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pressure_sensor_tare(hx711_handle_t handle, int samples);

/**
 * @brief Calibrate the sensor with a known weight
 *
 * @param handle Sensor handle
 * @param known_weight The known weight value
 * @param samples Number of samples to average
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pressure_sensor_calibrate(hx711_handle_t handle, float known_weight,
                                    int samples);

/**
 * @brief Save calibration parameters to NVS
 *
 * @param handle Sensor handle
 * @param namespace_name NVS namespace
 * @param key_prefix Key prefix for storing calibration data
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pressure_sensor_save_calibration(hx711_handle_t handle,
                                           const char *namespace_name,
                                           const char *key_prefix);

/**
 * @brief Load calibration parameters from NVS
 *
 * @param handle Sensor handle
 * @param namespace_name NVS namespace
 * @param key_prefix Key prefix for retrieving calibration data
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pressure_sensor_load_calibration(hx711_handle_t handle,
                                           const char *namespace_name,
                                           const char *key_prefix);

/**
 * @brief Free resources associated with the pressure sensor
 *
 * @param handle Sensor handle to free
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pressure_sensor_deinit(hx711_handle_t handle);

#ifdef __cplusplus
}
#endif