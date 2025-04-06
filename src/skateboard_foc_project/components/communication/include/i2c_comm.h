#pragma once

#include "driver/i2c.h"
#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize I2C communication
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t i2c_comm_init(void);

/**
 * @brief Write to I2C device
 *
 * @param addr Device address
 * @param data Data buffer
 * @param len Data length
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t ESP_OK on success
 */
esp_err_t i2c_comm_write(uint8_t addr, const uint8_t *data, size_t len,
                         uint32_t timeout_ms);

/**
 * @brief Read from I2C device
 *
 * @param addr Device address
 * @param data Data buffer
 * @param len Data length
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t ESP_OK on success
 */
esp_err_t i2c_comm_read(uint8_t addr, uint8_t *data, size_t len,
                        uint32_t timeout_ms);

/**
 * @brief Write to and read from I2C device
 *
 * @param addr Device address
 * @param write_data Write data buffer
 * @param write_len Write data length
 * @param read_data Read data buffer
 * @param read_len Pointer to read data length
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t ESP_OK on success
 */
esp_err_t i2c_comm_write_read(uint8_t addr, const uint8_t *write_data,
                              size_t write_len, uint8_t *read_data,
                              uint8_t *read_len, uint32_t timeout_ms);

/**
 * @brief Scan I2C bus for devices
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t i2c_comm_scan(void);

/**
 * @brief Deinitialize I2C communication
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t i2c_comm_deinit(void);

#ifdef __cplusplus
}
#endif
