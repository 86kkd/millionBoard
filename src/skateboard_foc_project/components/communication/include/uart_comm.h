#pragma once

#include "driver/uart.h"
#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize UART communication
 *
 * @param uart_num UART port number
 * @param config UART configuration
 * @param tx_pin TX pin number
 * @param rx_pin RX pin number
 * @return esp_err_t ESP_OK on success
 */
esp_err_t uart_comm_init(uart_port_t uart_num, const uart_config_t *config,
                         int tx_pin, int rx_pin);

/**
 * @brief Write data to UART
 *
 * @param uart_num UART port number
 * @param data Data buffer
 * @param len Data length
 * @return esp_err_t ESP_OK on success
 */
esp_err_t uart_comm_write(uart_port_t uart_num, const uint8_t *data,
                          size_t len);

/**
 * @brief Read data from UART
 *
 * @param uart_num UART port number
 * @param data Data buffer
 * @param max_len Maximum data length
 * @param bytes_read Pointer to store bytes read
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT on timeout
 */
esp_err_t uart_comm_read(uart_port_t uart_num, uint8_t *data, size_t max_len,
                         size_t *bytes_read, uint32_t timeout_ms);

/**
 * @brief Read line from UART until newline or timeout
 *
 * @param uart_num UART port number
 * @param data Data buffer
 * @param max_len Maximum data length
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT on timeout
 */
esp_err_t uart_comm_read_line(uart_port_t uart_num, char *data, size_t max_len,
                              uint32_t timeout_ms);

/**
 * @brief Deinitialize UART communication
 *
 * @param uart_num UART port number
 * @return esp_err_t ESP_OK on success
 */
esp_err_t uart_comm_deinit(uart_port_t uart_num);

#ifdef __cplusplus
}
#endif
