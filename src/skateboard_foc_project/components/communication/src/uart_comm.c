#include "uart_comm.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "UART_COMM";

esp_err_t uart_comm_init(uart_port_t uart_num, const uart_config_t *config,
                         int tx_pin, int rx_pin) {
  if (uart_num >= UART_NUM_MAX || config == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  // Initialize UART with provided configuration
  esp_err_t ret = uart_param_config(uart_num, config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "UART parameter configuration failed");
    return ret;
  }

  // Set UART pins
  ret = uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE,
                     UART_PIN_NO_CHANGE);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "UART pin configuration failed");
    return ret;
  }

  // Install UART driver
  const int buf_size = 1024;
  ret = uart_driver_install(uart_num, buf_size * 2, buf_size * 2, 0, NULL, 0);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "UART driver installation failed");
    return ret;
  }

  ESP_LOGI(TAG, "UART%d initialized (TX:%d, RX:%d, Baud:%d)", uart_num, tx_pin,
           rx_pin, config->baud_rate);

  return ESP_OK;
}

esp_err_t uart_comm_write(uart_port_t uart_num, const uint8_t *data,
                          size_t len) {
  if (uart_num >= UART_NUM_MAX || data == NULL || len == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  int written = uart_write_bytes(uart_num, (const char *)data, len);
  if (written < 0) {
    ESP_LOGE(TAG, "UART write failed");
    return ESP_FAIL;
  }

  return ESP_OK;
}

esp_err_t uart_comm_read(uart_port_t uart_num, uint8_t *data, size_t max_len,
                         size_t *bytes_read, uint32_t timeout_ms) {
  if (uart_num >= UART_NUM_MAX || data == NULL || max_len == 0 ||
      bytes_read == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  *bytes_read = 0;

  // Set timeout
  TickType_t ticks_to_wait = timeout_ms / portTICK_PERIOD_MS;
  if (timeout_ms > 0 && ticks_to_wait == 0) {
    ticks_to_wait = 1; // Minimum wait time is 1 tick
  }

  // Read data
  int read_len = uart_read_bytes(uart_num, data, max_len, ticks_to_wait);
  if (read_len < 0) {
    ESP_LOGE(TAG, "UART read failed");
    return ESP_FAIL;
  }

  if (read_len == 0 && timeout_ms > 0) {
    return ESP_ERR_TIMEOUT;
  }

  *bytes_read = read_len;
  return ESP_OK;
}

esp_err_t uart_comm_read_line(uart_port_t uart_num, char *data, size_t max_len,
                              uint32_t timeout_ms) {
  if (uart_num >= UART_NUM_MAX || data == NULL ||
      max_len < 2) { // Need space for at least one char and null terminator
    return ESP_ERR_INVALID_ARG;
  }

  memset(data, 0, max_len);

  size_t bytes_read = 0;
  char c;
  TickType_t start_ticks = xTaskGetTickCount();
  TickType_t timeout_ticks = timeout_ms / portTICK_PERIOD_MS;

  while (bytes_read < max_len - 1) { // Leave space for null terminator
    // Calculate remaining timeout
    TickType_t elapsed_ticks = xTaskGetTickCount() - start_ticks;
    if (elapsed_ticks >= timeout_ticks) {
      if (bytes_read == 0) {
        return ESP_ERR_TIMEOUT;
      }
      break; // Partial line due to timeout
    }

    TickType_t remaining_ticks = timeout_ticks - elapsed_ticks;

    // Read one byte
    size_t read_bytes = 0;
    esp_err_t ret = uart_comm_read(uart_num, (uint8_t *)&c, 1, &read_bytes,
                                   remaining_ticks * portTICK_PERIOD_MS);

    if (ret == ESP_ERR_TIMEOUT) {
      if (bytes_read == 0) {
        return ESP_ERR_TIMEOUT;
      }
      break; // Partial line due to timeout
    }

    if (ret != ESP_OK || read_bytes != 1) {
      continue;
    }

    // Store the byte
    data[bytes_read++] = c;

    // Check for end of line
    if (c == '\n' || c == '\r') {
      break;
    }
  }

  // Null-terminate the string
  data[bytes_read] = '\0';

  // Trim trailing carriage return and newline
  if (bytes_read > 0) {
    size_t trim_pos = bytes_read;
    while (trim_pos > 0 &&
           (data[trim_pos - 1] == '\r' || data[trim_pos - 1] == '\n')) {
      data[--trim_pos] = '\0';
    }
  }

  return ESP_OK;
}

esp_err_t uart_comm_deinit(uart_port_t uart_num) {
  if (uart_num >= UART_NUM_MAX) {
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t ret = uart_driver_delete(uart_num);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "UART driver deletion failed");
  } else {
    ESP_LOGI(TAG, "UART%d deinitialized", uart_num);
  }

  return ret;
}
