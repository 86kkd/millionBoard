#include "battery_mgr.h"
#include "esp_log.h"
#include "uart_comm.h"
#include <math.h>
#include <string.h>

static const char *TAG = "BATTERY_MGR";

// Battery status
static battery_status_t battery_status = {0};
static bool is_initialized = false;

// Callback function
static void (*battery_callback)(battery_status_t *, void *) = NULL;
static void *battery_callback_data = NULL;

// CRC8 calculation function for communication
static uint8_t calculate_crc8(uint8_t *data, size_t len) {
  uint8_t crc = 0;
  uint8_t polynomial = 0x07; // CRC8 polynomial x^8 + x^2 + x + 1

  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ polynomial;
      } else {
        crc = crc << 1;
      }
    }
  }

  return crc;
}

// Send command to battery management IC
static esp_err_t send_battery_command(uint8_t cmd, uint8_t *data, size_t len,
                                      uint8_t *response, size_t *resp_len) {
  if (!is_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  // Prepare command frame
  uint8_t frame[20] = {0};
  frame[0] = 0x1C; // Device address
  frame[1] = cmd;

  if (data && len > 0) {
    memcpy(&frame[2], data, len);
  }

  // Calculate CRC
  uint8_t crc = calculate_crc8(frame, 2 + len);
  frame[2 + len] = crc;

  // Send command
  esp_err_t ret = uart_comm_write(CONFIG_BATTERY_UART_PORT, frame, 3 + len);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to send battery command");
    return ret;
  }

  // Wait for response
  vTaskDelay(pdMS_TO_TICKS(50));

  // Read response
  uint8_t resp_buffer[100] = {0};
  size_t bytes_read = 0;

  ret = uart_comm_read(CONFIG_BATTERY_UART_PORT, resp_buffer,
                       sizeof(resp_buffer), &bytes_read, 100);
  if (ret != ESP_OK || bytes_read < 3) {
    ESP_LOGE(TAG, "Failed to read battery response");
    return ESP_FAIL;
  }

  // Verify response
  if (resp_buffer[0] != 0x1C || resp_buffer[1] != cmd) {
    ESP_LOGE(TAG, "Invalid battery response header");
    return ESP_FAIL;
  }

  // Verify CRC
  uint8_t received_crc = resp_buffer[bytes_read - 1];
  uint8_t calculated_crc = calculate_crc8(resp_buffer, bytes_read - 1);

  if (received_crc != calculated_crc) {
    ESP_LOGE(
        TAG,
        "Battery response CRC mismatch: received 0x%02X, calculated 0x%02X",
        received_crc, calculated_crc);
    return ESP_FAIL;
  }

  // Copy response data if needed
  if (response && resp_len) {
    *resp_len = bytes_read - 3; // Exclude address, command, and CRC
    if (*resp_len > 0) {
      memcpy(response, &resp_buffer[2], *resp_len);
    }
  }

  return ESP_OK;
}

// Initialize battery management
esp_err_t battery_mgr_init(void) {
  if (is_initialized) {
    return ESP_OK;
  }

  ESP_LOGI(TAG, "Initializing battery management");

  // Initialize UART for battery communication
  uart_config_t uart_config = {
      .baud_rate = CONFIG_BATTERY_UART_BAUD,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .rx_flow_ctrl_thresh = 0,
      .source_clk = UART_SCLK_APB,
  };

  esp_err_t ret =
      uart_comm_init(CONFIG_BATTERY_UART_PORT, &uart_config,
                     CONFIG_BATTERY_UART_TX_GPIO, CONFIG_BATTERY_UART_RX_GPIO);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize UART for battery communication");
    return ret;
  }

  // Read battery firmware version
  uint8_t response[10] = {0};
  size_t resp_len = 0;

  ret = send_battery_command(0x01, NULL, 0, response, &resp_len);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to read battery firmware version, continuing anyway");
    // Not critical, continue
  } else if (resp_len >= 1) {
    ESP_LOGI(TAG, "Battery firmware version: %d.%d", response[0] >> 4,
             response[0] & 0x0F);
  }

  // Initialize battery status
  battery_status.voltage = 0.0f;
  battery_status.current = 0.0f;
  battery_status.percentage = 0;
  battery_status.temperature = 25.0f;
  battery_status.cycle_count = 0;
  battery_status.is_charging = false;
  battery_status.is_fully_charged = false;
  battery_status.has_error = false;
  battery_status.error_code = 0;

  is_initialized = true;
  ESP_LOGI(TAG, "Battery management initialized successfully");

  return ESP_OK;
}

// Get battery status
esp_err_t battery_mgr_get_status(battery_status_t *status) {
  if (!is_initialized || status == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  // Read battery cell voltages
  uint8_t reg_addr = 0x10;
  uint8_t response[32] = {0};
  size_t resp_len = 0;

  esp_err_t ret = send_battery_command(0x01, &reg_addr, 1, response, &resp_len);
  if (ret == ESP_OK && resp_len >= 16) {
    // Calculate total voltage by summing cell voltages
    float total_voltage = 0.0f;
    uint8_t cells = 0;

    for (int i = 0; i < 8; i++) {
      if (i < resp_len / 2) {
        uint16_t cell_voltage = (response[i * 2] << 8) | response[i * 2 + 1];

        // Cell voltage in mV (formula from datasheet)
        float voltage_mv = cell_voltage * 5.0f / 32.0f;

        // Only count valid cells (non-zero voltage)
        if (voltage_mv > 500.0f) {
          total_voltage += voltage_mv / 1000.0f; // Convert to V
          cells++;
        }
      }
    }

    if (cells > 0) {
      battery_status.voltage = total_voltage;
    }
  } else {
    ESP_LOGW(TAG, "Failed to read battery cell voltages");
  }

  // Read battery current
  reg_addr = 0x20;
  ret = send_battery_command(0x01, &reg_addr, 1, response, &resp_len);
  if (ret == ESP_OK && resp_len >= 2) {
    int16_t current_raw = (response[0] << 8) | response[1];

    // Current calculation (formula from datasheet)
    // Assuming 1mΩ sense resistor
    battery_status.current =
        current_raw * 90.0f / 26214.4f / 0.001f / 1000.0f; // A
  } else {
    ESP_LOGW(TAG, "Failed to read battery current");
  }

  // Read battery status registers
  reg_addr = 0x01;
  ret = send_battery_command(0x01, &reg_addr, 1, response, &resp_len);
  if (ret == ESP_OK && resp_len >= 1) {
    uint8_t status_byte = response[0];

    // Parse status bits
    battery_status.has_error =
        (status_byte & 0x01) || // OV (over voltage)
        (status_byte & 0x02) || // UV (under voltage)
        (status_byte & 0x04) || // OCD1 (discharge overcurrent 1)
        (status_byte & 0x08) || // OCD2 (discharge overcurrent 2)
        (status_byte & 0x10) || // SC (short circuit)
        (status_byte & 0x20);   // OCC (charge overcurrent)

    if (battery_status.has_error) {
      battery_status.error_code = status_byte & 0x3F;
    } else {
      battery_status.error_code = 0;
    }
  }

  // Read battery status register 3 (charging status)
  reg_addr = 0x03;
  ret = send_battery_command(0x01, &reg_addr, 1, response, &resp_len);
  if (ret == ESP_OK && resp_len >= 1) {
    uint8_t status_byte = response[0];

    battery_status.is_charging = (status_byte & 0x01) != 0;

    // Read temperatures
    reg_addr = 0x21;
    ret = send_battery_command(0x01, &reg_addr, 1, response, &resp_len);
    if (ret == ESP_OK && resp_len >= 2) {
      uint16_t temp_raw = (response[0] << 8) | response[1];

      // Temperature calculation (simplified)
      if (temp_raw < 32768) {
        float rt = 10.0f * temp_raw / (32768.0f - temp_raw);

        // NTC 10K B3950 thermistor conversion (simplified)
        float log_rt = logf(rt / 10.0f);
        battery_status.temperature =
            1.0f / (3.354016E-3 + 2.569850E-4 * log_rt +
                    2.620131E-6 * log_rt * log_rt +
                    6.383091E-8 * log_rt * log_rt * log_rt) -
            273.15f;
      }
    }
  }

  // Estimate battery percentage based on voltage
  // This is a simple linear interpolation and would be more sophisticated in a
  // real implementation
  float min_voltage = 3.0f * 10.0f; // 10 cells at 3.0V (minimum)
  float max_voltage = 4.2f * 10.0f; // 10 cells at 4.2V (maximum)
  float percentage =
      ((battery_status.voltage - min_voltage) / (max_voltage - min_voltage)) *
      100.0f;

  if (percentage < 0.0f)
    percentage = 0.0f;
  if (percentage > 100.0f)
    percentage = 100.0f;

  battery_status.percentage = (uint8_t)percentage;

  // Copy status to output
  memcpy(status, &battery_status, sizeof(battery_status_t));

  return ESP_OK;
}
