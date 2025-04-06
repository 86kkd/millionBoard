#include "i2c_comm.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "I2C_COMM";

// Default I2C configuration
#define I2C_NUM I2C_NUM_0
#define I2C_SDA_PIN CONFIG_NFC_SDA_GPIO
#define I2C_SCL_PIN CONFIG_NFC_SCL_GPIO
#define I2C_FREQ_HZ 100000

static bool is_initialized = false;

esp_err_t i2c_comm_init(void) {
  if (is_initialized) {
    return ESP_OK;
  }

  ESP_LOGI(TAG, "Initializing I2C communication");

  // Configure I2C
  i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = I2C_SDA_PIN,
      .scl_io_num = I2C_SCL_PIN,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = I2C_FREQ_HZ,
  };

  esp_err_t ret = i2c_param_config(I2C_NUM, &conf);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "I2C parameter configuration failed");
    return ret;
  }

  ret = i2c_driver_install(I2C_NUM, I2C_MODE_MASTER, 0, 0, 0);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "I2C driver installation failed");
    return ret;
  }

  is_initialized = true;
  ESP_LOGI(
      TAG,
      "I2C communication initialized successfully (SDA:%d, SCL:%d, Freq:%d Hz)",
      I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);

  return ESP_OK;
}

esp_err_t i2c_comm_write(uint8_t addr, const uint8_t *data, size_t len,
                         uint32_t timeout_ms) {
  if (!is_initialized || data == NULL || len == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write(cmd, (uint8_t *)data, len, true);
  i2c_master_stop(cmd);

  esp_err_t ret = i2c_master_cmd_begin(I2C_NUM, cmd, pdMS_TO_TICKS(timeout_ms));
  i2c_cmd_link_delete(cmd);

  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "I2C write to addr 0x%02X failed: 0x%x", addr, ret);
  }

  return ret;
}

esp_err_t i2c_comm_read(uint8_t addr, uint8_t *data, size_t len,
                        uint32_t timeout_ms) {
  if (!is_initialized || data == NULL || len == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_stop(cmd);

  i2c_cmd_handle_t cmd_read = i2c_cmd_link_create();
  i2c_master_start(cmd_read);
  i2c_master_write_byte(cmd_read, (addr << 1) | I2C_MASTER_READ, true);

  size_t bytes_read = 0;
  while (bytes_read < len) {
    size_t remaining = len - bytes_read;
    esp_err_t ret = i2c_master_read(cmd_read, &data[bytes_read], remaining,
                                    i2c_ack_type_t(remaining > 1));
    if (ret != ESP_OK) {
      i2c_cmd_link_delete(cmd_read);
      return ret;
    }
    bytes_read += remaining;
  }

  i2c_master_stop(cmd_read);
  i2c_cmd_link_delete(cmd_read);

  return ESP_OK;
}

esp_err_t i2c_comm_write_read(uint8_t addr, const uint8_t *write_data,
                              size_t write_len, uint8_t *read_data,
                              uint8_t *read_len, uint32_t timeout_ms) {
  if (!is_initialized || write_data == NULL || write_len == 0 ||
      read_data == NULL || read_len == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write(cmd, (uint8_t *)write_data, write_len, true);
  i2c_master_stop(cmd);

  i2c_cmd_handle_t cmd_read = i2c_cmd_link_create();
  i2c_master_start(cmd_read);
  i2c_master_write_byte(cmd_read, (addr << 1) | I2C_MASTER_READ, true);

  size_t bytes_read = 0;
  while (bytes_read < *read_len) {
    size_t remaining = *read_len - bytes_read;
    esp_err_t ret = i2c_master_read(cmd_read, &read_data[bytes_read], remaining,
                                    i2c_ack_type_t(remaining > 1));
    if (ret != ESP_OK) {
      i2c_cmd_link_delete(cmd_read);
      return ret;
    }
    bytes_read += remaining;
  }

  i2c_master_stop(cmd_read);
  i2c_cmd_link_delete(cmd_read);

  *read_len = bytes_read;
  return ESP_OK;
}

esp_err_t i2c_comm_scan(void) {
  if (!is_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, 0x00, true);
  i2c_master_stop(cmd);

  i2c_cmd_handle_t cmd_read = i2c_cmd_link_create();
  i2c_master_start(cmd_read);
  i2c_master_write_byte(cmd_read, 0x00, true);

  uint8_t address;
  size_t bytes_read = 0;
  while (1) {
    esp_err_t ret = i2c_master_read(cmd_read, &address, 1,
                                    i2c_ack_type_t(bytes_read < 127));
    if (ret != ESP_OK) {
      i2c_cmd_link_delete(cmd_read);
      return ret;
    }
    if (address == 0x00) {
      break;
    }
    ESP_LOGI(TAG, "Found device at address 0x%02X", address);
    bytes_read++;
  }

  i2c_cmd_link_delete(cmd_read);
  return ESP_OK;
}

esp_err_t i2c_comm_deinit(void) {
  if (!is_initialized) {
    return ESP_OK;
  }

  esp_err_t ret = i2c_driver_delete(I2C_NUM);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "I2C driver deletion failed");
  } else {
    ESP_LOGI(TAG, "I2C communication deinitialized");
  }

  is_initialized = false;
  return ret;
}
