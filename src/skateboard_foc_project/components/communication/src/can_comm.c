#include "can_comm.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "CAN_COMM";

// CAN configuration
#define CAN_TX_GPIO CONFIG_CAN_TX_GPIO
#define CAN_RX_GPIO CONFIG_CAN_RX_GPIO
#define CAN_BITRATE CONFIG_CAN_BITRATE

// Callback structure
typedef struct {
  uint32_t id;
  void (*callback)(uint32_t, uint8_t *, uint8_t, void *);
  void *user_data;
} can_callback_t;

// Global variables
static bool is_initialized = false;
static QueueHandle_t can_rx_queue = NULL;
static TaskHandle_t can_rx_task_handle = NULL;
static can_callback_t callbacks[10] = {0}; // Support up to 10 callbacks
static int num_callbacks = 0;

// Forward declarations
static void can_rx_task(void *pvParameters);

esp_err_t can_comm_init(void) {
  if (is_initialized) {
    return ESP_OK;
  }

  ESP_LOGI(TAG, "Initializing CAN communication");

  // Configure CAN
  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);

  // Configure CAN timing based on bitrate
  twai_timing_config_t t_config;

  switch (CAN_BITRATE) {
  case 125000:
    t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_125KBITS();
    break;
  case 250000:
    t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_250KBITS();
    break;
  case 500000:
    t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_500KBITS();
    break;
  case 1000000:
    t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_1MBITS();
    break;
  default:
    ESP_LOGE(TAG, "Unsupported CAN bitrate: %d", CAN_BITRATE);
    return ESP_ERR_INVALID_ARG;
  }

  // Accept all messages
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install CAN driver
  esp_err_t ret = twai_driver_install(&g_config, &t_config, &f_config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to install CAN driver: 0x%x", ret);
    return ret;
  }

  // Start CAN controller
  ret = twai_start();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start CAN: 0x%x", ret);
    twai_driver_uninstall();
    return ret;
  }

  // Create a queue for CAN messages
  can_rx_queue = xQueueCreate(20, sizeof(twai_message_t));
  if (can_rx_queue == NULL) {
    ESP_LOGE(TAG, "创建CAN RX队列失败");
    twai_stop();
    twai_driver_uninstall();
    return ESP_ERR_NO_MEM;
  }

  // Create a task to process CAN messages
  BaseType_t task_created =
      xTaskCreate(can_rx_task, "CAN_RX", 4096, NULL, 5, &can_rx_task_handle);
  if (task_created != pdPASS) {
    ESP_LOGE(TAG, "创建CAN RX任务失败");
    vQueueDelete(can_rx_queue);
    twai_stop();
    twai_driver_uninstall();
    return ESP_ERR_NO_MEM;
  }

  is_initialized = true;
  ESP_LOGI(
      TAG,
      "CAN 通信初始化成功 (TX:%d, RX:%d, Bitrate:%d)",
      CAN_TX_GPIO, CAN_RX_GPIO, CAN_BITRATE);

  return ESP_OK;
}

esp_err_t can_comm_send(uint32_t id, uint8_t *data, uint8_t length,
                        uint32_t timeout_ms) {
  if (!is_initialized || data == NULL || length > 8) {
    return ESP_ERR_INVALID_ARG;
  }

  // Prepare CAN message
  twai_message_t message;
  memset(&message, 0, sizeof(message));

  message.identifier = id;
  message.data_length_code = length;
  memcpy(message.data, data, length);

  // Check if it's an extended ID
  if (id > 0x7FF) {
    message.flags |= TWAI_MSG_FLAG_EXTD;
  }

  // Send message
  esp_err_t ret = twai_transmit(&message, pdMS_TO_TICKS(timeout_ms));
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to send CAN message: 0x%x", ret);
  }

  return ret;
}

esp_err_t can_comm_receive(uint32_t *id, uint8_t *data, uint8_t *length,
                           uint32_t timeout_ms) {
  if (!is_initialized || id == NULL || data == NULL || length == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  twai_message_t message;
  esp_err_t ret = twai_receive(&message, pdMS_TO_TICKS(timeout_ms));
  if (ret != ESP_OK) {
    if (ret == ESP_ERR_TIMEOUT) {
      // This is a normal timeout, no need to log as error
      return ESP_ERR_TIMEOUT;
    }
    ESP_LOGE(TAG, "Failed to receive CAN message: 0x%x", ret);
    return ret;
  }

  // Copy received data
  *id = message.identifier;
  *length = message.data_length_code;
  memcpy(data, message.data, message.data_length_code);

  return ESP_OK;
}

esp_err_t can_comm_register_callback(uint32_t id,
                                     void (*callback)(uint32_t, uint8_t *,
                                                      uint8_t, void *),
                                     void *user_data) {
  if (!is_initialized || callback == NULL || num_callbacks >= 10) {
    return ESP_ERR_INVALID_ARG;
  }

  // Add callback to the list
  callbacks[num_callbacks].id = id;
  callbacks[num_callbacks].callback = callback;
  callbacks[num_callbacks].user_data = user_data;
  num_callbacks++;

  ESP_LOGI(TAG, "Registered CAN callback for ID 0x%X", id);

  return ESP_OK;
}

esp_err_t can_comm_deinit(void) {
  if (!is_initialized) {
    return ESP_OK;
  }

  // Stop RX task
  if (can_rx_task_handle != NULL) {
    vTaskDelete(can_rx_task_handle);
    can_rx_task_handle = NULL;
  }

  // Delete queue
  if (can_rx_queue != NULL) {
    vQueueDelete(can_rx_queue);
    can_rx_queue = NULL;
  }

  // Stop CAN
  esp_err_t ret = twai_stop();
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to stop CAN: 0x%x", ret);
  }

  // Uninstall driver
  ret = twai_driver_uninstall();
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to uninstall CAN driver: 0x%x", ret);
  }

  is_initialized = false;
  num_callbacks = 0;
  ESP_LOGI(TAG, "CAN communication deinitialized");

  return ESP_OK;
}

esp_err_t can_comm_bus_recovery(void) {
  if (!is_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  ESP_LOGW(TAG, "Initiating CAN bus recovery");

  // Stop CAN
  esp_err_t ret = twai_stop();
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to stop CAN for recovery: 0x%x", ret);
  }

  // Brief delay
  vTaskDelay(pdMS_TO_TICKS(100));

  // Restart CAN
  ret = twai_start();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to restart CAN after recovery: 0x%x", ret);

    // Try a full reset
    ESP_LOGW(TAG, "Attempting full CAN reset");
    twai_driver_uninstall();

    vTaskDelay(pdMS_TO_TICKS(100));

    // Reconfigure CAN
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config;

    switch (CAN_BITRATE) {
    case 125000:
      t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_125KBITS();
      break;
    case 250000:
      t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_250KBITS();
      break;
    case 500000:
      t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_500KBITS();
      break;
    case 1000000:
      t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_1MBITS();
      break;
    default:
      ESP_LOGE(TAG, "Unsupported CAN bitrate: %d", CAN_BITRATE);
      return ESP_ERR_INVALID_ARG;
    }

    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ret = twai_driver_install(&g_config, &t_config, &f_config);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to reinstall CAN driver: 0x%x", ret);
      return ret;
    }

    ret = twai_start();
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to start CAN after reset: 0x%x", ret);
      return ret;
    }
  }

  ESP_LOGI(TAG, "CAN bus recovery completed");

  return ESP_OK;
}

// Task to receive CAN messages and call registered callbacks
static void can_rx_task(void *pvParameters) {
  twai_message_t message;

  while (1) {
    esp_err_t ret = twai_receive(&message, pdMS_TO_TICKS(100));

    if (ret == ESP_OK) {
      // Handle message
      for (int i = 0; i < num_callbacks; i++) {
        if (callbacks[i].id == 0 || callbacks[i].id == message.identifier) {
          callbacks[i].callback(message.identifier, message.data,
                                message.data_length_code,
                                callbacks[i].user_data);
        }
      }
    } else if (ret != ESP_ERR_TIMEOUT) {
      ESP_LOGW(TAG, "CAN receive error: 0x%x", ret);

      // Check for bus-off state
      twai_status_info_t status;
      twai_get_status_info(&status);

      if (status.state == TWAI_STATE_BUS_OFF) {
        ESP_LOGE(TAG, "CAN bus-off state detected, attempting recovery");
        can_comm_bus_recovery();
      }
    }

    // Small delay to prevent excessive CPU usage
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}
