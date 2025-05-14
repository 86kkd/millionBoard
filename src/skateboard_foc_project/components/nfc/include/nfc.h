#pragma once

#include "driver/rc522_spi.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "rc522.h"
#include "rc522_picc.h"
#include <esp_event.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Forward declarations for ESP event types
 */
typedef const char *esp_event_base_t;
typedef void (*esp_event_handler_t)(void *handler_arg,
                                    esp_event_base_t event_base,
                                    int32_t event_id, void *event_data);

/**
 * @brief NFC event base
 */
extern const esp_event_base_t NFC_EVENTS;

/**
 * @brief NFC event types
 */
typedef enum {
  NFC_EVENT_TAG_DETECTED, /**< Tag detected in the field */
  NFC_EVENT_TAG_REMOVED,  /**< Tag removed from the field */
} nfc_event_t;

/**
 * @brief NFC tag data structure
 */
typedef struct {
  uint8_t uid[10];     /**< UID buffer */
  uint8_t uid_length;  /**< UID length */
  uint8_t data[16];    /**< Tag data buffer */
  uint8_t data_length; /**< Tag data length */
} nfc_tag_data_t;

/**
 * @brief Configuration for NFC module
 */
typedef struct {
  uint8_t i2c_addr;          /**< I2C address of PN532 */
  uint32_t poll_interval_ms; /**< Polling interval in milliseconds */
  size_t task_stack_size;    /**< FreeRTOS task stack size */
  UBaseType_t task_priority; /**< FreeRTOS task priority */
} nfc_config_t;

/**
 * @brief NFC handle type
 */
typedef struct nfc *nfc_handle_t;

/**
 * @brief Create and initialize NFC module
 *
 * @param config Configuration parameters
 * @param out_handle Returned handle for the NFC instance
 * @return esp_err_t ESP_OK on success
 */
esp_err_t nfc_create(const nfc_config_t *config, nfc_handle_t *out_handle);

/**
 * @brief Start NFC polling
 *
 * @param handle NFC instance handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t nfc_start(nfc_handle_t handle);

/**
 * @brief Pause NFC polling
 *
 * @param handle NFC instance handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t nfc_pause(nfc_handle_t handle);

/**
 * @brief Destroy NFC module and free resources
 *
 * @param handle NFC instance handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t nfc_destroy(nfc_handle_t handle);

/**
 * @brief Register NFC events
 *
 * @param handle NFC instance handle
 * @param event Event type to register
 * @param handler Event handler
 * @param handler_arg User-provided argument for handler
 * @return esp_err_t ESP_OK on success
 */
esp_err_t nfc_register_events(nfc_handle_t handle, nfc_event_t event,
                              esp_event_handler_t handler, void *handler_arg);

/**
 * @brief Unregister NFC events
 *
 * @param handle NFC instance handle
 * @param event Event type to unregister
 * @param handler Event handler
 * @return esp_err_t ESP_OK on success
 */
esp_err_t nfc_unregister_events(nfc_handle_t handle, nfc_event_t event,
                                esp_event_handler_t handler);

#ifdef __cplusplus
}
#endif
