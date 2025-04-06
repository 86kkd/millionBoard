#pragma once

#include "driver/twai.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// CAN ID definitions
#define CAN_ID_ANGLE_SENSOR_DATA 0x185 // Incline sensor data ID

/**
 * @brief Initialize CAN communication
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t can_comm_init(void);

/**
 * @brief Send CAN message
 *
 * @param id CAN message ID
 * @param data Message data
 * @param length Data length (max 8 bytes)
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t ESP_OK on success
 */
esp_err_t can_comm_send(uint32_t id, uint8_t *data, uint8_t length,
                        uint32_t timeout_ms);

/**
 * @brief Receive CAN message
 *
 * @param id Pointer to store received CAN ID
 * @param data Data buffer
 * @param length Pointer to store received data length
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT on timeout
 */
esp_err_t can_comm_receive(uint32_t *id, uint8_t *data, uint8_t *length,
                           uint32_t timeout_ms);

/**
 * @brief Register CAN receive callback
 *
 * @param id CAN ID to listen for, use 0 for all IDs
 * @param callback Callback function
 * @param user_data User data to pass to callback
 * @return esp_err_t ESP_OK on success
 */
esp_err_t can_comm_register_callback(uint32_t id,
                                     void (*callback)(uint32_t, uint8_t *,
                                                      uint8_t, void *),
                                     void *user_data);

/**
 * @brief Deinitialize CAN communication
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t can_comm_deinit(void);

/**
 * @brief Handle bus recovery in case of errors
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t can_comm_bus_recovery(void);

#ifdef __cplusplus
}
#endif