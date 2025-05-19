#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>
#include "pa1010d.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Send command to PA1010D GPS module.
 *
 * @param handle PA1010D driver handle.
 * @param command Command string to send (null-terminated).
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t pa1010d_send_command(pa1010d_handle_t handle, const char *command);

/**
 * @brief Initialize PA1010D GPS and I2C interface for FreeRTOS task.
 *
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t pa1010d_gps_init(void);

/**
 * @brief PA1010D GPS FreeRTOS task function.
 *
 * @param pvParameters Task parameters (unused).
 */
void pa1010d_gps_task(void *pvParameters);

#ifdef __cplusplus
}
#endif
