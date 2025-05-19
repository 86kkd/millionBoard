#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize RC522 NFC module and start FreeRTOS task.
 *
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t nfc_init(void);

#ifdef __cplusplus
}
#endif
