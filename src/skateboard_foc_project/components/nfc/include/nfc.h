#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize RC522 NFC module and start FreeRTOS task.
 *
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t nfc_init(void);

// 添加认证回调类型及注册函数
/**
 * @brief NFC 认证回调函数类型，当检测到卡片时会调用，参数表示是否通过认证
 */
typedef void (*nfc_auth_cb_t)(bool authorized);
/**
 * @brief 注册 NFC 认证回调
 *
 * @param cb 当检测到经过授权的卡片时调用 cb(true)，其他情况调用 cb(false)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t nfc_register_auth_callback(nfc_auth_cb_t cb);

#ifdef __cplusplus
}
#endif
