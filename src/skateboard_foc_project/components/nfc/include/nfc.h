#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief NFC 标签数据结构
 */
typedef struct {
  uint8_t uid[10];     // UID 数据缓冲区
  uint8_t uid_length;  // UID 实际长度
  uint8_t data[16];    // 标签数据
  uint8_t data_length; // 数据长度
} nfc_tag_data_t;

/**
 * @brief 初始化 NFC 模块
 *
 * @return esp_err_t ESP_OK 表示成功
 */
esp_err_t nfc_init(void);

/**
 * @brief 读取 NFC 被动目标卡片
 *
 * @param uid UID 缓冲区
 * @param uid_len UID 长度指针
 * @return esp_err_t ESP_OK 表示成功，ESP_ERR_NOT_FOUND 表示未找到卡片
 */
esp_err_t nfc_read_passive_target(uint8_t *uid, uint8_t *uid_len);

/**
 * @brief 检查卡片是否授权
 *
 * @param uid UID 缓冲区
 * @param uid_len UID 长度
 * @return bool 如果卡片授权返回 true
 */
bool nfc_check_authorized(uint8_t *uid, uint8_t uid_len);

/**
 * @brief 添加授权的 UID
 *
 * @param uid UID 缓冲区
 * @param uid_len UID 长度
 * @return esp_err_t ESP_OK 表示成功
 */
esp_err_t nfc_add_authorized_uid(uint8_t *uid, uint8_t uid_len);

/**
 * @brief 清除所有授权的 UID
 *
 * @return esp_err_t ESP_OK 表示成功
 */
esp_err_t nfc_clear_authorized_uids(void);

/**
 * @brief 读取 NFC 标签数据
 *
 * @param block_num 读取的块号
 * @param data 数据缓冲区
 * @param data_len 数据长度指针
 * @return esp_err_t ESP_OK 表示成功
 */
esp_err_t nfc_read_data(uint8_t block_num, uint8_t *data, uint8_t *data_len);

/**
 * @brief 写入 NFC 标签数据
 *
 * @param block_num 写入的块号
 * @param data 数据缓冲区
 * @param data_len 数据长度
 * @return esp_err_t ESP_OK 表示成功
 */
esp_err_t nfc_write_data(uint8_t block_num, const uint8_t *data,
                         uint8_t data_len);

#ifdef __cplusplus
}
#endif
