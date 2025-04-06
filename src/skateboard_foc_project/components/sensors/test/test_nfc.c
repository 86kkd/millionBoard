#include "esp_log.h"
#include "nfc.h"
#include "unity.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "NFC_TEST";

// 将十六进制字符串转换为字节数组
static void hex2bytes(const char *hex, uint8_t *bytes, size_t *len) {
  size_t hex_len = strlen(hex);
  size_t i;
  size_t byte_len = 0;

  for (i = 0; i < hex_len; i += 2) {
    if (i + 1 >= hex_len)
      break;

    char c1 = hex[i];
    char c2 = hex[i + 1];

    uint8_t b = 0;
    if (c1 >= '0' && c1 <= '9')
      b = (c1 - '0') << 4;
    else if (c1 >= 'a' && c1 <= 'f')
      b = (c1 - 'a' + 10) << 4;
    else if (c1 >= 'A' && c1 <= 'F')
      b = (c1 - 'A' + 10) << 4;

    if (c2 >= '0' && c2 <= '9')
      b |= (c2 - '0');
    else if (c2 >= 'a' && c2 <= 'f')
      b |= (c2 - 'a' + 10);
    else if (c2 >= 'A' && c2 <= 'F')
      b |= (c2 - 'A' + 10);

    bytes[byte_len++] = b;
  }

  *len = byte_len;
}

// 测试 NFC 初始化
TEST_CASE("NFC 初始化测试", "[nfc]") {
  esp_err_t ret = nfc_init();
  TEST_ASSERT_EQUAL(ESP_OK, ret);
}

// 测试添加授权 UID
TEST_CASE("NFC 添加授权 UID 测试", "[nfc]") {
  // 使用配置的测试 UID
  const char *test_uid_hex = CONFIG_NFC_TEST_UID;
  uint8_t test_uid[10];
  size_t uid_len = 0;

  hex2bytes(test_uid_hex, test_uid, &uid_len);

  esp_err_t ret = nfc_add_authorized_uid(test_uid, uid_len);
  TEST_ASSERT_EQUAL(ESP_OK, ret);

  // 验证 UID 是否被授权
  bool is_authorized = nfc_check_authorized(test_uid, uid_len);
  TEST_ASSERT_TRUE(is_authorized);

  // 使用不同的 UID 应该不被授权
  uint8_t different_uid[4] = {0x11, 0x22, 0x33, 0x44};
  is_authorized = nfc_check_authorized(different_uid, 4);
  TEST_ASSERT_FALSE(is_authorized);
}

// 测试清除授权 UID
TEST_CASE("NFC 清除授权 UID 测试", "[nfc]") {
  // 先添加一个 UID
  uint8_t test_uid[4] = {0xAA, 0xBB, 0xCC, 0xDD};
  esp_err_t ret = nfc_add_authorized_uid(test_uid, 4);
  TEST_ASSERT_EQUAL(ESP_OK, ret);

  // 清除所有授权 UID
  ret = nfc_clear_authorized_uids();
  TEST_ASSERT_EQUAL(ESP_OK, ret);

  // 验证 UID 不再被授权
  bool is_authorized = nfc_check_authorized(test_uid, 4);
  TEST_ASSERT_FALSE(is_authorized);
}

// 测试 NFC 读卡（这是一个手动测试，需要有卡）
TEST_CASE("NFC 读卡测试", "[nfc][manual]") {
  ESP_LOGI(TAG, "请将 NFC 卡放置在读卡器上...");

  // 等待一段时间以便放卡
  vTaskDelay(pdMS_TO_TICKS(3000));

  uint8_t uid[10];
  uint8_t uid_len = 0;

  esp_err_t ret = nfc_read_passive_target(uid, &uid_len);

  // 如果有卡，应该成功读取
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "成功读取卡片，UID 长度: %d", uid_len);
    ESP_LOG_BUFFER_HEX(TAG, uid, uid_len);
    TEST_ASSERT_TRUE(uid_len > 0);

    // 测试添加这张卡为授权卡
    ret = nfc_add_authorized_uid(uid, uid_len);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    // 验证是否授权成功
    bool is_authorized = nfc_check_authorized(uid, uid_len);
    TEST_ASSERT_TRUE(is_authorized);
  } else {
    ESP_LOGW(TAG, "未检测到卡片，跳过测试");
  }
}
