#include "nfc.h"
#include "esp_log.h"
#include "i2c_comm.h"

static const char *TAG = "NFC";

// PN532 commands
#define PN532_COMMAND_GETFIRMWAREVERSION 0x02
#define PN532_COMMAND_SAMCONFIGURATION 0x14
#define PN532_COMMAND_INLISTPASSIVETARGET 0x4A

// Global variables
static bool is_initialized = false;
static uint8_t authorized_uid[10][10]; // Store up to 10 authorized UIDs
static uint8_t authorized_uid_lengths[10];
static int num_authorized_uids = 0;

esp_err_t nfc_init(void) {
  if (is_initialized) {
    return ESP_OK; // Already initialized
  }

  ESP_LOGI(TAG, "Initializing NFC module");

  // Ensure I2C is initialized
  if (i2c_comm_init() != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize I2C");
    return ESP_FAIL;
  }

  uint8_t cmd_firmware[] = {0x00, 0x00, 0xFF, 0x02, 0xFE,
                            0xD4, 0x02, 0x2A, 0x00};
  uint8_t response[20];
  uint8_t resp_len = 0;

  // Get firmware version to check if NFC reader is responsive
  esp_err_t ret =
      i2c_comm_write_read(CONFIG_NFC_I2C_ADDR, cmd_firmware,
                          sizeof(cmd_firmware), response, &resp_len, 100);
  if (ret != ESP_OK || resp_len < 12) {
    ESP_LOGE(TAG, "Failed to get NFC firmware version");
    return ESP_FAIL;
  }

  // Check if response is valid
  if (response[7] != 0xD5 || response[8] != 0x03) {
    ESP_LOGE(TAG, "Invalid NFC firmware response");
    return ESP_FAIL;
  }

  // Print firmware version
  ESP_LOGI(TAG, "NFC firmware version: %d.%d", response[9], response[10]);

  // Configure the PN532 (Normal mode, waiting for commands)
  uint8_t cmd_sam[] = {0x00, 0x00, 0xFF, 0x05, 0xFB, 0xD4, 0x14,
                       0x01, 0x00, 0x00, 0x00, 0x10, 0x00};

  ret = i2c_comm_write_read(CONFIG_NFC_I2C_ADDR, cmd_sam, sizeof(cmd_sam),
                            response, &resp_len, 100);
  if (ret != ESP_OK || resp_len < 8) {
    ESP_LOGE(TAG, "Failed to configure NFC");
    return ESP_FAIL;
  }

  is_initialized = true;
  ESP_LOGI(TAG, "NFC initialized successfully");

  return ESP_OK;
}

esp_err_t nfc_read_passive_target(uint8_t *uid, uint8_t *uid_len) {
  if (!is_initialized || uid == NULL || uid_len == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  // Command to read passive target
  uint8_t cmd[] = {0x00, 0x00, 0xFF, 0x04, 0xFC, 0xD4,
                   0x4A, 0x01, 0x00, 0x00, 0xE1, 0x00};
  uint8_t response[30];
  uint8_t resp_len = 0;

  esp_err_t ret = i2c_comm_write_read(CONFIG_NFC_I2C_ADDR, cmd, sizeof(cmd),
                                      response, &resp_len, 300);
  if (ret != ESP_OK || resp_len < 11) {
    return ESP_ERR_NOT_FOUND; // No card in field or communication error
  }

  // Check if a card was found
  if (response[7] != 0xD5 || response[8] != 0x4B || response[9] != 0x01) {
    return ESP_ERR_NOT_FOUND;
  }

  // Extract UID
  *uid_len = response[12];
  if (*uid_len > 0 && *uid_len <= 10) {
    memcpy(uid, &response[13], *uid_len);

    ESP_LOGI(TAG, "Card found, UID length: %d", *uid_len);
    ESP_LOG_BUFFER_HEX(TAG, uid, *uid_len);

    return ESP_OK;
  }

  return ESP_ERR_INVALID_RESPONSE;
}

bool nfc_check_authorized(uint8_t *uid, uint8_t uid_len) {
  if (uid == NULL || uid_len == 0 || uid_len > 10) {
    return false;
  }

  // Check against authorized UIDs
  for (int i = 0; i < num_authorized_uids; i++) {
    if (uid_len == authorized_uid_lengths[i]) {
      if (memcmp(uid, authorized_uid[i], uid_len) == 0) {
        ESP_LOGI(TAG, "Authorized card detected");
        return true;
      }
    }
  }

  ESP_LOGW(TAG, "Unauthorized card detected");
  return false;
}

esp_err_t nfc_add_authorized_uid(uint8_t *uid, uint8_t uid_len) {
  if (uid == NULL || uid_len == 0 || uid_len > 10) {
    return ESP_ERR_INVALID_ARG;
  }

  if (num_authorized_uids >= 10) {
    ESP_LOGW(TAG, "Authorized UID list is full");
    return ESP_ERR_NO_MEM;
  }

  // Store the new authorized UID
  memcpy(authorized_uid[num_authorized_uids], uid, uid_len);
  authorized_uid_lengths[num_authorized_uids] = uid_len;
  num_authorized_uids++;

  ESP_LOGI(TAG, "Added new authorized UID");
  ESP_LOG_BUFFER_HEX(TAG, uid, uid_len);

  return ESP_OK;
}

esp_err_t nfc_clear_authorized_uids(void) {
  num_authorized_uids = 0;
  ESP_LOGI(TAG, "Cleared all authorized UIDs");
  return ESP_OK;
}