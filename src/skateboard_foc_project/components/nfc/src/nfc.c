#include "nfc.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c.h"
#include <string.h>
#include "driver/spi_master.h"
#include "driver/rc522_spi.h"
#include "rc522.h"
#include "rc522_picc.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>

static const char *TAG = "NFC";

// RC522 SPI 接口引脚定义，根据实际硬件连接修改
#define RC522_SPI_BUS_GPIO_MISO    3
#define RC522_SPI_BUS_GPIO_MOSI    8
#define RC522_SPI_BUS_GPIO_SCLK    40
#define RC522_SCANNER_GPIO_SDA 39
#define RC522_SCANNER_GPIO_RST     2

static rc522_driver_handle_t driver;
static rc522_handle_t scanner;

// NFC 认证回调句柄
static nfc_auth_cb_t auth_cb = NULL;

// 实现注册认证回调函数
esp_err_t nfc_register_auth_callback(nfc_auth_cb_t cb)
{
    auth_cb = cb;
    return ESP_OK;
}

static void on_picc_state_changed(void *arg, esp_event_base_t base, int32_t event_id, void *data)
{
    rc522_picc_state_changed_event_t *event = (rc522_picc_state_changed_event_t *)data;
    if (event->picc->state == RC522_PICC_STATE_ACTIVE) {
        rc522_picc_print(event->picc);
        // 授权卡检测
        const rc522_picc_uid_t *uid = &event->picc->uid;
        const uint8_t allowed_uid[4] = {0xA0, 0x12, 0x46, 0x4F};
        bool authorized = (uid->length == 4 &&
            uid->value[0] == allowed_uid[0] &&
            uid->value[1] == allowed_uid[1] &&
            uid->value[2] == allowed_uid[2] &&
            uid->value[3] == allowed_uid[3]);
        if (authorized) {
            ESP_LOGI(TAG, "Authorized card detected");
        } else {
            ESP_LOGW(TAG, "Unauthorized card");
        }
        if (auth_cb) {
            auth_cb(authorized);
        }
    } else if (event->picc->state == RC522_PICC_STATE_IDLE && event->old_state >= RC522_PICC_STATE_ACTIVE) {
        ESP_LOGI(TAG, "Card removed");
        if (auth_cb) {
            auth_cb(false);
        }
    }
}

static void nfc_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Initializing RC522 NFC module");

    rc522_spi_config_t driver_config = {
        .host_id = SPI2_HOST,
        .bus_config = &(spi_bus_config_t){
            .miso_io_num = RC522_SPI_BUS_GPIO_MISO,
            .mosi_io_num = RC522_SPI_BUS_GPIO_MOSI,
            .sclk_io_num = RC522_SPI_BUS_GPIO_SCLK,
        },
        .dev_config = {
            .spics_io_num = RC522_SCANNER_GPIO_SDA,
        },
        .rst_io_num = RC522_SCANNER_GPIO_RST,
    };

    // 创建 SPI 驱动并安装
    rc522_spi_create(&driver_config, &driver);
    rc522_driver_install(driver);

    // 初始化 RC522 扫描器
    rc522_config_t scanner_config = {
        .driver = driver,
    };
    rc522_create(&scanner_config, &scanner);
    rc522_register_events(scanner, RC522_EVENT_PICC_STATE_CHANGED, on_picc_state_changed, NULL);
    rc522_start(scanner);

    ESP_LOGI(TAG, "RC522 scanner started");

    // 保持任务运行
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

esp_err_t nfc_init(void)
{
    xTaskCreate(nfc_task, "nfc", 4096, NULL, 5, NULL);
    return ESP_OK;
}