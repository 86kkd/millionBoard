#include <esp_log.h>
#include "rc522.h"
#include "driver/rc522_spi.h"
#include "rc522_picc.h"
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "rc522-basic-spi-example";

#define RC522_SPI_BUS_GPIO_MISO    (3)
#define RC522_SPI_BUS_GPIO_MOSI    (8)
#define RC522_SPI_BUS_GPIO_SCLK    (40)
#define RC522_SPI_SCANNER_GPIO_SDA (39)
#define RC522_SCANNER_GPIO_RST     (2) // soft-reset
#define UNLOCK_GPIO (5)

static rc522_spi_config_t driver_config = {
    .host_id = SPI3_HOST,
    .bus_config = &(spi_bus_config_t){
        .miso_io_num = RC522_SPI_BUS_GPIO_MISO,
        .mosi_io_num = RC522_SPI_BUS_GPIO_MOSI,
        .sclk_io_num = RC522_SPI_BUS_GPIO_SCLK,
    },
    .dev_config = {
        .spics_io_num = RC522_SPI_SCANNER_GPIO_SDA,
    },
    .rst_io_num = RC522_SCANNER_GPIO_RST,
};

static rc522_driver_handle_t driver;
static rc522_handle_t scanner;

static rc522_picc_uid_t stored_uid = { .length = 0 };

static void on_picc_state_changed(void *arg, esp_event_base_t base, int32_t event_id, void *data)
{
    rc522_picc_state_changed_event_t *event = (rc522_picc_state_changed_event_t *)data;
    if (event->picc->state == RC522_PICC_STATE_ACTIVE) {
        rc522_picc_uid_t *uid = &event->picc->uid;
        if (stored_uid.length == 0) {
            memcpy(stored_uid.value, uid->value, uid->length);
            stored_uid.length = uid->length;
            char uid_str[RC522_PICC_UID_STR_BUFFER_SIZE_MAX];
            rc522_picc_uid_to_str(&stored_uid, uid_str, sizeof(uid_str));
            ESP_LOGI(TAG, "已注册卡片: %s", uid_str);
        } else if (stored_uid.length == uid->length && memcmp(stored_uid.value, uid->value, uid->length) == 0) {
            ESP_LOGI(TAG, "匹配成功，解锁中...");
            gpio_set_level(UNLOCK_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(2000));
            gpio_set_level(UNLOCK_GPIO, 0);
        } else {
            char uid_str[RC522_PICC_UID_STR_BUFFER_SIZE_MAX];
            rc522_picc_uid_to_str(uid, uid_str, sizeof(uid_str));
            ESP_LOGI(TAG, "未识别卡片: %s", uid_str);
        }
    } else if (event->picc->state == RC522_PICC_STATE_IDLE && event->old_state >= RC522_PICC_STATE_ACTIVE) {
        ESP_LOGI(TAG, "卡片已移除");
    }
}

void app_main()
{
    ESP_LOGI(TAG, "启动 RC522 SPI 示例");

    // 初始化解锁 GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << UNLOCK_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(UNLOCK_GPIO, 0);

    // 创建 SPI 驱动
    rc522_spi_create(&driver_config, &driver);
    rc522_driver_install(driver);

    // 配置和启动扫描器
    rc522_config_t scanner_config = {
        .driver = driver,
    };
    rc522_create(&scanner_config, &scanner);
    rc522_register_events(scanner, RC522_EVENT_PICC_STATE_CHANGED, on_picc_state_changed, NULL);
    rc522_start(scanner);
    ESP_LOGI(TAG, "扫描器初始化完成，等待卡片...");
}
