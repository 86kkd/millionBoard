#include "utils.h"
#include "config.h"
#include "driver/gpio.h"

#define TAG "utils"

// 定义计数器变量
static volatile uint32_t rx_changes = 0;
static volatile uint32_t tx_changes = 0;

// GPIO中断处理函数
static void IRAM_ATTR gpio_isr_handler(void *arg) {
  uint32_t gpio_num = (uint32_t)arg;
  if (gpio_num == RX_GPIO_NUM) {
    rx_changes++;
  } else if (gpio_num == TX_GPIO_NUM) {
    tx_changes++;
  }
}

// 设置GPIO监视
void setup_pin_monitor() {
  // RX引脚配置
  gpio_config_t rx_conf = {.pin_bit_mask = (1ULL << RX_GPIO_NUM),
                           .mode = GPIO_MODE_INPUT,
                           .pull_up_en = GPIO_PULLUP_DISABLE,
                           .pull_down_en = GPIO_PULLDOWN_DISABLE,
                           .intr_type = GPIO_INTR_ANYEDGE};
  gpio_config(&rx_conf);

  // TX引脚配置
  gpio_config_t tx_conf = {.pin_bit_mask = (1ULL << TX_GPIO_NUM),
                           .mode = GPIO_MODE_INPUT, // 设为输入模式以监视
                           .pull_up_en = GPIO_PULLUP_DISABLE,
                           .pull_down_en = GPIO_PULLDOWN_DISABLE,
                           .intr_type = GPIO_INTR_ANYEDGE};
  gpio_config(&tx_conf);

  // 安装GPIO中断服务
  gpio_install_isr_service(0);

  // 添加中断处理程序
  gpio_isr_handler_add(RX_GPIO_NUM, gpio_isr_handler, (void *)RX_GPIO_NUM);
  gpio_isr_handler_add(TX_GPIO_NUM, gpio_isr_handler, (void *)TX_GPIO_NUM);
}

// 监视任务
void twai_monitor_task(void *pvParameters) {
  uint32_t last_rx = 0;
  uint32_t last_tx = 0;

  while (1) {
    uint32_t current_rx = rx_changes;
    uint32_t current_tx = tx_changes;

    // 计算每秒的变化次数
    uint32_t rx_rate = current_rx - last_rx;
    uint32_t tx_rate = current_tx - last_tx;

    ESP_LOGI(TAG, "RX changes/s: %lu, TX changes/s: %lu", rx_rate, tx_rate);

    // 更新上次的计数
    last_rx = current_rx;
    last_tx = current_tx;

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
