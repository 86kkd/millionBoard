#include <stdio.h>
#include "esp_err.h"
#include "esp_log.h"
#include "hal/twai_types.h"
#include "driver/twai.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

void app_main(void);

#ifdef __cplusplus
}
#endif

// CAN 引脚定义
#define TX_GPIO_NUM     GPIO_NUM_5
#define RX_GPIO_NUM     GPIO_NUM_4
#define TX_TASK_PRIO    8
#define RX_TASK_PRIO    9

// CANopen ID定义
#define NODE_ID         0x05
#define PDO1_ID        (0x180 + NODE_ID)  // 0x185
#define PDO2_ID        (0x280 + NODE_ID)  // 0x285
#define PDO3_ID        (0x380 + NODE_ID)  // 0x385
#define SDO_TX_ID      (0x580 + NODE_ID)  // 0x585
#define SDO_RX_ID      (0x600 + NODE_ID)  // 0x605

// 传感器数据结构
typedef struct {
    float roll;    // X轴角度
    float pitch;   // Y轴角度 
    float yaw;     // Z轴角度
    float temp;    // 温度
    uint8_t status;// 状态
} sensor_data_t;

sensor_data_t sensor_data = {0};

// TWAI配置初始化
void twai_init(void)
{
    // 初始化TWAI配置
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);
    
    // 设置时序为250kbps
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
    
    // 过滤器配置 - 接收所有消息
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // 安装TWAI驱动
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    
    // 启动TWAI驱动
    ESP_ERROR_CHECK(twai_start());
}

// 解析PDO1数据
void parse_pdo1(uint8_t *data, sensor_data_t *sensor)
{
    // Roll角度解析 (B0-B1)
    int16_t roll_raw = (int16_t)(data[0] | (data[1] << 8));
    sensor->roll = roll_raw * 0.01f;
    
    // Pitch角度解析 (B2-B3)
    int16_t pitch_raw = (int16_t)(data[2] | (data[3] << 8));
    sensor->pitch = pitch_raw * 0.01f;
    
    // Yaw角度解析 (B4-B5)
    int16_t yaw_raw = (int16_t)(data[4] | (data[5] << 8));
    sensor->yaw = yaw_raw * 0.01f;
    
    // 温度解析 (B6)
    sensor->temp = (float)data[6] / 2.0f - 40.0f;
    
    // 状态解析 (B7)
    sensor->status = data[7];
}

// 发送SDO配置命令
esp_err_t send_sdo_command(uint16_t index, uint8_t subindex, uint32_t data)
{
    twai_message_t tx_msg;
    
    tx_msg.identifier = SDO_RX_ID;
    tx_msg.data_length_code = 8;
    tx_msg.flags = TWAI_MSG_FLAG_NONE;
    
    // SDO命令格式
    tx_msg.data[0] = 0x23;  // 写4字节命令
    tx_msg.data[1] = index & 0xFF;  // 索引低字节
    tx_msg.data[2] = (index >> 8) & 0xFF;  // 索引高字节
    tx_msg.data[3] = subindex;  // 子索引
    
    // 数据
    tx_msg.data[4] = data & 0xFF;
    tx_msg.data[5] = (data >> 8) & 0xFF;
    tx_msg.data[6] = (data >> 16) & 0xFF;
    tx_msg.data[7] = (data >> 24) & 0xFF;
    
    return twai_transmit(&tx_msg, pdMS_TO_TICKS(100));
}

// 接收任务
void twai_receive_task(void *arg)
{
    twai_message_t rx_msg;
    
    while (1) {
        esp_err_t ret = twai_receive(&rx_msg, pdMS_TO_TICKS(100));
        
        if (ret == ESP_OK) {
            // 处理接收到的消息
            switch (rx_msg.identifier) {
                case PDO1_ID:
                    parse_pdo1(rx_msg.data, &sensor_data);
                    break;
                    
                case SDO_TX_ID:
                    // 处理SDO响应
                    break;
                    
                default:
                    break;
            }
        }
    }
}

// 配置传感器示例
void config_sensor(void)
{
    // 设置波特率为250kbps
    send_sdo_command(0x1021, 0x00, 0x03);
    
    // 设置数据输出周期为10ms
    send_sdo_command(0x1023, 0x00, 0x0A);
    
    // 开启PDO1
    send_sdo_command(0x6000, 0x01, 0x01);
}

// 主程序
void app_main(void)
{
    // 初始化TWAI
    twai_init();
    
    // 配置传感器
    config_sensor();
    
    // 创建接收任务
    xTaskCreate(twai_receive_task, "twai_rx", 4096, NULL, RX_TASK_PRIO, NULL);

    // 主循环
    while(1) {
        printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f, Temp: %.1f\n",
               sensor_data.roll, sensor_data.pitch, 
               sensor_data.yaw, sensor_data.temp);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}