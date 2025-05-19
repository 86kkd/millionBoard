#include "gps.h"
#include "esp_log.h"
#include "math.h"
#include "string.h"
#include "uart_comm.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "pa1010d.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "i2c_comm.h"
#define PA1010D_I2C_ADDR 0x10
#define PA1010D_I2C_TIMEOUT_MS 1000

static const char *TAG = "GPS";

// 实现 PA1010D 发送命令函数
esp_err_t pa1010d_send_command(pa1010d_handle_t handle, const char *command)
{
    if (handle == NULL || command == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 获取命令长度
    size_t cmd_len = strlen(command);
    
    // 通过通信组件发送命令到PA1010D
    esp_err_t ret = i2c_comm_write(PA1010D_I2C_ADDR,
                                   (const uint8_t *)command,
                                   cmd_len,
                                   PA1010D_I2C_TIMEOUT_MS);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send command to PA1010D: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Command sent to PA1010D: %s", command);
    }
    
    return ret;
}

// 卫星系统类型
typedef enum {
    SAT_SYS_UNKNOWN = 0,
    SAT_SYS_GPS,
    SAT_SYS_GLONASS,
    SAT_SYS_BEIDOU,
    SAT_SYS_GALILEO,
    SAT_SYS_GNSS
} pa_satellite_system_t;

// GPS 信息结构体
typedef struct {
    bool has_fix;
    int num_satellites;
    float latitude;
    float longitude;
    pa_satellite_system_t system;
} pa1010d_gps_info_t;

static const char *PA_TAG = "PA1010D_GPS";
static pa1010d_handle_t pa_handle = NULL;

static pa_satellite_system_t identify_pa_system(const char *msg) {
    if (!msg || msg[0] != '$') return SAT_SYS_UNKNOWN;
    if (strncmp(msg, "$GP", 3) == 0) return SAT_SYS_GPS;
    if (strncmp(msg, "$GL", 3) == 0) return SAT_SYS_GLONASS;
    if (strncmp(msg, "$GB", 3) == 0) return SAT_SYS_BEIDOU;
    if (strncmp(msg, "$GA", 3) == 0) return SAT_SYS_GALILEO;
    if (strncmp(msg, "$GN", 3) == 0) return SAT_SYS_GNSS;
    return SAT_SYS_UNKNOWN;
}

// 获取系统名称
static const char* get_pa_system_name(pa_satellite_system_t sys) {
    switch (sys) {
        case SAT_SYS_GPS: return "GPS";
        case SAT_SYS_GLONASS: return "GLONASS";
        case SAT_SYS_BEIDOU: return "北斗";
        case SAT_SYS_GALILEO: return "Galileo";
        case SAT_SYS_GNSS: return "多系统";
        default: return "未知";
    }
}

// 解析 NMEA 消息
static void parse_pa_nmea_message(const char *msg, pa1010d_gps_info_t *info) {
    if (!msg || !info) return;
    info->system = identify_pa_system(msg);
    // GGA 消息解析可见卫星数和定位状态
    if (strstr(msg, "GGA")) {
        char buf[200]; strcpy(buf, msg);
        char *token = strtok(buf, ",");
        int idx = 0;
        char *fields[15] = {0};
        while (token && idx < 15) {
            fields[idx++] = token;
            token = strtok(NULL, ",");
        }
        if (idx > 7) {
            info->num_satellites = atoi(fields[7]);
            int fix = atoi(fields[6]);
            info->has_fix = (fix > 0);
            if (fields[2] && fields[4]) {
                info->latitude = atof(fields[2]);
                info->longitude = atof(fields[4]);
            }
        }
    }
    // GSV 可见卫星总数
    else if (strstr(msg, "GSV")) {
        char buf[200]; strcpy(buf, msg);
        char *token = strtok(buf, ",");
        int idx = 0;
        char *fields[5] = {0};
        while (token && idx < 5) {
            fields[idx++] = token;
            token = strtok(NULL, ",*");
        }
        if (idx > 3) info->num_satellites = atoi(fields[3]);
    }
    // GSA 定位类型
    else if (strstr(msg, "GSA")) {
        char buf[200]; strcpy(buf, msg);
        char *token = strtok(buf, ",");
        int idx = 0;
        char *fields[3] = {0};
        while (token && idx < 3) {
            fields[idx++] = token;
            token = strtok(NULL, ",");
        }
        if (idx > 2) info->has_fix = (atoi(fields[2]) > 1);
    }
}

// 打印 GPS 状态
static void print_pa_gps_status(const pa1010d_gps_info_t *info) {
    if (!info) return;
    ESP_LOGI(PA_TAG, "===========================");
    ESP_LOGI(PA_TAG, "系统: %s", get_pa_system_name(info->system));
    ESP_LOGI(PA_TAG, "定位: %s", info->has_fix ? "已锁定" : "未锁定");
    ESP_LOGI(PA_TAG, "卫星数: %d", info->num_satellites);
    if (info->has_fix) {
        ESP_LOGI(PA_TAG, "经度: %.6f, 纬度: %.6f", info->longitude, info->latitude);
    }
    ESP_LOGI(PA_TAG, "===========================");
}

// 初始化 PA1010D GPS 并创建 FreeRTOS 任务
esp_err_t pa1010d_gps_init(void) {
    if (pa_handle) return ESP_OK;
    pa1010d_config_t cfg = {.i2c_port = I2C_NUM_0, .i2c_dev_addr = 0x10};
    esp_err_t ret = pa1010d_init(&cfg, &pa_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(PA_TAG, "init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    const char *cmd = "$PMTK353,1,1,0,1,0*2B\r\n";
    ret = pa1010d_send_command(pa_handle, cmd);
    if (ret != ESP_OK) ESP_LOGE(PA_TAG, "send cmd failed: %s", esp_err_to_name(ret));
    xTaskCreate(pa1010d_gps_task, "pa1010d_gps", 4096, NULL, 2, NULL);
    return ret;
}

// PA1010D GPS FreeRTOS 任务函数
void pa1010d_gps_task(void *pvParameters) {
    char msg[200];
    int count = 0;
    pa1010d_gps_info_t info = {0};
    while (1) {
        if (pa1010d_get_nmea_msg(pa_handle, msg, sizeof(msg), 1000) == ESP_OK) {
            // 打印原始 NMEA 消息
            ESP_LOGI(PA_TAG, "Got message: '%s'", msg);
            parse_pa_nmea_message(msg, &info);
            if (++count >= 10) {
                print_pa_gps_status(&info);
                count = 0;
            }
        } else {
            ESP_LOGE(PA_TAG, "get nmea msg failed");
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}