#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "pa1010d.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "example";

// 定义卫星系统类型
typedef enum {
    SAT_SYS_UNKNOWN = 0,
    SAT_SYS_GPS,      // GPS (美国)
    SAT_SYS_GLONASS,  // GLONASS (俄罗斯)
    SAT_SYS_BEIDOU,   // 北斗 (中国)
    SAT_SYS_GALILEO,  // Galileo (欧洲)
    SAT_SYS_GNSS      // 全球导航卫星系统组合
} satellite_system_t;

// 解析NMEA数据结构
typedef struct {
    bool has_fix;            // 是否有位置锁定
    int num_satellites;      // 当前可见卫星数量
    float latitude;          // 纬度
    float longitude;         // 经度
    satellite_system_t system; // 卫星系统类型
} gps_data_t;

// 自定义函数：发送命令到PA1010D模块
esp_err_t pa1010d_send_command(pa1010d_handle_t handle, const char *command)
{
    if (handle == NULL || command == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 在当前应用中，我们知道GPS模块使用的I2C端口和地址
    // 直接使用已知的I2C端口和地址信息
    i2c_port_t i2c_port = I2C_NUM_0;
    uint8_t device_addr = 0x10;
    
    // 获取命令长度
    size_t cmd_len = strlen(command);
    
    // 通过I2C发送命令到PA1010D
    esp_err_t ret = i2c_master_write_to_device(
        i2c_port,
        device_addr,
        (const uint8_t *)command,
        cmd_len,
        pdMS_TO_TICKS(1000));
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send command to PA1010D: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Command sent: %s", command);
    }
    
    return ret;
}

// 检查字符串是否包含字符
bool contains_char(const char *str, char ch) {
    return strchr(str, ch) != NULL;
}

// 根据NMEA语句前缀识别卫星系统
satellite_system_t identify_satellite_system(const char *nmea_msg) {
    if (nmea_msg[0] != '$') {
        return SAT_SYS_UNKNOWN;
    }
    
    // 根据NMEA前缀标识卫星系统
    // $GP: GPS
    // $GL: GLONASS
    // $GB: 北斗
    // $GA: Galileo
    // $GN: 多系统组合
    
    if (strncmp(nmea_msg, "$GP", 3) == 0) {
        return SAT_SYS_GPS;
    } else if (strncmp(nmea_msg, "$GL", 3) == 0) {
        return SAT_SYS_GLONASS;
    } else if (strncmp(nmea_msg, "$GB", 3) == 0) {
        return SAT_SYS_BEIDOU;
    } else if (strncmp(nmea_msg, "$GA", 3) == 0) {
        return SAT_SYS_GALILEO;
    } else if (strncmp(nmea_msg, "$GN", 3) == 0) {
        return SAT_SYS_GNSS;
    }
    
    return SAT_SYS_UNKNOWN;
}

// 获取卫星系统名称
const char* get_system_name(satellite_system_t sys) {
    switch (sys) {
        case SAT_SYS_GPS:     return "GPS (美国)";
        case SAT_SYS_GLONASS: return "GLONASS (俄罗斯)";
        case SAT_SYS_BEIDOU:  return "北斗 (中国)";
        case SAT_SYS_GALILEO: return "Galileo (欧洲)";
        case SAT_SYS_GNSS:    return "多系统组合";
        default:              return "未知系统";
    }
}

// 解析NMEA消息
void parse_nmea_message(const char *nmea_msg, gps_data_t *data) {
    // 识别卫星系统
    data->system = identify_satellite_system(nmea_msg);
    
    // GGA消息包含位置信息和卫星数量
    if (strstr(nmea_msg, "GGA")) {
        // 示例: $GNGGA,000604.887,,,,,0,0,,,M,,M,,*53
        // 解析字段 (逗号分隔)
        char *tokens[15] = {0};
        char msg_copy[200];
        strcpy(msg_copy, nmea_msg);
        
        char *token = strtok(msg_copy, ",");
        int i = 0;
        
        while (token != NULL && i < 15) {
            tokens[i++] = token;
            token = strtok(NULL, ",");
        }
        
        // 检查是否有可用卫星
        if (i >= 8) {
            data->num_satellites = atoi(tokens[7]);
            
            // 检查是否定位成功 (字段6: 0=无效定位, 1=有效定位)
            int fix_quality = (i >= 7) ? atoi(tokens[6]) : 0;
            data->has_fix = (fix_quality > 0);
            
            // 如果有经纬度数据，解析它们
            if (i >= 5 && strlen(tokens[2]) > 0 && strlen(tokens[4]) > 0) {
                // 这里简化处理，实际应用中应该更精确地解析
                data->latitude = atof(tokens[2]);
                data->longitude = atof(tokens[4]);
            }
        }
    }
    
    // GSV消息包含可见卫星数量
    else if (strstr(nmea_msg, "GSV")) {
        // 示例: $GPGSV,1,1,00*79
        char *tokens[5] = {0};
        char msg_copy[200];
        strcpy(msg_copy, nmea_msg);
        
        char *token = strtok(msg_copy, ",");
        int i = 0;
        
        while (token != NULL && i < 5) {
            tokens[i++] = token;
            token = strtok(NULL, ",*");
        }
        
        // GSV第三个字段是可见卫星总数
        if (i >= 4) {
            data->num_satellites = atoi(tokens[3]);
        }
    }
    
    // GSA消息包含定位状态
    else if (strstr(nmea_msg, "GSA")) {
        // 示例: $GPGSA,A,1,,,,,,,,,,,,,,,*1E
        char *tokens[3] = {0};
        char msg_copy[200];
        strcpy(msg_copy, nmea_msg);
        
        char *token = strtok(msg_copy, ",");
        int i = 0;
        
        while (token != NULL && i < 3) {
            tokens[i++] = token;
            token = strtok(NULL, ",");
        }
        
        // GSA第二个字段是定位模式, 第三个字段是定位类型 (1=无定位, 2=2D定位, 3=3D定位)
        if (i >= 3) {
            int fix_type = atoi(tokens[2]);
            data->has_fix = (fix_type > 1);
        }
    }
}

// 打印GPS状态信息
void print_gps_status(const gps_data_t *data) {
    ESP_LOGI(TAG, "==========================");
    ESP_LOGI(TAG, "卫星系统: %s", get_system_name(data->system));
    ESP_LOGI(TAG, "信号状态: %s", data->has_fix ? "已锁定 ✓" : "未锁定 ✗");
    ESP_LOGI(TAG, "可见卫星数: %d", data->num_satellites);
    
    if (data->has_fix) {
        ESP_LOGI(TAG, "纬度: %.6f", data->latitude);
        ESP_LOGI(TAG, "经度: %.6f", data->longitude);
    }
    ESP_LOGI(TAG, "==========================");
}

void app_main(void)
{
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 44,
        .scl_io_num = 43,
        .master.clk_speed = 100000,
    };

    ESP_ERROR_CHECK(
        i2c_param_config(I2C_NUM_0, &i2c_config));

    ESP_ERROR_CHECK(
        i2c_driver_install(I2C_NUM_0,
                           I2C_MODE_MASTER,
                           0, 0, 0));

    pa1010d_config_t config = {
        .i2c_port = I2C_NUM_0,
        .i2c_dev_addr = 0x10,
    };
    pa1010d_handle_t handle = NULL;
    ESP_ERROR_CHECK(pa1010d_init(&config, &handle));

    // 配置卫星系统
    // 选择一种配置方式：

    // 1. 仅使用GPS系统
    // const char *gps_only_cmd = "$PMTK353,1,0,0,0,0*2A\r\n";
    // ESP_ERROR_CHECK(pa1010d_send_command(handle, gps_only_cmd));

    // 2. 仅使用GLONASS系统
    // const char *glonass_only_cmd = "$PMTK353,0,1,0,0,0*2B\r\n";
    // ESP_ERROR_CHECK(pa1010d_send_command(handle, glonass_only_cmd));

    // 3. 使用GPS+GLONASS（默认配置）
    // const char *gps_glonass_cmd = "$PMTK353,1,1,0,0,0*2A\r\n";
    // ESP_ERROR_CHECK(pa1010d_send_command(handle, gps_glonass_cmd));

    // 4. 使用GPS+北斗
    // const char *gps_beidou_cmd = "$PMTK353,1,0,1,0,0*2B\r\n";
    // ESP_ERROR_CHECK(pa1010d_send_command(handle, gps_beidou_cmd));


    // 5. 使用GPS+Galileo
    // const char *gps_galileo_cmd = "$PMTK353,1,0,0,1,0*2B\r\n";
    // ESP_ERROR_CHECK(pa1010d_send_command(handle, gps_galileo_cmd));

    // 使用GPS+GLONASS+Galileo (PA1010D支持的系统)
    const char *supported_systems_cmd = "$PMTK353,1,1,0,1,0*2B\r\n";
    ESP_ERROR_CHECK(pa1010d_send_command(handle, supported_systems_cmd));


    // 延时一段时间，让模块处理命令
    vTaskDelay(pdMS_TO_TICKS(1000));

    char nmea_msg_buf[200];
    gps_data_t gps_data = {0};
    int msg_count = 0;
    
    while (true) {
        esp_err_t err = pa1010d_get_nmea_msg(handle,
                                             nmea_msg_buf,
                                             sizeof(nmea_msg_buf),
                                             1000);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Error getting NMEA message: 0x%x", err);
            continue;
        }

        ESP_LOGI(TAG, "Got message: '%s'", nmea_msg_buf);
        
        // 解析NMEA数据
        parse_nmea_message(nmea_msg_buf, &gps_data);
        
        // 每10条消息打印一次GPS状态汇总
        msg_count++;
        if (msg_count >= 10) {
            print_gps_status(&gps_data);
            msg_count = 0;
        }
    }
}
