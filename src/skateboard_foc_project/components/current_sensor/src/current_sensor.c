#include "current_sensor.h"
#include "esp_log.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "math.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "CURRENT_SENSOR";

#if CONFIG_ENABLE_CURRENT_SENSOR

// ADC校准句柄
static adc_cali_handle_t adc_cali_handle[4] = {NULL}; // 最多支持4个ADC通道(每个电机2个)

// ADC单次读取句柄
static adc_oneshot_unit_handle_t adc1_handle = NULL;

// 电流传感器通道配置
typedef struct {
    adc_channel_t adc_channel;    // ADC通道号
    adc_unit_t adc_unit;          // ADC单元号
    int zero_mv;                  // 零点参考电压(mV)
    float scale;                  // 电流比例系数(mA/mV)
    float filtered_value;         // 滤波后的电流值(mA)
    bool is_calculated;           // 是否是通过计算得到的值
} current_sensor_channel_t;

// 电机电流传感器状态
typedef struct {
    current_sensor_channel_t phase_u;
    current_sensor_channel_t phase_v;
    current_sensor_channel_t phase_w;
    bool calibrated;
} motor_current_sensor_t;

// 全局电流传感器状态
static struct {
    motor_current_sensor_t motors[2]; // 主电机和副电机
    bool initialized;
    float filter_alpha;
} current_sensor_state = {0};

// 校准ADC通道
static esp_err_t calibrate_adc_channel(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, int *out_handle_idx) {
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    
    #if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = unit,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
    if (ret == ESP_OK) {
        // 找到一个空闲的句柄位置
        for (int i = 0; i < 4; i++) {
            if (adc_cali_handle[i] == NULL) {
                adc_cali_handle[i] = handle;
                *out_handle_idx = i;
                break;
            }
        }
        ESP_LOGI(TAG, "ADC通道%d校准成功，使用Curve Fitting校准方案", channel);
        return ESP_OK;
    }
    #endif

    #if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = unit,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
    if (ret == ESP_OK) {
        // 找到一个空闲的句柄位置
        for (int i = 0; i < 4; i++) {
            if (adc_cali_handle[i] == NULL) {
                adc_cali_handle[i] = handle;
                *out_handle_idx = i;
                break;
            }
        }
        ESP_LOGI(TAG, "ADC通道%d校准成功，使用Line Fitting校准方案", channel);
        return ESP_OK;
    }
    #endif

    ESP_LOGW(TAG, "ADC通道%d校准失败！使用默认校准方案", channel);
    return ESP_FAIL;
}

// 读取ADC原始值并转换为电压
static esp_err_t read_adc_voltage(adc_channel_t channel, int cali_handle_idx, int *voltage_mv) {
    int adc_raw = 0;
    esp_err_t ret = adc_oneshot_read(adc1_handle, channel, &adc_raw);
    if (ret != ESP_OK) {
        return ret;
    }

    if (cali_handle_idx >= 0 && adc_cali_handle[cali_handle_idx] != NULL) {
        ret = adc_cali_raw_to_voltage(adc_cali_handle[cali_handle_idx], adc_raw, voltage_mv);
        if (ret != ESP_OK) {
            return ret;
        }
    } else {
        // 如果没有校准，使用简单线性转换
        *voltage_mv = (adc_raw * 1100) / 4095;  // 假设满量程为1.1V
    }

    return ESP_OK;
}

// 读取电流传感器
static esp_err_t read_current_sensor(current_sensor_channel_t *sensor, float *current_ma) {
    // 如果是计算得到的通道，不进行ADC读取
    if (sensor->is_calculated) {
        *current_ma = sensor->filtered_value;
        return ESP_OK;
    }

    int voltage_mv = 0;
    int cali_handle_idx = -1; // 未校准

    esp_err_t ret = read_adc_voltage(sensor->adc_channel, cali_handle_idx, &voltage_mv);
    if (ret != ESP_OK) {
        return ret;
    }

    // 计算电流值：(电压偏移量 * 比例系数)
    // 现在比例系数直接为mA/mV，不再需要单位转换
    float current = (voltage_mv - sensor->zero_mv) * sensor->scale;
    
    // 应用低通滤波
    sensor->filtered_value = current_sensor_state.filter_alpha * current + 
                            (1.0f - current_sensor_state.filter_alpha) * sensor->filtered_value;
    
    *current_ma = sensor->filtered_value;
    
    return ESP_OK;
}

// 初始化电流传感器
esp_err_t current_sensor_init(void) {
    if (current_sensor_state.initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "初始化电流传感器...");

    // 初始化过滤器系数
    current_sensor_state.filter_alpha = CONFIG_CURRENT_FILTER_ALPHA / 100.0f;

    // 配置ADC
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));

    // 配置通道
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = CONFIG_CURRENT_ADC_WIDTH,
        .atten = CONFIG_CURRENT_ADC_ATTEN,
    };

    // 获取电流比例系数（直接使用配置值，不再需要除以1000）
    float current_scale = (float)CONFIG_CURRENT_SCALE_FACTOR_MA_MV;

    // 配置主电机电流传感器
    current_sensor_state.motors[MOTOR_ID_PRIMARY].phase_u.adc_channel = CONFIG_MOTOR1_CURRENT_U_ADC_CHANNEL;
    current_sensor_state.motors[MOTOR_ID_PRIMARY].phase_u.adc_unit = ADC_UNIT_1;
    current_sensor_state.motors[MOTOR_ID_PRIMARY].phase_u.zero_mv = CONFIG_CURRENT_ZERO_REFERENCE_MV;
    current_sensor_state.motors[MOTOR_ID_PRIMARY].phase_u.scale = current_scale;
    current_sensor_state.motors[MOTOR_ID_PRIMARY].phase_u.is_calculated = false;
    
    // V相为计算得到的值
    current_sensor_state.motors[MOTOR_ID_PRIMARY].phase_v.zero_mv = CONFIG_CURRENT_ZERO_REFERENCE_MV;
    current_sensor_state.motors[MOTOR_ID_PRIMARY].phase_v.scale = current_scale;
    current_sensor_state.motors[MOTOR_ID_PRIMARY].phase_v.is_calculated = true;
    
    current_sensor_state.motors[MOTOR_ID_PRIMARY].phase_w.adc_channel = CONFIG_MOTOR1_CURRENT_W_ADC_CHANNEL;
    current_sensor_state.motors[MOTOR_ID_PRIMARY].phase_w.adc_unit = ADC_UNIT_1;
    current_sensor_state.motors[MOTOR_ID_PRIMARY].phase_w.zero_mv = CONFIG_CURRENT_ZERO_REFERENCE_MV;
    current_sensor_state.motors[MOTOR_ID_PRIMARY].phase_w.scale = current_scale;
    current_sensor_state.motors[MOTOR_ID_PRIMARY].phase_w.is_calculated = false;
    
    // 配置副电机电流传感器
    current_sensor_state.motors[MOTOR_ID_SECONDARY].phase_u.adc_channel = CONFIG_MOTOR2_CURRENT_U_ADC_CHANNEL;
    current_sensor_state.motors[MOTOR_ID_SECONDARY].phase_u.adc_unit = ADC_UNIT_1;
    current_sensor_state.motors[MOTOR_ID_SECONDARY].phase_u.zero_mv = CONFIG_CURRENT_ZERO_REFERENCE_MV;
    current_sensor_state.motors[MOTOR_ID_SECONDARY].phase_u.scale = current_scale;
    current_sensor_state.motors[MOTOR_ID_SECONDARY].phase_u.is_calculated = false;
    
    // V相为计算得到的值
    current_sensor_state.motors[MOTOR_ID_SECONDARY].phase_v.zero_mv = CONFIG_CURRENT_ZERO_REFERENCE_MV;
    current_sensor_state.motors[MOTOR_ID_SECONDARY].phase_v.scale = current_scale;
    current_sensor_state.motors[MOTOR_ID_SECONDARY].phase_v.is_calculated = true;
    
    current_sensor_state.motors[MOTOR_ID_SECONDARY].phase_w.adc_channel = CONFIG_MOTOR2_CURRENT_W_ADC_CHANNEL;
    current_sensor_state.motors[MOTOR_ID_SECONDARY].phase_w.adc_unit = ADC_UNIT_1;
    current_sensor_state.motors[MOTOR_ID_SECONDARY].phase_w.zero_mv = CONFIG_CURRENT_ZERO_REFERENCE_MV;
    current_sensor_state.motors[MOTOR_ID_SECONDARY].phase_w.scale = current_scale;
    current_sensor_state.motors[MOTOR_ID_SECONDARY].phase_w.is_calculated = false;

    // 只配置需要的ADC通道 (U和W相)
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, current_sensor_state.motors[MOTOR_ID_PRIMARY].phase_u.adc_channel, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, current_sensor_state.motors[MOTOR_ID_PRIMARY].phase_w.adc_channel, &config));
    
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, current_sensor_state.motors[MOTOR_ID_SECONDARY].phase_u.adc_channel, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, current_sensor_state.motors[MOTOR_ID_SECONDARY].phase_w.adc_channel, &config));

    current_sensor_state.initialized = true;
    ESP_LOGI(TAG, "电流传感器初始化完成");

    // 进行初始校准
    current_sensor_calibrate(MOTOR_ID_PRIMARY);
    current_sensor_calibrate(MOTOR_ID_SECONDARY);

    return ESP_OK;
}

// 获取三相电流 - 使用两相采样，计算第三相
esp_err_t current_sensor_get_three_phase_current(motor_id_t motor_id, float *current_u, float *current_v, float *current_w) {
    if (!current_sensor_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (motor_id != MOTOR_ID_PRIMARY && motor_id != MOTOR_ID_SECONDARY) {
        return ESP_ERR_INVALID_ARG;
    }

    if (current_u == NULL || current_v == NULL || current_w == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    motor_current_sensor_t *sensor = &current_sensor_state.motors[motor_id];
    
    // 读取两相电流 (U和W)
    esp_err_t ret_u = read_current_sensor(&sensor->phase_u, current_u);
    esp_err_t ret_w = read_current_sensor(&sensor->phase_w, current_w);
    
    if (ret_u != ESP_OK || ret_w != ESP_OK) {
        return ESP_FAIL;
    }
    
    // 计算第三相 (V) 电流: Ia + Ib + Ic = 0, 所以 Ib = -(Ia + Ic)
    float v_current = -(*current_u + *current_w);
    
    // 存储计算得到的V相电流
    sensor->phase_v.filtered_value = v_current;
    *current_v = v_current;

    ESP_LOGD(TAG, "电机%d电流: U=%.1fmA, V=%.1fmA, W=%.1fmA", 
             motor_id, *current_u, *current_v, *current_w);

    return ESP_OK;
}

// Clarke变换：将三相电流转换为Alpha-Beta坐标系
esp_err_t current_sensor_get_alpha_beta_current(motor_id_t motor_id, float *current_alpha, float *current_beta) {
    if (!current_sensor_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (motor_id != MOTOR_ID_PRIMARY && motor_id != MOTOR_ID_SECONDARY) {
        return ESP_ERR_INVALID_ARG;
    }

    if (current_alpha == NULL || current_beta == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    float i_a, i_b, i_c;
    esp_err_t ret = current_sensor_get_three_phase_current(motor_id, &i_a, &i_b, &i_c);
    if (ret != ESP_OK) {
        return ret;
    }

    // Clarke变换
    *current_alpha = i_a;
    *current_beta = (i_a + 2 * i_b) / sqrtf(3.0f);

    return ESP_OK;
}

// Park变换：将Alpha-Beta坐标系转换为DQ坐标系
esp_err_t current_sensor_get_dq_current(motor_id_t motor_id, float angle, float *current_d, float *current_q) {
    if (!current_sensor_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (motor_id != MOTOR_ID_PRIMARY && motor_id != MOTOR_ID_SECONDARY) {
        return ESP_ERR_INVALID_ARG;
    }

    if (current_d == NULL || current_q == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    float i_alpha, i_beta;
    esp_err_t ret = current_sensor_get_alpha_beta_current(motor_id, &i_alpha, &i_beta);
    if (ret != ESP_OK) {
        return ret;
    }

    // Park变换
    float sin_angle = sinf(angle);
    float cos_angle = cosf(angle);

    *current_d = i_alpha * cos_angle + i_beta * sin_angle;
    *current_q = -i_alpha * sin_angle + i_beta * cos_angle;

    return ESP_OK;
}

// 校准电流传感器
esp_err_t current_sensor_calibrate(motor_id_t motor_id) {
    if (!current_sensor_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (motor_id != MOTOR_ID_PRIMARY && motor_id != MOTOR_ID_SECONDARY) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "校准电机%d电流传感器...", motor_id);

    motor_current_sensor_t *sensor = &current_sensor_state.motors[motor_id];
    
    // 一般校准前需要确保电机处于停止状态，没有电流流过
    const int num_samples = 16;
    int voltage_u_sum = 0, voltage_w_sum = 0;
    int voltage_u, voltage_w;
    int handle_idx = -1;  // 未校准

    // 读取多个样本并求平均值 (仅U和W相)
    for (int i = 0; i < num_samples; i++) {
        read_adc_voltage(sensor->phase_u.adc_channel, handle_idx, &voltage_u);
        read_adc_voltage(sensor->phase_w.adc_channel, handle_idx, &voltage_w);
        
        voltage_u_sum += voltage_u;
        voltage_w_sum += voltage_w;
        
        // 稍微延迟以获取更稳定的读数
        vTaskDelay(1);
    }

    // 更新零点参考电压
    sensor->phase_u.zero_mv = voltage_u_sum / num_samples;
    sensor->phase_w.zero_mv = voltage_w_sum / num_samples;
    
    // V相使用同样的零点参考电压（两相平均值）
    sensor->phase_v.zero_mv = (sensor->phase_u.zero_mv + sensor->phase_w.zero_mv) / 2;

    ESP_LOGI(TAG, "电机%d电流传感器校准完成，参考电压 U:%dmV, V(计算):%dmV, W:%dmV", 
            motor_id, sensor->phase_u.zero_mv, sensor->phase_v.zero_mv, sensor->phase_w.zero_mv);

    sensor->calibrated = true;
    return ESP_OK;
}

#else

// 当电流传感器功能被禁用时，提供空实现

esp_err_t current_sensor_init(void) {
    ESP_LOGW(TAG, "电流传感器功能已禁用");
    return ESP_OK;
}

esp_err_t current_sensor_get_three_phase_current(motor_id_t motor_id, float *current_u, float *current_v, float *current_w) {
    if (current_u) *current_u = 0;
    if (current_v) *current_v = 0;
    if (current_w) *current_w = 0;
    return ESP_OK;
}

esp_err_t current_sensor_get_alpha_beta_current(motor_id_t motor_id, float *current_alpha, float *current_beta) {
    if (current_alpha) *current_alpha = 0;
    if (current_beta) *current_beta = 0;
    return ESP_OK;
}

esp_err_t current_sensor_get_dq_current(motor_id_t motor_id, float angle, float *current_d, float *current_q) {
    if (current_d) *current_d = 0;
    if (current_q) *current_q = 0;
    return ESP_OK;
}

esp_err_t current_sensor_calibrate(motor_id_t motor_id) {
    return ESP_OK;
}

#endif 