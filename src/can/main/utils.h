#ifndef UTILS_H
#define UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

// 包含需要的头文件
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// 任务函数声明
void setup_pin_monitor();
void twai_monitor_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif // UTILS_H
