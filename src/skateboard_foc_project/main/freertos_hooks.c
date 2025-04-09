/**
 * FreeRTOS钩子函数实现
 *
 * 本文件提供了当前ESP-IDF配置所需的
 * FreeRTOS钩子函数的自定义实现。
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @brief vPortCleanUpTCB函数的实现
 *
 * 该函数在任务删除期间由vPortTCBPreDeleteHook调用。
 * 负责清理任何与TCB相关的资源。
 *
 * @param pxTCB 指向任务控制块的指针
 */
void vPortCleanUpTCB(void *pxTCB) {
  // 空实现 - 仅用于满足链接器要求
  // 如有需要可在此添加自定义清理代码
}