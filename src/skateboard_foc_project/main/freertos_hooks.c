/**
 * FreeRTOS hooks implementation
 *
 * This file provides custom implementations for FreeRTOS hooks
 * required by the current ESP-IDF configuration.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @brief Implementation of vPortCleanUpTCB
 *
 * This function is called by vPortTCBPreDeleteHook during task deletion.
 * It's responsible for cleaning up any TCB-related resources.
 *
 * @param pxTCB Pointer to the Task Control Block
 */
void vPortCleanUpTCB(void *pxTCB) {
  // Empty implementation - just satisfies the linker
  // Add custom cleanup code here if needed
}