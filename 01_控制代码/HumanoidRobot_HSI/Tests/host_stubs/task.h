#ifndef BINARY_TEST_TASK_H
#define BINARY_TEST_TASK_H
#include "FreeRTOS.h"
TickType_t xTaskGetTickCount(void);
void vTaskDelay(TickType_t ticks);
void vTaskDelayUntil(TickType_t *last, TickType_t ticks);
#endif
