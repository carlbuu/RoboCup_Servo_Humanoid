#ifndef APP_TASKS_H
#define APP_TASKS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

/* 串口接收队列及任务句柄，供 UART DMA 中断通知层使用。 */
extern QueueHandle_t g_imuRxQueue;
extern QueueHandle_t g_cameraRxQueue;

extern TaskHandle_t g_imuTaskHandle;
extern TaskHandle_t g_cameraTaskHandle;
extern TaskHandle_t g_servoTaskHandle;
extern TaskHandle_t g_debugTaskHandle;

/*
 *函数简介:创建机器人应用层使用的FreeRTOS通信对象
 *参数说明:无
 *返回类型:无
 *备注:在MX_FREERTOS_Init()内、四个任务创建之前调用，负责创建接收队列和
 *     调试互斥量；创建失败时进入Error_Handler()
 */
void App_RTOS_ObjectsInit(void);

/*
 *函数简介:绑定应用任务句柄并初始化UART DMA软件通道
 *参数说明:imuTask、cameraTask、servoTask、debugTask分别为四个应用任务句柄
 *返回类型:无
 *备注:在CubeMX创建完四个任务后调用，任务句柄用于UART中断的任务通知
 */
void App_RTOS_BindTaskHandles(TaskHandle_t imuTask,
                              TaskHandle_t cameraTask,
                              TaskHandle_t servoTask,
                              TaskHandle_t debugTask);

/*
 *函数简介:陀螺仪数据接收与25字节组帧任务
 *参数说明:argument为FreeRTOS任务入口参数，当前未使用
 *返回类型:无
 *备注:永久等待USART1接收通知，完整帧交给JY61_Process()处理
 */
void IMU_Task(void *argument);

/*
 *函数简介:摄像头串口数据处理任务
 *参数说明:argument为FreeRTOS任务入口参数，当前未使用
 *返回类型:无
 *备注:永久等待USART2接收通知，数据交给Camera_Process()处理
 */
void Camera_Task(void *argument);

/*
 *函数简介:系统状态低优先级调试输出任务
 *参数说明:argument为FreeRTOS任务入口参数，当前未使用
 *返回类型:无
 *备注:每1000毫秒通过UART4输出一次接收统计、IMU数据和舵机状态
 */
void Debug_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* APP_TASKS_H */
