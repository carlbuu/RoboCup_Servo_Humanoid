#ifndef UART_DMA_H
#define UART_DMA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "stm32f4xx_hal.h"
#include "task.h"

#define UART_DMA_PACKET_DATA_SIZE  256

typedef enum
{
    UART_DMA_SOURCE_IMU = 0,
    UART_DMA_SOURCE_CAMERA
} UART_DMA_Source_t;

/* 队列按值复制该结构体，数据生命周期与 DMA 原始缓冲区无关。 */
typedef struct
{
    UART_DMA_Source_t source;
    uint16_t length;
    uint32_t receivedTick; /* IDLE入队时间，不使用后续任务解析时间。 */
    uint8_t data[UART_DMA_PACKET_DATA_SIZE];
} UART_DMA_Packet_t;

typedef struct
{
    uint32_t imuRxBytes;
    uint32_t cameraRxBytes;
    uint32_t imuDroppedPackets;
    uint32_t cameraDroppedPackets;
    uint32_t uartErrors;
    uint32_t cameraIdleEvents;
    uint32_t cameraErrorEvents;
    uint32_t cameraLastError;
    uint32_t cameraRestartAttempts;
    uint32_t cameraRestartFailures;
} UART_DMA_Statistics_t;

/*
 *函数简介:建立USART1/USART2与队列、接收任务之间的软件映射
 *参数说明:imuQueue和cameraQueue为两个数据队列；imuTask和cameraTask为接收任务句柄
 *返回类型:无
 *备注:只初始化软件对象，不启动DMA；调用前必须完成外设和FreeRTOS对象创建
 */
void UART_DMA_Init(QueueHandle_t imuQueue,
                   TaskHandle_t imuTask,
                   QueueHandle_t cameraQueue,
                   TaskHandle_t cameraTask);

/*
 *函数简介:启动USART1和USART2循环DMA接收及IDLE中断
 *参数说明:无
 *返回类型:HAL_StatusTypeDef，全部成功返回HAL_OK，否则返回HAL_ERROR
 *备注:应在FreeRTOS调度器运行后调用一次
 */
HAL_StatusTypeDef UART_DMA_Start(void);

/*
 *函数简介:处理UART空闲中断并把DMA新增数据送入队列
 *参数说明:huart为触发IDLE事件的UART句柄
 *返回类型:无
 *备注:仅供USART1/USART2中断入口调用，ISR中不执行协议解析
 */
void UART_DMA_IdleIRQHandler(UART_HandleTypeDef *huart);

/*
 *函数简介:在任务上下文中恢复发生错误的UART DMA接收通道
 *参数说明:source指定IMU或摄像头数据源
 *返回类型:无
 *备注:ISR只设置恢复标志，停止和重启DMA由本函数完成
 */
void UART_DMA_Service(UART_DMA_Source_t source);

/*
 *函数简介:取得UART DMA接收字节数、丢包数和错误数快照
 *参数说明:无
 *返回类型:UART_DMA_Statistics_t，返回当前统计结构体副本
 *备注:内部使用临界区保证多个统计字段读取一致
 */
UART_DMA_Statistics_t UART_DMA_GetStatistics(void);

#ifdef __cplusplus
}
#endif

#endif /* UART_DMA_H */
