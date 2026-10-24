#ifndef SERVO_BUS_H
#define SERVO_BUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "FreeRTOS.h"
#include "stm32f4xx_hal.h"
#include "task.h"

#include "servo_config.h"

extern uint8_t Servo_TxBuffer[SERVO_TX_BUFFER_SIZE];

/*
 *函数简介:初始化USART3舵机总线的软件状态和DMA缓冲区
 *参数说明:无
 *返回类型:无
 *备注:该函数不会启动DMA，也不会切换任何GPIO方向；URT-2负责TTL半双工转换，
 *     STM32一侧始终把USART3当作普通全双工UART使用
 */
void Servo_Bus_Init(void);

/*
 *函数简介:把ServoTask句柄绑定到USART3总线层
 *参数说明:taskHandle为ServoTask的FreeRTOS任务句柄
 *返回类型:无
 *备注:DMA发送完成、UART IDLE和错误中断通过该句柄向ServoTask发送通知；
 *     应在ServoTask开始运行后、启动接收DMA之前调用
 */
void Servo_Bus_BindTask(TaskHandle_t taskHandle);

/*
 *函数简介:启动USART3循环DMA接收和UART空闲线中断
 *参数说明:无
 *返回类型:HAL_StatusTypeDef，启动成功返回HAL_OK，否则返回HAL错误状态
 *备注:DMA只启动一次并保持Circular模式；IDLE中断只搬运新数据和通知任务，
 *     SCS反馈解析由Servo_Bus_ProcessReceivedData()在任务上下文完成
 */
HAL_StatusTypeDef Servo_Bus_StartReceive(void);

/*
 *函数简介:使用USART3 TX DMA非阻塞发送一条舵机数据包
 *参数说明:data指向待发送数据；length为有效字节数，最大为SERVO_TX_BUFFER_SIZE
 *返回类型:HAL_StatusTypeDef，成功启动DMA返回HAL_OK；忙或参数错误返回对应状态
 *备注:函数先将数据复制到稳定的Servo_TxBuffer，再调用HAL_UART_Transmit_DMA()；
 *     禁止使用阻塞式HAL_UART_Transmit()
 */
HAL_StatusTypeDef Servo_Bus_Send(const uint8_t *data, uint16_t length);

/*
 *函数简介:等待当前USART3 DMA发送完成通知
 *参数说明:timeout为FreeRTOS等待超时时间
 *返回类型:BaseType_t，发送完成返回pdTRUE，等待超时或串口错误返回pdFALSE
 *备注:等待期间当前ServoTask进入阻塞状态，CPU可以运行IMU和摄像头任务；
 *     HAL_UART_TxCpltCallback()会从中断唤醒任务，不进行忙等待
 */
BaseType_t Servo_Bus_WaitTxComplete(TickType_t timeout);

/*
 *函数简介:处理USART3 IDLE中断并搬运循环DMA中新收到的数据
 *参数说明:huart为发生中断的UART句柄，只接受huart3
 *返回类型:无
 *备注:本函数只清除IDLE标志、读取DMA位置、复制数据到软件环形缓冲区并通知任务；
 *     必须从USART3_IRQHandler()调用，禁止在这里解析SCS协议
 */
void Servo_Bus_IdleIRQHandler(UART_HandleTypeDef *huart);

/*
 *函数简介:在ServoTask中取出USART3接收数据并交给SCS反馈解析器
 *参数说明:无
 *返回类型:无
 *备注:该函数可以每20ms调用一次，也可在收到RX通知后调用；反馈包允许跨越多次
 *     UART IDLE事件，完整性最终由协议Length字段和Checksum共同判断
 */
void Servo_Bus_ProcessReceivedData(void);

/*
 *函数简介:在任务上下文恢复发生错误的USART3 DMA接收
 *参数说明:无
 *返回类型:无
 *备注:UART错误中断只设置恢复标志并通知任务；停止DMA、清理状态和重新启动接收
 *     都由本函数完成，避免在中断中执行复杂HAL流程
 */
void Servo_Bus_Service(void);

/*
 *函数简介:处理USART3 DMA发送完成HAL回调
 *参数说明:无
 *返回类型:无
 *备注:只能在HAL_UART_TxCpltCallback()中调用；清除发送忙状态并使用
 *     xTaskNotifyFromISR()通知ServoTask
 */
void Servo_Bus_TxCompleteCallbackFromISR(void);

/*
 *函数简介:处理USART3 UART错误HAL回调
 *参数说明:无
 *返回类型:无
 *备注:只能在HAL_UART_ErrorCallback()中调用；不在中断中重启DMA，只设置标志并
 *     唤醒ServoTask，由Servo_Bus_Service()执行恢复
 */
void Servo_Bus_ErrorCallbackFromISR(void);

/*
 *函数简介:读取USART3 DMA发送忙状态
 *参数说明:无
 *返回类型:uint8_t，返回1表示正在发送，返回0表示当前空闲
 *备注:用于ServoTask和调试代码判断能否开始下一帧，函数不会阻塞等待
 */
uint8_t Servo_Bus_IsTxBusy(void);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_BUS_H */
