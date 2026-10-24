#include "servo_bus.h"

#include <stddef.h>
#include <string.h>

#include "servo_protocol.h"
#include "usart.h"

uint8_t Servo_TxBuffer[SERVO_TX_BUFFER_SIZE];

static uint8_t s_rxDmaBuffer[SERVO_RX_DMA_BUFFER_SIZE];
static uint8_t s_rxRingBuffer[SERVO_RX_RING_BUFFER_SIZE];

static volatile uint16_t s_rxRingWrite;
static volatile uint16_t s_rxRingRead;
static uint16_t s_rxDmaLastPosition;

static volatile uint8_t s_txBusy;
static volatile uint8_t s_restartRequested;
static TaskHandle_t s_servoTaskHandle;

static uint16_t Servo_Bus_NextRingIndex(uint16_t index);
static void Servo_Bus_CopyRangeFromISR(const uint8_t *data,
                                       uint16_t length,
                                       BaseType_t *dataCopied);
static void Servo_Bus_NotifyFromISR(uint32_t notifyBits,
                                    BaseType_t *higherPriorityTaskWoken);

/*
 *函数简介:计算USART3接收软件环形缓冲区的下一个下标
 *参数说明:index为当前缓冲区下标
 *返回类型:uint16_t，返回加1并完成末尾回绕后的下标
 *备注:使用明确的边界判断，不要求缓冲区大小必须是2的整数次幂
 */
static uint16_t Servo_Bus_NextRingIndex(uint16_t index)
{
    index++;
    if (index >= SERVO_RX_RING_BUFFER_SIZE)
    {
        index = 0;
    }

    return index;
}

/*
 *函数简介:初始化USART3舵机总线的软件状态和DMA缓冲区
 *参数说明:无
 *返回类型:无
 *备注:只清空软件状态，不启动DMA，不执行半双工方向切换
 */
void Servo_Bus_Init(void)
{
    memset(Servo_TxBuffer, 0, sizeof(Servo_TxBuffer));
    memset(s_rxDmaBuffer, 0, sizeof(s_rxDmaBuffer));
    memset(s_rxRingBuffer, 0, sizeof(s_rxRingBuffer));

    s_rxRingWrite = 0;
    s_rxRingRead = 0;
    s_rxDmaLastPosition = 0;
    s_txBusy = 0;
    s_restartRequested = 0;
    s_servoTaskHandle = NULL;
}

/*
 *函数简介:把ServoTask句柄绑定到USART3总线层
 *参数说明:taskHandle为ServoTask任务句柄
 *返回类型:无
 *备注:绑定完成后USART3相关中断才能通过任务通知唤醒正确任务
 */
void Servo_Bus_BindTask(TaskHandle_t taskHandle)
{
    taskENTER_CRITICAL();
    s_servoTaskHandle = taskHandle;
    taskEXIT_CRITICAL();
}

/*
 *函数简介:启动USART3循环DMA接收和UART IDLE中断
 *参数说明:无
 *返回类型:HAL_StatusTypeDef，成功返回HAL_OK
 *备注:关闭DMA半传输和传输完成中断，只以UART IDLE作为不定长反馈数据块边界
 */
HAL_StatusTypeDef Servo_Bus_StartReceive(void)
{
    HAL_StatusTypeDef status;

    if (huart3.hdmarx == NULL)
    {
        return HAL_ERROR;
    }

    s_rxDmaLastPosition = 0;
    s_rxRingWrite = 0;
    s_rxRingRead = 0;
    s_restartRequested = 0;

    status = HAL_UART_Receive_DMA(&huart3,
                                  s_rxDmaBuffer,
                                  SERVO_RX_DMA_BUFFER_SIZE);
    if (status != HAL_OK)
    {
        return status;
    }

    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_TC);

    __HAL_UART_CLEAR_IDLEFLAG(&huart3);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    return HAL_OK;
}

/*
 *函数简介:使用USART3 TX DMA非阻塞发送一条舵机数据包
 *参数说明:data指向数据包；length为有效长度
 *返回类型:HAL_StatusTypeDef，成功启动返回HAL_OK，忙时返回HAL_BUSY
 *备注:DMA使用全局稳定缓冲区，调用者返回后可以立即修改自己的原始数据
 */
HAL_StatusTypeDef Servo_Bus_Send(const uint8_t *data, uint16_t length)
{
    HAL_StatusTypeDef status;

    if ((data == NULL) || (length == 0) ||
        (length > SERVO_TX_BUFFER_SIZE))
    {
        return HAL_ERROR;
    }

    taskENTER_CRITICAL();
    if (s_txBusy != 0)
    {
        taskEXIT_CRITICAL();
        return HAL_BUSY;
    }

    memcpy(Servo_TxBuffer, data, length);
    s_txBusy = 1;
    taskEXIT_CRITICAL();

    status = HAL_UART_Transmit_DMA(&huart3, Servo_TxBuffer, length);
    if (status != HAL_OK)
    {
        taskENTER_CRITICAL();
        s_txBusy = 0;
        taskEXIT_CRITICAL();
    }

    return status;
}

/*
 *函数简介:等待当前USART3 DMA发送完成通知
 *参数说明:timeout为最大等待tick数
 *返回类型:BaseType_t，发送完成返回pdTRUE，超时或错误返回pdFALSE
 *备注:xTaskNotifyWait()使ServoTask阻塞，不会占用CPU循环查询DMA寄存器
 */
BaseType_t Servo_Bus_WaitTxComplete(TickType_t timeout)
{
    TickType_t startTick;
    TickType_t elapsed;
    TickType_t remaining;
    uint32_t notifyValue;

    if (s_txBusy == 0)
    {
        return pdTRUE;
    }

    startTick = xTaskGetTickCount();
    remaining = timeout;

    while (s_txBusy != 0)
    {
        notifyValue = 0;
        if (xTaskNotifyWait(0,
                            SERVO_NOTIFY_TX_COMPLETE |
                            SERVO_NOTIFY_RX_DATA |
                            SERVO_NOTIFY_UART_ERROR,
                            &notifyValue,
                            remaining) != pdTRUE)
        {
            taskENTER_CRITICAL();
            s_restartRequested = 1;
            taskEXIT_CRITICAL();
            Servo_Bus_Service();
            return pdFALSE;
        }

        if ((notifyValue & SERVO_NOTIFY_UART_ERROR) != 0)
        {
            Servo_Bus_Service();
            return pdFALSE;
        }

        elapsed = xTaskGetTickCount() - startTick;
        if (elapsed >= timeout)
        {
            if (s_txBusy != 0)
            {
                taskENTER_CRITICAL();
                s_restartRequested = 1;
                taskEXIT_CRITICAL();
                Servo_Bus_Service();
                return pdFALSE;
            }

            return pdTRUE;
        }

        remaining = timeout - elapsed;
    }

    return pdTRUE;
}

/*
 *函数简介:从中断上下文向ServoTask发送一个或多个事件位
 *参数说明:notifyBits为通知位；higherPriorityTaskWoken用于记录是否需要立刻切换任务
 *返回类型:无
 *备注:任务尚未绑定时直接返回，防止调度器启动前使用FreeRTOS FromISR接口
 */
static void Servo_Bus_NotifyFromISR(uint32_t notifyBits,
                                    BaseType_t *higherPriorityTaskWoken)
{
    if (s_servoTaskHandle == NULL)
    {
        return;
    }

    (void)xTaskNotifyFromISR(s_servoTaskHandle,
                            notifyBits,
                            eSetBits,
                            higherPriorityTaskWoken);
}

/*
 *函数简介:把循环DMA的一段连续新数据复制到软件环形缓冲区
 *参数说明:data指向DMA新数据；length为字节数；dataCopied返回是否至少复制了1字节
 *返回类型:无
 *备注:软件环形缓冲区满时丢弃后续新字节，不覆盖任务尚未读取的旧数据
 */
static void Servo_Bus_CopyRangeFromISR(const uint8_t *data,
                                       uint16_t length,
                                       BaseType_t *dataCopied)
{
    uint16_t index;

    for (index = 0; index < length; index++)
    {
        uint16_t nextWrite = Servo_Bus_NextRingIndex(s_rxRingWrite);

        if (nextWrite == s_rxRingRead)
        {
            break;
        }

        s_rxRingBuffer[s_rxRingWrite] = data[index];
        s_rxRingWrite = nextWrite;
        *dataCopied = pdTRUE;
    }
}

/*
 *函数简介:处理USART3 IDLE中断并搬运循环DMA中新收到的数据
 *参数说明:huart必须指向huart3
 *返回类型:无
 *备注:ISR只搬运和通知；反馈协议解析留在ServoTask执行
 */
void Servo_Bus_IdleIRQHandler(UART_HandleTypeDef *huart)
{
    uint16_t dmaRemaining;
    uint16_t currentPosition;
    BaseType_t dataCopied = pdFALSE;
    BaseType_t higherPriorityTaskWoken = pdFALSE;

    if ((huart != &huart3) ||
        (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) == RESET) ||
        (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE) == RESET))
    {
        return;
    }

    __HAL_UART_CLEAR_IDLEFLAG(huart);

    dmaRemaining = (uint16_t)__HAL_DMA_GET_COUNTER(huart->hdmarx);
    currentPosition = (uint16_t)(SERVO_RX_DMA_BUFFER_SIZE - dmaRemaining);

    if (currentPosition > s_rxDmaLastPosition)
    {
        Servo_Bus_CopyRangeFromISR(&s_rxDmaBuffer[s_rxDmaLastPosition],
                                   (uint16_t)(currentPosition -
                                              s_rxDmaLastPosition),
                                   &dataCopied);
    }
    else if (currentPosition < s_rxDmaLastPosition)
    {
        Servo_Bus_CopyRangeFromISR(&s_rxDmaBuffer[s_rxDmaLastPosition],
                                   (uint16_t)(SERVO_RX_DMA_BUFFER_SIZE -
                                              s_rxDmaLastPosition),
                                   &dataCopied);

        if (currentPosition > 0)
        {
            Servo_Bus_CopyRangeFromISR(s_rxDmaBuffer,
                                       currentPosition,
                                       &dataCopied);
        }
    }

    s_rxDmaLastPosition =
        (currentPosition == SERVO_RX_DMA_BUFFER_SIZE) ? 0 : currentPosition;

    if (dataCopied == pdTRUE)
    {
        Servo_Bus_NotifyFromISR(SERVO_NOTIFY_RX_DATA,
                                &higherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

/*
 *函数简介:在ServoTask中取出USART3接收数据并交给SCS反馈解析器
 *参数说明:无
 *返回类型:无
 *备注:每次使用小块临时数组降低任务栈占用，直到软件环形缓冲区被读空
 */
void Servo_Bus_ProcessReceivedData(void)
{
    uint8_t dataBlock[32];
    uint16_t blockLength;

    do
    {
        blockLength = 0;

        while ((s_rxRingRead != s_rxRingWrite) &&
               (blockLength < sizeof(dataBlock)))
        {
            dataBlock[blockLength++] = s_rxRingBuffer[s_rxRingRead];
            s_rxRingRead = Servo_Bus_NextRingIndex(s_rxRingRead);
        }

        if (blockLength > 0)
        {
            Servo_Protocol_InputFeedback(dataBlock, blockLength);
        }
    }
    while (blockLength > 0);
}

/*
 *函数简介:在任务上下文恢复发生错误的USART3 DMA接收
 *参数说明:无
 *返回类型:无
 *备注:只有错误中断设置了恢复请求时才停止并重新启动USART3 DMA
 */
void Servo_Bus_Service(void)
{
    uint8_t restartRequested;

    taskENTER_CRITICAL();
    restartRequested = s_restartRequested;
    s_restartRequested = 0;
    taskEXIT_CRITICAL();

    if (restartRequested != 0)
    {
        __HAL_UART_DISABLE_IT(&huart3, UART_IT_IDLE);
        (void)HAL_UART_DMAStop(&huart3);

        taskENTER_CRITICAL();
        s_txBusy = 0;
        taskEXIT_CRITICAL();

        (void)Servo_Bus_StartReceive();
    }
}

/*
 *函数简介:处理USART3 DMA发送完成HAL回调
 *参数说明:无
 *返回类型:无
 *备注:ISR只清除忙标志并通知ServoTask，不在中断里开始下一帧发送
 */
void Servo_Bus_TxCompleteCallbackFromISR(void)
{
    BaseType_t higherPriorityTaskWoken = pdFALSE;

    s_txBusy = 0;
    Servo_Bus_NotifyFromISR(SERVO_NOTIFY_TX_COMPLETE,
                            &higherPriorityTaskWoken);
    portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

/*
 *函数简介:处理USART3 UART错误HAL回调
 *参数说明:无
 *返回类型:无
 *备注:ISR设置重启标志并唤醒任务，复杂恢复流程由Servo_Bus_Service()完成
 */
void Servo_Bus_ErrorCallbackFromISR(void)
{
    BaseType_t higherPriorityTaskWoken = pdFALSE;

    s_txBusy = 0;
    s_restartRequested = 1;
    Servo_Bus_NotifyFromISR(SERVO_NOTIFY_UART_ERROR |
                            SERVO_NOTIFY_TX_COMPLETE,
                            &higherPriorityTaskWoken);
    portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

/*
 *函数简介:读取USART3 DMA发送忙状态
 *参数说明:无
 *返回类型:uint8_t，1表示忙，0表示空闲
 *备注:单字节变量在Cortex-M4上可原子读取，不需要关闭中断
 */
uint8_t Servo_Bus_IsTxBusy(void)
{
    return s_txBusy;
}
