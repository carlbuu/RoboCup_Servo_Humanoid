#include "uart_dma.h"

#include <string.h>

#include "servo_bus.h"
#include "usart.h"

/*
 * USART1/2 使用循环 DMA。缓冲区尺寸故意不等于协议公开处理缓冲区，
 * 降低 DMA 整圈回绕后“新旧位置相同”造成歧义的概率。
 */
#define USART1_DMA_BUFFER_SIZE  128
#define USART2_DMA_BUFFER_SIZE  512

typedef struct
{
    UART_HandleTypeDef *huart;
    uint8_t *dmaBuffer;
    uint16_t dmaBufferSize;
    volatile uint16_t lastPosition;
    volatile uint8_t restartRequested;
    QueueHandle_t queue;
    TaskHandle_t task;
    UART_DMA_Source_t source;
    UART_DMA_Packet_t isrPacket;
} UART_DMA_Channel_t;

static uint8_t s_usart1DmaBuffer[USART1_DMA_BUFFER_SIZE];
static uint8_t s_usart2DmaBuffer[USART2_DMA_BUFFER_SIZE];

static UART_DMA_Channel_t s_imuChannel;
static UART_DMA_Channel_t s_cameraChannel;
static volatile UART_DMA_Statistics_t s_statistics;

static HAL_StatusTypeDef UART_DMA_StartChannel(UART_DMA_Channel_t *channel);
static void UART_DMA_QueueRangeFromISR(UART_DMA_Channel_t *channel,
                                       const uint8_t *data,
                                       uint16_t length,
                                       BaseType_t *dataQueued,
                                       BaseType_t *higherPriorityTaskWoken);
static UART_DMA_Channel_t *UART_DMA_GetChannel(UART_DMA_Source_t source);
static void UART_DMA_RequestRecoveryFromISR(UART_DMA_Channel_t *channel,
                                             BaseType_t *higherPriorityTaskWoken);

/*
 *函数简介:初始化USART1和USART2循环DMA接收通道的软件管理对象
 *参数说明:imuQueue为IMU数据队列；imuTask为IMU接收任务句柄；
 *         cameraQueue为摄像头数据队列；cameraTask为摄像头接收任务句柄
 *返回类型:无
 *备注:该函数只保存外设、缓冲区、队列和任务之间的对应关系，不会立即启动DMA；
 *     调用前必须完成CubeMX串口、DMA初始化以及FreeRTOS队列和任务创建
 */
void UART_DMA_Init(QueueHandle_t imuQueue,
                   TaskHandle_t imuTask,
                   QueueHandle_t cameraQueue,
                   TaskHandle_t cameraTask)
{
    memset((void *)&s_statistics, 0, sizeof(s_statistics));
    memset(&s_imuChannel, 0, sizeof(s_imuChannel));
    memset(&s_cameraChannel, 0, sizeof(s_cameraChannel));

    s_imuChannel.huart = &huart1;
    s_imuChannel.dmaBuffer = s_usart1DmaBuffer;
    s_imuChannel.dmaBufferSize = sizeof(s_usart1DmaBuffer);
    s_imuChannel.queue = imuQueue;
    s_imuChannel.task = imuTask;
    s_imuChannel.source = UART_DMA_SOURCE_IMU;

    s_cameraChannel.huart = &huart2;
    s_cameraChannel.dmaBuffer = s_usart2DmaBuffer;
    s_cameraChannel.dmaBufferSize = sizeof(s_usart2DmaBuffer);
    s_cameraChannel.queue = cameraQueue;
    s_cameraChannel.task = cameraTask;
    s_cameraChannel.source = UART_DMA_SOURCE_CAMERA;
}

/*
 *函数简介:启动USART1和USART2的循环DMA接收及UART IDLE中断
 *参数说明:无
 *返回类型:HAL_StatusTypeDef，两个通道均启动成功返回HAL_OK，否则返回HAL_ERROR
 *备注:由IMU_Task在调度器开始运行后调用一次，避免调度器启动前发生中断并使用
 *     FreeRTOS FromISR接口；任一通道失败都会向调用者报告失败
 */
HAL_StatusTypeDef UART_DMA_Start(void)
{
    HAL_StatusTypeDef imuStatus;
    HAL_StatusTypeDef cameraStatus;

    imuStatus = UART_DMA_StartChannel(&s_imuChannel);
    cameraStatus = UART_DMA_StartChannel(&s_cameraChannel);

    return ((imuStatus == HAL_OK) && (cameraStatus == HAL_OK))
         ? HAL_OK
         : HAL_ERROR;
}

/*
 *函数简介:启动指定UART DMA接收通道
 *参数说明:channel指向待启动的UART_DMA_Channel_t通道管理结构体
 *返回类型:HAL_StatusTypeDef，参数和资源有效且DMA启动成功返回HAL_OK，否则返回错误状态
 *备注:清零DMA读取位置和恢复标志，调用HAL_UART_Receive_DMA()后关闭DMA半传输中断，保留
 *     完成中断以支持HAL错误中止回调；UART IDLE仍作为数据块边界；本函数在任务上下文中执行
 */
static HAL_StatusTypeDef UART_DMA_StartChannel(UART_DMA_Channel_t *channel)
{
    HAL_StatusTypeDef status;

    if ((channel == NULL) || (channel->huart == NULL) ||
        (channel->huart->hdmarx == NULL) || (channel->queue == NULL) ||
        (channel->task == NULL))
    {
        return HAL_ERROR;
    }

    channel->lastPosition = 0;
    channel->restartRequested = 0;
    __HAL_UART_CLEAR_IDLEFLAG(channel->huart);

    /*
     * DMA 已由 CubeMX 配置为 DMA_CIRCULAR。
     * 这里只启动一次；IDLE 后不停止 DMA。
     */
    status = HAL_UART_Receive_DMA(channel->huart,
                                 channel->dmaBuffer,
                                 channel->dmaBufferSize);
    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * 不使用半传输/传输完成作为帧边界，仅关闭 HT 以降低中断负担。
     * 数据边界统一由 UART IDLE 中断判断。TC 必须保持开启，否则 HAL_DMA_Abort_IT()
     * 停止DMA后无法调用错误回调，接收会停在READY且不请求恢复。
     */
    __HAL_DMA_DISABLE_IT(channel->huart->hdmarx, DMA_IT_HT);
    __HAL_DMA_ENABLE_IT(channel->huart->hdmarx, DMA_IT_TC);

    __HAL_UART_CLEAR_IDLEFLAG(channel->huart);
    __HAL_UART_ENABLE_IT(channel->huart, UART_IT_IDLE);

    return HAL_OK;
}

/*
 *函数简介:处理USART1或USART2的UART空闲线中断并搬运新接收数据
 *参数说明:huart指向发生IDLE中断的HAL UART句柄，仅支持huart1和huart2
 *返回类型:无
 *备注:本函数在USART中断入口调用，只清除IDLE标志、读取DMA写入位置、复制新数据
 *     到FreeRTOS队列并通知对应任务；不会执行JY61或摄像头协议解析
 */
void UART_DMA_IdleIRQHandler(UART_HandleTypeDef *huart)
{
    UART_DMA_Channel_t *channel;
    uint16_t dmaRemaining;
    uint16_t currentPosition;
    BaseType_t dataQueued = pdFALSE;
    BaseType_t higherPriorityTaskWoken = pdFALSE;

    if ((huart == NULL) ||
        (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) == RESET) ||
        (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE) == RESET))
    {
        return;
    }

    /* STM32F4 通过读 SR/DR 的硬件顺序清除 IDLE 标志。 */
    __HAL_UART_CLEAR_IDLEFLAG(huart);

    if (huart == s_imuChannel.huart)
    {
        channel = &s_imuChannel;
    }
    else if (huart == s_cameraChannel.huart)
    {
        channel = &s_cameraChannel;
        s_statistics.cameraIdleEvents++;
    }
    else
    {
        return;
    }

    dmaRemaining = (uint16_t)__HAL_DMA_GET_COUNTER(huart->hdmarx);
    currentPosition = (uint16_t)(channel->dmaBufferSize - dmaRemaining);

    if (currentPosition > channel->lastPosition)
    {
        UART_DMA_QueueRangeFromISR(
            channel,
            &channel->dmaBuffer[channel->lastPosition],
            (uint16_t)(currentPosition - channel->lastPosition),
            &dataQueued,
            &higherPriorityTaskWoken);
    }
    else if (currentPosition < channel->lastPosition)
    {
        /* DMA 已回绕，分别搬运尾段和头段。 */
        UART_DMA_QueueRangeFromISR(
            channel,
            &channel->dmaBuffer[channel->lastPosition],
            (uint16_t)(channel->dmaBufferSize - channel->lastPosition),
            &dataQueued,
            &higherPriorityTaskWoken);

        if (currentPosition > 0)
        {
            UART_DMA_QueueRangeFromISR(channel,
                                       channel->dmaBuffer,
                                       currentPosition,
                                       &dataQueued,
                                       &higherPriorityTaskWoken);
        }
    }

    channel->lastPosition = (currentPosition == channel->dmaBufferSize)
                          ? 0
                          : currentPosition;

    if ((dataQueued == pdTRUE) && (channel->task != NULL))
    {
        /*
         * xTaskNotifyFromISR()：从中断给任务发送轻量通知。
         * eIncrement 把通知值当作计数信号量使用；复杂解析留给任务。
         */
        (void)xTaskNotifyFromISR(channel->task,
                                0,
                                eIncrement,
                                &higherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

/*
 *函数简介:把循环DMA缓冲区中的一段连续数据分块发送到FreeRTOS队列
 *参数说明:channel为数据来源通道；data为连续数据首地址；length为有效字节数；
 *         dataQueued用于返回是否至少成功入队一包；higherPriorityTaskWoken用于
 *         向FreeRTOS返回本次入队是否唤醒了更高优先级任务
 *返回类型:无
 *备注:该静态函数只能在中断上下文中调用，每个队列包最多保存256字节；队列满时
 *     不阻塞，丢包计数会累加，协议解析仍由接收任务完成
 */
static void UART_DMA_QueueRangeFromISR(UART_DMA_Channel_t *channel,
                                       const uint8_t *data,
                                       uint16_t length,
                                       BaseType_t *dataQueued,
                                       BaseType_t *higherPriorityTaskWoken)
{
    uint16_t offset = 0;

    while (offset < length)
    {
        uint16_t chunkLength = (uint16_t)(length - offset);

        if (chunkLength > UART_DMA_PACKET_DATA_SIZE)
        {
            chunkLength = UART_DMA_PACKET_DATA_SIZE;
        }

        channel->isrPacket.source = channel->source;
        channel->isrPacket.length = chunkLength;
        channel->isrPacket.receivedTick = (uint32_t)xTaskGetTickCountFromISR();
        memcpy(channel->isrPacket.data, &data[offset], chunkLength);

        /*
         * xQueueSendFromISR()：在 ISR 中把一项数据复制进队列。
         * 该 API 不阻塞，队列满时立即返回 errQUEUE_FULL。
         */
        if (xQueueSendFromISR(channel->queue,
                             &channel->isrPacket,
                             higherPriorityTaskWoken) == pdPASS)
        {
            *dataQueued = pdTRUE;

            if (channel->source == UART_DMA_SOURCE_IMU)
            {
                s_statistics.imuRxBytes += chunkLength;
            }
            else
            {
                s_statistics.cameraRxBytes += chunkLength;
            }
        }
        else if (channel->source == UART_DMA_SOURCE_IMU)
        {
            s_statistics.imuDroppedPackets++;
        }
        else
        {
            s_statistics.cameraDroppedPackets++;
        }

        offset = (uint16_t)(offset + chunkLength);
    }
}

/*
 *函数简介:根据数据源类型取得对应的UART DMA通道管理对象
 *参数说明:source为UART_DMA_SOURCE_IMU或UART_DMA_SOURCE_CAMERA
 *返回类型:UART_DMA_Channel_t指针，类型有效时返回对应通道，无效时返回NULL
 *备注:返回的是模块内部静态对象地址，调用者只能在uart_dma.c内部使用，
 *     不应释放该指针或修改其通道映射关系
 */
static UART_DMA_Channel_t *UART_DMA_GetChannel(UART_DMA_Source_t source)
{
    if (source == UART_DMA_SOURCE_IMU)
    {
        return &s_imuChannel;
    }
    if (source == UART_DMA_SOURCE_CAMERA)
    {
        return &s_cameraChannel;
    }
    return NULL;
}

/*
 *函数简介:在UART错误中断中标记指定接收通道需要重新启动
 *参数说明:channel为发生错误的接收通道；higherPriorityTaskWoken用于返回任务
 *         通知是否唤醒了更高优先级任务
 *返回类型:无
 *备注:ISR中只设置restartRequested并通知任务，不停止或重启DMA；实际恢复操作
 *     由UART_DMA_Service()在任务上下文中完成
 */
static void UART_DMA_RequestRecoveryFromISR(
    UART_DMA_Channel_t *channel,
    BaseType_t *higherPriorityTaskWoken)
{
    channel->restartRequested = 1;

    if (channel->task != NULL)
    {
        (void)xTaskNotifyFromISR(channel->task,
                                0,
                                eIncrement,
                                higherPriorityTaskWoken);
    }
}

/*
 *函数简介:在任务上下文中执行指定UART接收通道的错误恢复
 *参数说明:source为需要检查的接收数据源，可选择IMU或摄像头通道
 *返回类型:无
 *备注:读取并清除中断设置的恢复标志；需要恢复时先关闭IDLE中断、停止旧DMA，
 *     再重新启动循环DMA；无恢复请求时立即返回
 */
void UART_DMA_Service(UART_DMA_Source_t source)
{
    UART_DMA_Channel_t *channel = UART_DMA_GetChannel(source);
    uint8_t restartRequested;
    HAL_StatusTypeDef restartStatus;

    if (channel == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    restartRequested = channel->restartRequested;
    channel->restartRequested = 0;
    taskEXIT_CRITICAL();

    if (restartRequested != 0)
    {
        if (source == UART_DMA_SOURCE_CAMERA)
        {
            taskENTER_CRITICAL();
            s_statistics.cameraRestartAttempts++;
            taskEXIT_CRITICAL();
        }
        __HAL_UART_DISABLE_IT(channel->huart, UART_IT_IDLE);
        (void)HAL_UART_DMAStop(channel->huart);
        restartStatus = UART_DMA_StartChannel(channel);
        if ((source == UART_DMA_SOURCE_CAMERA) && (restartStatus != HAL_OK))
        {
            taskENTER_CRITICAL();
            s_statistics.cameraRestartFailures++;
            taskEXIT_CRITICAL();
        }
    }
}

/*
 *函数简介:处理HAL报告的UART通信错误
 *参数说明:huart指向发生错误的HAL UART句柄
 *返回类型:无
 *备注:USART1或USART2错误只记录统计信息、设置恢复标志并通知接收任务；
 *     USART3错误交给舵机总线层通知ServoTask恢复；该HAL回调运行在中断上下文中
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    BaseType_t higherPriorityTaskWoken = pdFALSE;

    s_statistics.uartErrors++;

    if (huart == s_imuChannel.huart)
    {
        UART_DMA_RequestRecoveryFromISR(&s_imuChannel,
                                        &higherPriorityTaskWoken);
    }
    else if (huart == s_cameraChannel.huart)
    {
        s_statistics.cameraErrorEvents++;
        s_statistics.cameraLastError = HAL_UART_GetError(huart);
        UART_DMA_RequestRecoveryFromISR(&s_cameraChannel,
                                        &higherPriorityTaskWoken);
    }
    else if (huart == &huart3)
    {
        Servo_Bus_ErrorCallbackFromISR();
    }

    portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

/*
 *函数简介:处理HAL UART DMA发送完成事件
 *参数说明:huart指向完成发送的HAL UART句柄
 *返回类型:无
 *备注:当前仅处理USART3舵机总线，调用Servo_Bus_TxCompleteCallbackFromISR()
 *     清除发送忙标志并通知ServoTask；该HAL回调运行在中断上下文中
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3)
    {
        Servo_Bus_TxCompleteCallbackFromISR();
    }
}

/*
 *函数简介:读取UART DMA通信模块的运行统计快照
 *参数说明:无
 *返回类型:UART_DMA_Statistics_t，包含IMU和摄像头接收字节数、丢包数及UART错误数
 *备注:使用FreeRTOS临界区保证多个统计字段来自同一时刻；返回的是结构体副本，
 *     调用者修改返回值不会改变模块内部累计数据
 */
UART_DMA_Statistics_t UART_DMA_GetStatistics(void)
{
    UART_DMA_Statistics_t snapshot;

    taskENTER_CRITICAL();
    snapshot.imuRxBytes = s_statistics.imuRxBytes;
    snapshot.cameraRxBytes = s_statistics.cameraRxBytes;
    snapshot.imuDroppedPackets = s_statistics.imuDroppedPackets;
    snapshot.cameraDroppedPackets = s_statistics.cameraDroppedPackets;
    snapshot.uartErrors = s_statistics.uartErrors;
    snapshot.cameraIdleEvents = s_statistics.cameraIdleEvents;
    snapshot.cameraErrorEvents = s_statistics.cameraErrorEvents;
    snapshot.cameraLastError = s_statistics.cameraLastError;
    snapshot.cameraRestartAttempts = s_statistics.cameraRestartAttempts;
    snapshot.cameraRestartFailures = s_statistics.cameraRestartFailures;
    taskEXIT_CRITICAL();

    return snapshot;
}
