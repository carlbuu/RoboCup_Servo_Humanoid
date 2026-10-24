#include "app_tasks.h"

#include <string.h>

#include "camera.h"
#include "camera_walk_link.h"
#include "debug_uart.h"
#include "imu.h"
#include "main.h"
#include "robot_control.h"
#include "robot_walk_config.h"
#include "servo.h"
#include "uart_dma.h"
#include "usart.h"

#define IMU_QUEUE_LENGTH     8
#define CAMERA_QUEUE_LENGTH  6

#if ROBOT_RX_DIAGNOSTICS_ENABLE
/* 仅缓存供诊断的副本；解析器仍接收全部数据。每秒最多打印64字节。 */
#define CAMERA_RAW_DIAGNOSTIC_BYTES 64U
#define CAMERA_RAW_LOG_LINE_BYTES   16U

typedef struct
{
    uint8_t data[CAMERA_RAW_DIAGNOSTIC_BYTES];
    uint16_t length;
    uint32_t parsedBytes;
    uint32_t clippedBytes;
} CameraRxDiagnostic_t;

static CameraRxDiagnostic_t s_cameraRxDiagnostic;

/**
 * @brief 保存进入Camera_Process前的接收副本，不在摄像头任务打印。
 * @param data 本次接收块，length 接收字节数。
 * @retval 无。
 */
static void CameraRxDiagnostic_Capture(const uint8_t *data, uint16_t length)
{
    uint16_t copyLength;
    taskENTER_CRITICAL();
    s_cameraRxDiagnostic.parsedBytes += length;
    copyLength = (uint16_t)(CAMERA_RAW_DIAGNOSTIC_BYTES -
                           s_cameraRxDiagnostic.length);
    if (copyLength > length) { copyLength = length; }
    if (copyLength != 0U)
    {
        memcpy(&s_cameraRxDiagnostic.data[s_cameraRxDiagnostic.length],
               data, copyLength);
        s_cameraRxDiagnostic.length += copyLength;
    }
    s_cameraRxDiagnostic.clippedBytes += (uint32_t)(length - copyLength);
    taskEXIT_CRITICAL();
}

/**
 * @brief 在低优先级调试任务输出接收/错误计数与有限的原始字节。
 * @param 无。
 * @retval 无。
 */
static void CameraRxDiagnostic_Print(void)
{
    static const char digits[] = "0123456789ABCDEF";
    static CameraRxDiagnostic_t snapshot;
    static CameraRxDiagnostic_t lastRaw;
    static TickType_t lastRawTick;
    uint8_t rawFresh;
    UART_DMA_Statistics_t statistics;
    static char hex[CAMERA_RAW_LOG_LINE_BYTES * 3U + 1U];
    uint16_t offset;
    uint16_t index;
    uint16_t count;
    uint16_t part;
    uint16_t parts;
    uint8_t dmaEnabled = 0U;
    const char *mode;

#if CAMERA_PROTOCOL_BINARY
    mode = "original_binary";
#else
    mode = "original_ascii";
#endif

    taskENTER_CRITICAL();
    snapshot = s_cameraRxDiagnostic;
    s_cameraRxDiagnostic.length = 0U;
    s_cameraRxDiagnostic.clippedBytes = 0U;
    taskEXIT_CRITICAL();
    rawFresh = (uint8_t)(snapshot.length != 0U);
    if (rawFresh != 0U)
    {
        lastRaw = snapshot;
        lastRawTick = xTaskGetTickCount();
    }
    statistics = UART_DMA_GetStatistics();
    if (huart2.hdmarx != NULL)
    {
        dmaEnabled = (uint8_t)((huart2.hdmarx->Instance->CR & DMA_SxCR_EN) != 0U);
    }

    Debug_Printf("[CAMRX] bytes=%lu parsed=%lu idle=%lu drop=%lu\r\n",
        (unsigned long)statistics.cameraRxBytes,
        (unsigned long)snapshot.parsedBytes,
        (unsigned long)statistics.cameraIdleEvents,
        (unsigned long)statistics.cameraDroppedPackets);
    Debug_Printf("[CAMERR] err=%lu last=%08lX restart=%lu fail=%lu\r\n",
        (unsigned long)statistics.cameraErrorEvents,
        (unsigned long)statistics.cameraLastError,
        (unsigned long)statistics.cameraRestartAttempts,
        (unsigned long)statistics.cameraRestartFailures);
    Debug_Printf("[CAMHW] mode=%s baud=%lu pclk=%lu brr=%04lX rx=%02lX dmar=%u dma_en=%u stack_words=%lu\r\n",
        mode, (unsigned long)huart2.Init.BaudRate,
        (unsigned long)HAL_RCC_GetPCLK1Freq(),
        (unsigned long)huart2.Instance->BRR,
        (unsigned long)huart2.RxState,
        (unsigned int)((huart2.Instance->CR3 & USART_CR3_DMAR) != 0U),
        (unsigned int)dmaEnabled,
        (unsigned long)uxTaskGetStackHighWaterMark(NULL));

    Debug_Printf("[FW] FOUR_MODES_RX built=%s %s\r\n", __DATE__, __TIME__);
    Debug_Printf("[CAMREG] halerr=%08lX sr=%08lX dma_state=%lu dma_err=%08lX\r\n",
        (unsigned long)huart2.ErrorCode,
        (unsigned long)huart2.Instance->SR,
        (unsigned long)(huart2.hdmarx != NULL ? huart2.hdmarx->State : 0U),
        (unsigned long)(huart2.hdmarx != NULL ? huart2.hdmarx->ErrorCode : 0U));
    Debug_Printf("[CAMIRQ] tc=%u en=%lu pending=%lu hisr=%08lX\r\n",
        (unsigned int)(huart2.hdmarx != NULL &&
            (huart2.hdmarx->Instance->CR & DMA_IT_TC) != 0U),
        (unsigned long)NVIC_GetEnableIRQ(DMA1_Stream5_IRQn),
        (unsigned long)NVIC_GetPendingIRQ(DMA1_Stream5_IRQn),
        (unsigned long)DMA1->HISR);

    parts = (uint16_t)((lastRaw.length + CAMERA_RAW_LOG_LINE_BYTES - 1U) /
                      CAMERA_RAW_LOG_LINE_BYTES);
    part = 0U;
    for (offset = 0U; offset < lastRaw.length; offset += count)
    {
        count = (uint16_t)(lastRaw.length - offset);
        if (count > CAMERA_RAW_LOG_LINE_BYTES) { count = CAMERA_RAW_LOG_LINE_BYTES; }
        for (index = 0U; index < count; ++index)
        {
            hex[index * 3U] = digits[lastRaw.data[offset + index] >> 4];
            hex[index * 3U + 1U] = digits[lastRaw.data[offset + index] & 0x0FU];
            hex[index * 3U + 2U] = ' ';
        }
        hex[count * 3U - 1U] = '\0';
        part++;
        Debug_Printf("[CAMRAW] fresh=%u age_ms=%lu part=%u/%u n=%u skipped=%lu hex=%s\r\n",
            (unsigned int)rawFresh,
            (unsigned long)((xTaskGetTickCount() - lastRawTick) * portTICK_PERIOD_MS),
            (unsigned int)part, (unsigned int)parts, (unsigned int)count,
            (unsigned long)lastRaw.clippedBytes, hex);
    }
}

#endif /* ROBOT_RX_DIAGNOSTICS_ENABLE */

TaskHandle_t g_imuTaskHandle;
TaskHandle_t g_cameraTaskHandle;
TaskHandle_t g_servoTaskHandle;
TaskHandle_t g_debugTaskHandle;

QueueHandle_t g_imuRxQueue;
QueueHandle_t g_cameraRxQueue;

/*
 *函数简介:创建机器人应用层使用的FreeRTOS通信对象
 *参数说明:无
 *返回类型:无
 *备注:创建IMU接收队列、摄像头接收队列和调试串口互斥量；必须在任务创建前调用，
 *     如果队列或互斥量创建失败，将进入Error_Handler()，避免系统带故障继续运行
 */
void App_RTOS_ObjectsInit(void)
{
    /*
     * xQueueCreate()：
     * 在 FreeRTOS 堆中创建固定长度队列。队列按值复制数据，因此 DMA
     * 后续覆盖循环缓冲区时，已入队的数据仍然安全。
     */
    g_imuRxQueue = xQueueCreate(IMU_QUEUE_LENGTH, sizeof(UART_DMA_Packet_t));
    g_cameraRxQueue = xQueueCreate(CAMERA_QUEUE_LENGTH,
                                   sizeof(UART_DMA_Packet_t));

    if ((g_imuRxQueue == NULL) || (g_cameraRxQueue == NULL))
    {
        Error_Handler();
    }

    /* 初始化视觉协议，再创建舵机最新目标队列和总线软件状态。 */
    Camera_Init();
#if ROBOT_RX_DIAGNOSTICS_ENABLE
    memset(&s_cameraRxDiagnostic, 0, sizeof(s_cameraRxDiagnostic));
#endif
    Servo_Init();
    Debug_UART_Init();
}

/*
 *函数简介:保存四个应用任务句柄并初始化UART DMA通信层
 *参数说明:imuTask为IMU任务句柄；cameraTask为摄像头任务句柄；
 *         servoTask为舵机任务句柄；debugTask为调试任务句柄
 *返回类型:无
 *备注:UART空闲中断需要通过任务句柄准确通知接收任务；任一句柄为空时进入
 *     Error_Handler()；该函数应在所有任务创建完成后、调度器启动前调用
 */
void App_RTOS_BindTaskHandles(TaskHandle_t imuTask,
                              TaskHandle_t cameraTask,
                              TaskHandle_t servoTask,
                              TaskHandle_t debugTask)
{
    if ((imuTask == NULL) || (cameraTask == NULL) ||
        (servoTask == NULL) || (debugTask == NULL))
    {
        Error_Handler();
    }

    g_imuTaskHandle = imuTask;
    g_cameraTaskHandle = cameraTask;
    g_servoTaskHandle = servoTask;
    g_debugTaskHandle = debugTask;

    UART_DMA_Init(g_imuRxQueue,
                  g_imuTaskHandle,
                  g_cameraRxQueue,
                  g_cameraTaskHandle);
}

/*
 *函数简介:接收并处理JY61陀螺仪的连续串口数据流
 *参数说明:argument为FreeRTOS任务入口参数，本框架中未使用
 *返回类型:无
 *备注:任务启动时开启USART1和USART2循环DMA；随后阻塞等待USART1 IDLE中断通知，
 *     从队列读取任意长度的数据块并交给JY61_StreamInput()；每个11字节原始帧
 *     独立完成帧头检查、校验和检查和数据换算，不在中断中执行协议解析
 */
void IMU_Task(void *argument)
{
    UART_DMA_Packet_t packet;

    (void)argument;

    /*
     * 调度器运行后再启动循环 DMA 和 IDLE 中断。这样传感器立即发送数据时，
     * xQueueSendFromISR() 和 xTaskNotifyFromISR() 已有完整的 RTOS 环境。
     */
    if (UART_DMA_Start() != HAL_OK)
    {
        Debug_Printf("[ERROR] USART1/USART2 DMA receive start failed.\r\n");
        Error_Handler();
    }

    for (;;)
    {
        /*
         * ulTaskNotifyTake(pdTRUE, portMAX_DELAY)：
         * - 没有通知时阻塞任务，不占用 CPU；
         * - pdTRUE 表示唤醒后清零通知计数；
         * - portMAX_DELAY 表示一直等待 ISR 通知。
         */
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* UART 错误恢复也在任务上下文完成，ISR 不执行复杂恢复流程。 */
        UART_DMA_Service(UART_DMA_SOURCE_IMU);

        while (xQueueReceive(g_imuRxQueue, &packet, 0) == pdPASS)
        {
            if (packet.source != UART_DMA_SOURCE_IMU)
            {
                continue;
            }

            /*
             * 队列数据块可能包含半帧、一帧或多帧，也可能从子帧中间开始。
             * 流式解析器会跨多次调用保存组帧状态；发现错误时直接放弃当前帧，
             * 从后续数据重新寻找0x55，不使用错误帧中的旧数据。
             */
            JY61_StreamInput(packet.data, packet.length);
        }
    }
}

/*
 *函数简介:接收并处理摄像头模块通过USART2发送的数据
 *参数说明:argument为FreeRTOS任务入口参数，本框架中未使用
 *返回类型:无
 *备注:任务等待USART2 IDLE通知，最多10ms后醒来检查DMA错误恢复，
 *     限制复制长度不超过256字节，再调用Camera_Process()；协议解析只在任务中
 *     执行，不在串口中断中执行
 */
void Camera_Task(void *argument)
{
    UART_DMA_Packet_t packet;
    uint8_t packetCount;

    (void)argument;

    for (;;)
    {
        /* 通知只负责唤醒，实际数据通过队列传递。 */
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(CAMERA_SERVICE_PERIOD_MS));

        UART_DMA_Service(UART_DMA_SOURCE_CAMERA);

        for (packetCount = 0U; packetCount < CAMERA_QUEUE_LENGTH; packetCount++)
        {
            uint16_t copyLength;

            if (xQueueReceive(g_cameraRxQueue, &packet, 0) != pdPASS)
            {
                break;
            }
            if (packet.source != UART_DMA_SOURCE_CAMERA)
            {
                continue;
            }

            copyLength = packet.length;
            if (copyLength > CAMERA_RX_BUFFER_SIZE)
            {
                copyLength = CAMERA_RX_BUFFER_SIZE;
            }

            memcpy(Camera_RxBuffer, packet.data, copyLength);

            /* 摄像头协议处理只在任务中执行。 */
#if ROBOT_RX_DIAGNOSTICS_ENABLE
            CameraRxDiagnostic_Capture(Camera_RxBuffer, copyLength);
#endif
            Camera_ProcessTimed(Camera_RxBuffer, copyLength, packet.receivedTick);
        }
        CameraWalk_SendStatus();
    }
}

/*
 *函数简介:周期输出机器人主控的运行状态和调试信息
 *参数说明:argument为FreeRTOS任务入口参数，本框架中未使用
 *返回类型:无
 *备注:每1000毫秒读取一次模式、视觉统计和舵机发送状态，并通过
 *     UART4输出；该任务优先级较低，调试输出不会抢占关键控制任务
 */
void Debug_Task(void *argument)
{
    CameraStatus_t cameraStatus;
    CameraStatistics_t cameraStatistics;
    RobotControlStatus_t robotStatus;
    (void)argument;
    Debug_Printf("[FW] FOUR_MODES mode=%s built=%s %s\r\n",
        RobotControl_GetModeName(), __DATE__, __TIME__);
    for (;;)
    {
        cameraStatus = Camera_GetStatus();
        cameraStatistics = Camera_GetStatistics();
        robotStatus = RobotControl_GetStatus();
        Debug_Printf("[ROBOT] mode=%s state=%s motion=%u frame=%u servo_busy=%u\r\n",
            RobotControl_GetModeName(), RobotControl_GetStateName(robotStatus.state),
            (unsigned int)robotStatus.motion, (unsigned int)robotStatus.frameIndex + 1U,
            Servo_IsTxBusy());
        Debug_Printf("[CAM] state=%s shape=%s(%u) seq=%lu ok=%lu cmd=%lu crc=%lu fmt=%lu\r\n",
            Camera_GetStateName(cameraStatus.state), Camera_GetShapeName(cameraStatus.shape),
            (unsigned int)cameraStatus.shape, (unsigned long)cameraStatus.recognitionSequence,
            (unsigned long)cameraStatistics.validFrames,
            (unsigned long)cameraStatistics.commandFrames,
            (unsigned long)cameraStatistics.checksumErrors,
            (unsigned long)cameraStatistics.formatErrors);
#if ROBOT_RUN_MODE != ROBOT_MODE_ACTION_DEBUG
        Debug_Printf("[VISION] phase=%s cycles=%u/%u queued=%u events=%lu/%lu dropped=%lu\r\n",
            RobotControl_GetWalkPhaseName(robotStatus.walkPhase),
            (unsigned int)robotStatus.completedCycles, (unsigned int)robotStatus.targetCycles,
            (unsigned int)robotStatus.pendingShapes,
            (unsigned long)robotStatus.eventsCompleted, (unsigned long)robotStatus.eventsStarted,
            (unsigned long)robotStatus.eventsDropped);
#endif
#if ROBOT_RX_DIAGNOSTICS_ENABLE
        CameraRxDiagnostic_Print();
#endif
        /* 低优先级日志任务每秒输出一次，让接收与动作任务及时运行。 */
        vTaskDelay(pdMS_TO_TICKS(1000U));
    }
}
