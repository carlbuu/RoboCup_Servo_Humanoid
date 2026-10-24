#include "robot_walk_config.h"

#include "camera_walk_link.h"
#include "camera.h"

#include <string.h>
#include <stdio.h>
#include "robot_control.h"
#include "usart.h"

#include "FreeRTOS.h"
#include "task.h"

#define CAMERA_ORIGINAL_SHAPE_QUEUE_LENGTH 16U
#define CAMERA_ORIGINAL_SEQUENCE_MAX       2147483647UL
#define CAMERA_ORIGINAL_BINARY_MAX         6U

uint8_t Camera_RxBuffer[CAMERA_RX_BUFFER_SIZE];

static CameraWalkRequest_t s_shapes[CAMERA_ORIGINAL_SHAPE_QUEUE_LENGTH];
static CameraWalkRequest_t s_motion;
static uint8_t s_shapeHead;
static uint8_t s_shapeTail;
static uint8_t s_shapeCount;
static uint8_t s_hasMotion;
static uint32_t s_frameFirstTick;
#if ROBOT_WALK_LATE_ACTIVE
static CameraWalkRequest_t s_samples[CAMERA_WALK_SAMPLE_QUEUE_LENGTH];
static uint8_t s_sampleRead;
static uint8_t s_sampleCount;
#endif
static uint32_t s_localSequence;
static volatile CameraStatus_t s_status;
static volatile CameraStatistics_t s_statistics;

#if CAMERA_PROTOCOL_BINARY
static uint8_t s_binaryFrame[CAMERA_ORIGINAL_BINARY_MAX];
static uint8_t s_binaryLength;
#else
static char s_asciiFrame[CAMERA_FRAME_MAX];
static uint16_t s_asciiLength;
static uint8_t s_discardLine;
#endif

static void Camera_CountFormatError(void)
{
    taskENTER_CRITICAL();
    s_statistics.formatErrors++;
    taskEXIT_CRITICAL();
}

static uint8_t Camera_IsValidCommand(uint8_t command, uint8_t argument)
{
    if (command == CAMERA_WALK_BACK)
    {
#ifdef ROBOT_BACKWARD_MOTION
        return (uint8_t)(argument == 1U);
#else
        return 0U; /* 未配置后退步态，不接受不可执行的BACK。 */
#endif
    }
    if (command > CAMERA_WALK_SHIFT_RIGHT)
    {
        return 0U;
    }
    if ((command == CAMERA_WALK_HOLD) || (command == CAMERA_WALK_BLOCKED))
    {
        return (uint8_t)(argument == 0U);
    }
    if ((command == CAMERA_WALK_LEFT) || (command == CAMERA_WALK_RIGHT) ||
        (command == CAMERA_WALK_SHIFT_LEFT) || (command == CAMERA_WALK_SHIFT_RIGHT))
    {
        return 1U; /* 保留0..255协议字段，但转向不采用角度值。 */
    }
    return (uint8_t)(argument != 0U);
}

/* 原视觉没有序号；本地序号仅用于日志，不代表视觉端事件唯一身份。 */
static void Camera_Receive(CameraWalkRequest_t *request, uint32_t now)
{
    taskENTER_CRITICAL();
    if (s_localSequence >= CAMERA_ORIGINAL_SEQUENCE_MAX)
    {
        s_localSequence = 1U;
    }
    else
    {
        s_localSequence++;
    }
    request->sequence = s_localSequence;
    request->receivedTick = now;
    request->firstReceivedTick = s_frameFirstTick;
    s_statistics.validFrames++;
    s_status.receivedTick = now;
    if (request->type == CAMERA_WALK_SHAPE)
    {
        s_statistics.shapeFrames++;
        if (s_shapeCount < CAMERA_ORIGINAL_SHAPE_QUEUE_LENGTH)
        {
            s_shapes[s_shapeTail] = *request;
            s_shapeTail = (uint8_t)((s_shapeTail + 1U) % CAMERA_ORIGINAL_SHAPE_QUEUE_LENGTH);
            s_shapeCount++;
            s_status.shape = (CameraShape_t)request->shape;
            s_status.recognitionSequence = request->sequence;
            s_status.state = CAMERA_STATE_SUCCESS;
        }
        else
        {
            s_statistics.requestQueueFull++;
            s_statistics.overflowErrors++;
        }
    }
    else
    {
        s_statistics.commandFrames++;
#if ROBOT_WALK_LATE_ACTIVE
        if (request->command == CAMERA_WALK_HOLD || request->command == CAMERA_WALK_BLOCKED)
        {
            /* 停止独立锁存；BLOCKED优先，普通建议不能覆盖停止。 */
            if (s_hasMotion == 0U || s_motion.command != CAMERA_WALK_BLOCKED)
            {
                s_motion = *request;
            }
            s_hasMotion = 1U;
            s_sampleCount = 0U;
        }
        else
        {
            uint8_t slot;
            if (s_sampleCount == CAMERA_WALK_SAMPLE_QUEUE_LENGTH)
            {
                s_sampleRead = (uint8_t)((s_sampleRead + 1U) % CAMERA_WALK_SAMPLE_QUEUE_LENGTH);
                s_sampleCount--;
                s_statistics.commandCoalesced++;
                s_statistics.requestQueueFull++;
            }
            slot = (uint8_t)((s_sampleRead + s_sampleCount) % CAMERA_WALK_SAMPLE_QUEUE_LENGTH);
            s_samples[slot] = *request;
            s_sampleCount++;
        }
#else
        if (s_hasMotion != 0U)
        {
            s_statistics.commandCoalesced++;
        }
        /* BLOCKED尚未被RobotTask消费时，后续FWD/TURN不得将它吞掉。 */
        if ((s_hasMotion == 0U) || (s_motion.command != CAMERA_WALK_BLOCKED))
        {
            s_motion = *request;
        }
        s_hasMotion = 1U;
#endif
    }
    taskEXIT_CRITICAL();
}

uint8_t CameraWalk_TakeRequest(CameraWalkRequest_t *request)
{
    uint8_t available;
    if (request == NULL)
    {
        return 0U;
    }
    available = 0U;
    taskENTER_CRITICAL();
#if ROBOT_WALK_LATE_ACTIVE
    if (s_hasMotion != 0U)
    {
        *request = s_motion;
        s_hasMotion = 0U;
        available = 1U;
    }
    else
#endif
    if (s_shapeCount != 0U)
    {
        *request = s_shapes[s_shapeHead];
        s_shapeHead = (uint8_t)((s_shapeHead + 1U) % CAMERA_ORIGINAL_SHAPE_QUEUE_LENGTH);
        s_shapeCount--;
        available = 1U;
    }
#if ROBOT_WALK_LATE_ACTIVE
    else if (s_sampleCount != 0U)
    {
        *request = s_samples[s_sampleRead];
        s_sampleRead = (uint8_t)((s_sampleRead + 1U) % CAMERA_WALK_SAMPLE_QUEUE_LENGTH);
        s_sampleCount--;
        available = 1U;
    }
#else
    else if (s_hasMotion != 0U)
    {
        *request = s_motion;
        s_hasMotion = 0U;
        available = 1U;
    }
#endif
    taskEXIT_CRITICAL();
    return available;
}

void CameraWalk_ClearMotion(void)
{
    taskENTER_CRITICAL();
#if ROBOT_WALK_LATE_ACTIVE
    s_sampleCount = 0U;
#endif
    /* 停稳清旧方向时，不能吞掉刚被高优先级CameraTask接收的暂停/阻塞。 */
    if (s_hasMotion == 0U ||
        (s_motion.command != CAMERA_WALK_HOLD && s_motion.command != CAMERA_WALK_BLOCKED))
    {
        s_hasMotion = 0U;
    }
    taskEXIT_CRITICAL();
}

#if CAMERA_PROTOCOL_BINARY

static uint8_t Camera_HandleBinary(uint32_t now)
{
    CameraWalkRequest_t request;
    uint8_t expectedLength;
    uint8_t checksum;
    expectedLength = (s_binaryFrame[1] == 0x55U) ? 5U : 6U;
    if (s_binaryFrame[expectedLength - 1U] != 0x0AU)
    {
        Camera_CountFormatError();
        return 0U;
    }
    memset(&request, 0, sizeof(request));
    if (s_binaryFrame[1] == 0x55U)
    {
        checksum = (uint8_t)(s_binaryFrame[2] ^ 0xA5U);
        if (s_binaryFrame[3] != checksum)
        {
            taskENTER_CRITICAL();
            s_statistics.checksumErrors++;
            taskEXIT_CRITICAL();
            return 0U;
        }
        if ((s_binaryFrame[2] < 1U) || (s_binaryFrame[2] > 6U))
        {
            Camera_CountFormatError();
            return 0U;
        }
        request.type = CAMERA_WALK_SHAPE;
        request.shape = s_binaryFrame[2];
    }
    else
    {
        checksum = (uint8_t)(s_binaryFrame[2] ^ s_binaryFrame[3]);
        if (s_binaryFrame[4] != checksum)
        {
            taskENTER_CRITICAL();
            s_statistics.checksumErrors++;
            taskEXIT_CRITICAL();
            return 0U;
        }
        if (Camera_IsValidCommand(s_binaryFrame[2], s_binaryFrame[3]) == 0U)
        {
            Camera_CountFormatError();
            return 0U;
        }
        request.type = CAMERA_WALK_COMMAND;
        request.command = s_binaryFrame[2];
        request.arg = s_binaryFrame[3];
    }
    Camera_Receive(&request, now);
    return 1U;
}

/* 错误帧滑动找AA55/AA57，保留末尾AA；可立即处理已完整的5字节候选。 */
static void Camera_ResyncBinary(void)
{
    uint8_t index;
    uint8_t remain;
    for (index = 1U; index + 1U < s_binaryLength; index++)
    {
        if ((s_binaryFrame[index] == 0xAAU) &&
            ((s_binaryFrame[index + 1U] == 0x55U) ||
             (s_binaryFrame[index + 1U] == 0x57U)))
        {
            remain = (uint8_t)(s_binaryLength - index);
            memmove(s_binaryFrame, &s_binaryFrame[index], remain);
            s_binaryLength = remain;
            return;
        }
    }
    remain = (uint8_t)(s_binaryFrame[s_binaryLength - 1U] == 0xAAU);
    s_binaryLength = remain;
    if (remain != 0U)
    {
        s_binaryFrame[0] = 0xAAU;
    }
}

static void Camera_ProcessByte(uint8_t value, uint32_t now)
{
    uint8_t expectedLength;
    if (s_binaryLength == 0U)
    {
        s_frameFirstTick = now;
        if (value == 0xAAU)
        {
            s_binaryFrame[0] = value;
            s_binaryLength = 1U;
        }
    }
    else if (s_binaryLength == 1U)
    {
        if ((value == 0x55U) || (value == 0x57U))
        {
            s_binaryFrame[1] = value;
            s_binaryLength = 2U;
        }
        else if (value != 0xAAU)
        {
            s_binaryLength = 0U;
        }
    }
    else
    {
        if (s_binaryLength >= sizeof(s_binaryFrame))
        {
            s_binaryLength = 0U;
            Camera_CountFormatError();
            return;
        }
        s_binaryFrame[s_binaryLength++] = value;
    }
    while (s_binaryLength >= 2U)
    {
        expectedLength = (s_binaryFrame[1] == 0x55U) ? 5U : 6U;
        if (s_binaryLength < expectedLength)
        {
            break;
        }
        if (Camera_HandleBinary(now) != 0U)
        {
            s_binaryLength = 0U;
        }
        else
        {
            Camera_ResyncBinary();
        }
    }
}

#else /* 原ASCII模式；按行同步，没有校验能力，不解码任何AA55/AA57帧。 */

static uint8_t Camera_ParseArgument(const char *text, uint8_t *argument)
{
    uint16_t value;
    uint8_t length;
    value = 0U;
    length = 0U;
    if ((text == NULL) || (*text == '\0'))
    {
        return 0U;
    }
    while (*text != '\0')
    {
        if ((*text < '0') || (*text > '9') || (++length > 3U))
        {
            return 0U;
        }
        value = (uint16_t)(value * 10U + (uint16_t)(*text - '0'));
        if (value > 255U)
        {
            return 0U;
        }
        text++;
    }
    *argument = (uint8_t)value;
    return 1U;
}

static void Camera_HandleAscii(uint32_t now)
{
    static const char * const commandNames[] =
    { "HOLD", "FWD", "TURNL", "TURNR", "BLOCKED", "BACK", "SHIFTL", "SHIFTR" };
    static const char * const shapeNames[] =
    { "", "CIRCLE", "STAR", "SQUARE", "DIAMOND", "CROSS", "TRIANGLE" };
    CameraWalkRequest_t request;
    char *fields[3];
    uint16_t index;
    uint8_t count;
    uint8_t argument;
    uint8_t command;
    count = 1U;
    fields[0] = s_asciiFrame;
    for (index = 0U; index < s_asciiLength; index++)
    {
        if (s_asciiFrame[index] == ',')
        {
            if (count >= 3U)
            {
                Camera_CountFormatError();
                return;
            }
            s_asciiFrame[index] = '\0';
            fields[count++] = &s_asciiFrame[index + 1U];
        }
    }
    if ((count != 3U) || (Camera_ParseArgument(fields[2], &argument) == 0U))
    {
        Camera_CountFormatError();
        return;
    }
    memset(&request, 0, sizeof(request));
    if (strcmp(fields[0], "CMD") == 0)
    {
        for (command = 0U; command <= CAMERA_WALK_SHIFT_RIGHT; command++)
        {
            if (strcmp(fields[1], commandNames[command]) == 0)
            {
                break;
            }
        }
        if (Camera_IsValidCommand(command, argument) == 0U)
        {
            Camera_CountFormatError();
            return;
        }
        request.type = CAMERA_WALK_COMMAND;
        request.command = command;
        request.arg = argument;
    }
    else if ((strcmp(fields[0], "SHAPE") == 0) && (argument >= 1U) &&
             (argument <= 6U) && (strcmp(fields[1], shapeNames[argument]) == 0))
    {
        request.type = CAMERA_WALK_SHAPE;
        request.shape = argument;
    }
    else
    {
        Camera_CountFormatError();
        return;
    }
    Camera_Receive(&request, now);
}

static void Camera_ProcessByte(uint8_t value, uint32_t now)
{
    if (s_asciiLength == 0U) { s_frameFirstTick = now; }
    if (value == '\n')
    {
        if ((s_discardLine == 0U) && (s_asciiLength != 0U))
        {
            if (s_asciiFrame[s_asciiLength - 1U] == '\r')
            {
                s_asciiLength--;
            }
            s_asciiFrame[s_asciiLength] = '\0';
            Camera_HandleAscii(now);
        }
        s_asciiLength = 0U;
        s_discardLine = 0U;
        return;
    }
    if (s_discardLine != 0U)
    {
        return;
    }
    if (((value < 0x20U) && (value != '\r')) || (value > 0x7EU))
    {
        Camera_CountFormatError();
        s_asciiLength = 0U;
        s_discardLine = 1U;
        return;
    }
    if (s_asciiLength >= sizeof(s_asciiFrame) - 1U)
    {
        taskENTER_CRITICAL();
        s_statistics.overflowErrors++;
        taskEXIT_CRITICAL();
        /* 溢出整行丢弃到LF，不能把其尾部误解释为另一条运动命令。 */
        s_asciiLength = 0U;
        s_discardLine = 1U;
        return;
    }
    s_asciiFrame[s_asciiLength++] = (char)value;
}

#endif /* CAMERA_PROTOCOL_BINARY */

void Camera_ProcessTimed(uint8_t *data, uint16_t len, uint32_t receivedTick)
{
    uint16_t index;
    uint32_t now;
    if (data == NULL)
    {
        return;
    }
    now = receivedTick;
    for (index = 0U; index < len; index++)
    {
        Camera_ProcessByte(data[index], now);
    }
}

void Camera_Process(uint8_t *data, uint16_t len)
{
    Camera_ProcessTimed(data, len, (uint32_t)xTaskGetTickCount());
}

void Camera_Init(void)
{
    CameraStatus_t status;
    CameraStatistics_t statistics;
    memset(&status, 0, sizeof(status));
    memset(&statistics, 0, sizeof(statistics));
    memset(Camera_RxBuffer, 0, sizeof(Camera_RxBuffer));
    memset(s_shapes, 0, sizeof(s_shapes));
    memset(&s_motion, 0, sizeof(s_motion));
    s_status = status;
    s_statistics = statistics;
    s_status.state = CAMERA_STATE_RECOGNIZING;
    s_shapeHead = 0U;
    s_shapeTail = 0U;
    s_shapeCount = 0U;
    s_hasMotion = 0U;
    s_frameFirstTick = 0U;
#if ROBOT_WALK_LATE_ACTIVE
    s_sampleRead = s_sampleCount = 0U;
#endif
    s_localSequence = 0U;
#if CAMERA_PROTOCOL_BINARY
    memset(s_binaryFrame, 0, sizeof(s_binaryFrame));
    s_binaryLength = 0U;
#else
    memset(s_asciiFrame, 0, sizeof(s_asciiFrame));
    s_asciiLength = 0U;
    s_discardLine = 0U;
#endif
}

CameraStatus_t Camera_GetStatus(void)
{
    CameraStatus_t snapshot;
    taskENTER_CRITICAL();
    snapshot = s_status;
    taskEXIT_CRITICAL();
    return snapshot;
}

CameraShape_t Camera_GetShape(void)
{
    return Camera_GetStatus().shape;
}

CameraStatistics_t Camera_GetStatistics(void)
{
    CameraStatistics_t snapshot;
    taskENTER_CRITICAL();
    snapshot = s_statistics;
    taskEXIT_CRITICAL();
    return snapshot;
}

const char *Camera_GetStateName(CameraState_t state)
{
    static const char * const names[] =
    { "recognizing", "success" };
    return ((unsigned int)state < 2U) ? names[state] : "unknown";
}

const char *Camera_GetShapeName(CameraShape_t shape)
{
    static const char * const names[] =
    { "none", "circle", "star", "square", "diamond", "cross", "triangle" };
    return ((unsigned int)shape < 7U) ? names[shape] : "unknown";
}



/**
 * @brief 100ms软件状态心跳；由Camera_Task调用，非阻塞中断发送。
 * @note TX缓冲区静态保存，UART忙时不修改；发送失败后下次周期重试。
 *       字段：阶段/动作/周期/图形开始/图形完成/待处理图形/后退能力/模式/运动序号/完成序号。
 */
void CameraWalk_SendStatus(void)
{
    static uint8_t buffer[160];
    static TickType_t lastTick;
    RobotControlStatus_t status;
    TickType_t now = xTaskGetTickCount();
    int length;
    uint16_t i;
    uint8_t checksum = 0U;
    uint8_t back = 0U;
    if (huart2.gState != HAL_UART_STATE_READY ||
        (TickType_t)(now - lastTick) < pdMS_TO_TICKS(100U)) { return; }
#ifdef ROBOT_BACKWARD_MOTION
    back = 1U;
#endif
    status = RobotControl_GetStatus();
    length = snprintf((char *)buffer, sizeof(buffer),
        "STATE,%u,%u,%u,%lu,%lu,%u,%u,%u,%lu,%lu",
        (unsigned int)status.walkPhase, (unsigned int)status.motion,
        (unsigned int)status.completedCycles, (unsigned long)status.eventsStarted,
        (unsigned long)status.eventsCompleted, (unsigned int)status.pendingShapes,
        (unsigned int)back, (unsigned int)ROBOT_RUN_MODE,
        (unsigned long)status.activeCommandSequence,
        (unsigned long)status.lastCompletedCommandSequence);
    if (length <= 0 || (unsigned int)length + 4U >= sizeof(buffer)) { return; }
    for (i = 0U; i < (uint16_t)length; ++i) { checksum ^= buffer[i]; }
    (void)snprintf((char *)&buffer[length], sizeof(buffer) - (unsigned int)length,
                   "*%02X\n", (unsigned int)checksum);
    if (HAL_UART_Transmit_IT(&huart2, buffer, (uint16_t)(length + 4)) == HAL_OK)
    {
        lastTick = now;
    }
}
