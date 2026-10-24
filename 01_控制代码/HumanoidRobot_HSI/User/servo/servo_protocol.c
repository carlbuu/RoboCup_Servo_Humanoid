#include "servo_protocol.h"

#include <stddef.h>//提供NULL
#include <string.h>//提供memcpy(),memset()

static uint8_t s_feedbackFrame[SERVO_FEEDBACK_FRAME_MAX];//缓存完整帧
static uint16_t s_feedbackIndex;                         //表示：当前收到几个字节。
static uint16_t s_feedbackExpectedLength;				 //等待的完整长度

static uint8_t Servo_Protocol_CalculateChecksum(const uint8_t *packet,
                                                uint16_t firstIndex,
                                                uint16_t lastIndex);
static uint8_t Servo_Protocol_CheckFeedback(const uint8_t *packet,
                                            uint16_t packetLength);
static void Servo_Protocol_DeliverFeedback(const uint8_t *packet,
                                           uint16_t packetLength);

/*
 *函数简介:计算SCS协议校验和
 *参数说明:packet指向完整或正在生成的数据包；firstIndex为参与累加的第一个下标；
 *         lastIndex为参与累加的最后一个下标
 *返回类型:uint8_t，返回所有指定字节累加后按位取反的低8位结果
 *备注:SCS校验公式为~(ID+Length+Instruction/Status+Parameter...)，帧头不参与计算
 */
static uint8_t Servo_Protocol_CalculateChecksum(const uint8_t *packet,
                                                uint16_t firstIndex,
                                                uint16_t lastIndex)
{
    uint16_t index;
    uint8_t sum = 0;

    for (index = firstIndex; index <= lastIndex; index++)
    {
        sum = (uint8_t)(sum + packet[index]);
    }

    return (uint8_t)(~sum);
}

/*
 *函数简介:生成一条SCS普通写指令数据包
 *参数说明:id为目标舵机ID；address为写入寄存器地址；data指向待写数据；
 *         dataLength为数据字节数；output为输出缓冲区；outputSize为输出缓冲区容量
 *返回类型:uint16_t，返回生成的数据包总长度，参数错误或缓冲区不足时返回0
 *备注:Length等于地址1字节、数据区、Instruction和Checksum的总字节数
 */
uint16_t Servo_Protocol_BuildWrite(uint8_t id,
                                   uint8_t address,
                                   const uint8_t *data,
                                   uint8_t dataLength,
                                   uint8_t *output,
                                   uint16_t outputSize)
{
    uint16_t packetLength;

    if ((id == SERVO_PACKET_HEADER) || (data == NULL) ||
        (dataLength == 0) || (output == NULL))
    {
        return 0;
    }

    packetLength = (uint16_t)dataLength + 7;
    if (packetLength > outputSize)
    {
        return 0;
    }

    output[0] = SERVO_PACKET_HEADER;
    output[1] = SERVO_PACKET_HEADER;
    output[2] = id;
    output[3] = (uint8_t)(dataLength + 3);
    output[4] = SERVO_INSTRUCTION_WRITE;
    output[5] = address;
    memcpy(&output[6], data, dataLength);
    output[packetLength - 1] =
        Servo_Protocol_CalculateChecksum(output, 2, packetLength - 2);

    return packetLength;
}

/*
 *函数简介:生成本次参与运动舵机的同步控制数据包
 *参数说明:targets为目标数组；count为舵机数量；output为输出缓冲区；
 *         outputSize为输出缓冲区容量
 *返回类型:uint16_t，成功返回数据包长度，参数错误或缓冲区不足返回0
 *备注:从0x29开始连续写入7字节，依次为加速度、位置、两个保留字节和速度
 */
uint16_t Servo_Protocol_BuildSyncMotion(const ServoSyncTarget_t *targets,
                                        uint8_t count,
                                        uint8_t *output,
                                        uint16_t outputSize)
{
    uint16_t packetLength;
    uint16_t outputIndex;
    uint8_t servoIndex;

    if ((targets == NULL) || (count == 0) ||
        (count > SERVO_COUNT) || (output == NULL))
    {
        return 0;
    }

    packetLength = (uint16_t)(8 +
                   (count * (SERVO_SYNC_MOTION_DATA_LENGTH + 1)));
    if (packetLength > outputSize)
    {
        return 0;
    }

    output[0] = SERVO_PACKET_HEADER;
    output[1] = SERVO_PACKET_HEADER;
    output[2] = SERVO_BROADCAST_ID;
    output[3] = (uint8_t)(packetLength - 4);
    output[4] = SERVO_INSTRUCTION_SYNC_WRITE;
    output[5] = SERVO_REGISTER_ACCELERATION;
    output[6] = SERVO_SYNC_MOTION_DATA_LENGTH;

    outputIndex = 7;
    for (servoIndex = 0; servoIndex < count; servoIndex++)
    {
        output[outputIndex++] = targets[servoIndex].id;
        output[outputIndex++] = targets[servoIndex].acceleration;
        output[outputIndex++] =
            (uint8_t)(targets[servoIndex].position & 0x00FF);
        output[outputIndex++] =
            (uint8_t)(targets[servoIndex].position >> 8);

        /* 0x2C和0x2D在本阶段不使用，同步写时固定填0。 */
        output[outputIndex++] = 0x00;
        output[outputIndex++] = 0x00;

        output[outputIndex++] =
            (uint8_t)(targets[servoIndex].speed & 0x00FF);
        output[outputIndex++] =
            (uint8_t)(targets[servoIndex].speed >> 8);
    }

    output[packetLength - 1] =
        Servo_Protocol_CalculateChecksum(output, 2, packetLength - 2);

    return packetLength;
}

/*
 *函数简介:生成多个舵机同步写入一个单字节寄存器的数据包
 *参数说明:address为寄存器地址；ids与values为一一对应的ID和值数组；count为数量；
 *         output为输出缓冲区；outputSize为缓冲区容量
 *返回类型:uint16_t，成功时返回数据包长度，参数错误或容量不足时返回0
 *备注:每个舵机在参数区占用ID和1字节数据，主要用于批量开启或关闭扭矩
 */
uint16_t Servo_Protocol_BuildSyncByte(uint8_t address,
                                      const uint8_t *ids,
                                      const uint8_t *values,
                                      uint8_t count,
                                      uint8_t *output,
                                      uint16_t outputSize)
{
    uint16_t packetLength;
    uint16_t outputIndex;
    uint8_t index;

    if ((ids == NULL) || (values == NULL) || (count == 0) ||
        (output == NULL))
    {
        return 0;
    }

    packetLength = (uint16_t)(8 + (count * 2));
    if (packetLength > outputSize)
    {
        return 0;
    }

    output[0] = SERVO_PACKET_HEADER;
    output[1] = SERVO_PACKET_HEADER;
    output[2] = SERVO_BROADCAST_ID;
    output[3] = (uint8_t)(packetLength - 4);
    output[4] = SERVO_INSTRUCTION_SYNC_WRITE;//同步写指令
    output[5] = address;
    output[6] = 1;

    outputIndex = 7;
    for (index = 0; index < count; index++)
    {
        output[outputIndex++] = ids[index];
        output[outputIndex++] = values[index];
    }

    output[packetLength - 1] =
        Servo_Protocol_CalculateChecksum(output, 2, packetLength - 2);

    return packetLength;
}

/*
 *函数简介:检查一条SCS反馈数据包的帧头、长度和校验和
 *参数说明:packet指向完整反馈包；packetLength为完整数据包字节数
 *返回类型:uint8_t，返回1表示反馈包正确，返回0表示需要丢弃
 *备注:合法反馈包的ID到Checksum全部字节相加后，低8位应等于0xFF
 */
static uint8_t Servo_Protocol_CheckFeedback(const uint8_t *packet,
                                            uint16_t packetLength)
{
    uint16_t index;
    uint8_t sum = 0;

    if ((packet == NULL) || (packetLength < 6) ||
        (packet[0] != SERVO_PACKET_HEADER) ||
        (packet[1] != SERVO_PACKET_HEADER) ||
        ((uint16_t)packet[3] + 4 != packetLength))
    {
        return 0;
    }

    for (index = 2; index < packetLength; index++)
    {
        sum = (uint8_t)(sum + packet[index]);
    }

    return (sum == 0xFF) ? 1 : 0;
}

/*
 *函数简介:把已校验的SCS反馈数据整理为容易扩展的结构体并交给用户回调
 *参数说明:packet指向完整反馈包；packetLength为完整反馈包长度
 *返回类型:无
 *备注:广播指令和超出参数缓存容量的数据不会交付；当前默认回调为空
 */
static void Servo_Protocol_DeliverFeedback(const uint8_t *packet,
                                           uint16_t packetLength)
{
    ServoFeedbackPacket_t feedback;
    uint8_t parameterLength;

    (void)packetLength;

    if (packet[2] == SERVO_BROADCAST_ID)
    {
        return;
    }

    parameterLength = (uint8_t)(packet[3] - 2);
    if (parameterLength > SERVO_FEEDBACK_PARAMETER_MAX)
    {
        return;
    }

    feedback.id = packet[2];
    feedback.error = packet[4];
    feedback.parameterLength = parameterLength;
    memset(feedback.parameters, 0, sizeof(feedback.parameters));

    if (parameterLength > 0)
    {
        memcpy(feedback.parameters, &packet[5], parameterLength);
    }

    Servo_FeedbackReceived(&feedback);
}

/*
 *函数简介:向SCS反馈流解析器输入一段任意长度的接收数据
 *参数说明:data指向任务取得的USART3原始数据；length为本次有效字节数
 *返回类型:无
 *备注:错误帧直接丢弃并重新等待0xFF 0xFF；解析过程只在ServoTask上下文执行
 */
void Servo_Protocol_InputFeedback(const uint8_t *data, uint16_t length)
{
    uint16_t dataIndex;
    uint8_t byte;

    if ((data == NULL) || (length == 0))
    {
        return;
    }

    for (dataIndex = 0; dataIndex < length; dataIndex++)
    {
        byte = data[dataIndex];

        if (s_feedbackIndex == 0)
        {
            if (byte == SERVO_PACKET_HEADER)
            {
                s_feedbackFrame[0] = byte;
                s_feedbackIndex = 1;
            }
            continue;
        }

        if (s_feedbackIndex == 1)
        {
            if (byte == SERVO_PACKET_HEADER)
            {
                s_feedbackFrame[1] = byte;
                s_feedbackIndex = 2;
            }
            else
            {
                s_feedbackIndex = 0;
            }
            continue;
        }

        if (s_feedbackIndex >= SERVO_FEEDBACK_FRAME_MAX)
        {
            s_feedbackIndex = 0;
            s_feedbackExpectedLength = 0;
            continue;
        }

        s_feedbackFrame[s_feedbackIndex++] = byte;

        if (s_feedbackIndex == 4)
        {
            s_feedbackExpectedLength = (uint16_t)s_feedbackFrame[3] + 4;
            if ((s_feedbackFrame[3] < 2) ||
                (s_feedbackExpectedLength > SERVO_FEEDBACK_FRAME_MAX))
            {
                s_feedbackIndex = 0;
                s_feedbackExpectedLength = 0;
            }
            continue;
        }

        if ((s_feedbackExpectedLength != 0) &&
            (s_feedbackIndex >= s_feedbackExpectedLength))
        {
            if (Servo_Protocol_CheckFeedback(s_feedbackFrame,
                                             s_feedbackExpectedLength) != 0)
            {
                Servo_Protocol_DeliverFeedback(s_feedbackFrame,
                                               s_feedbackExpectedLength);
            }

            s_feedbackIndex = 0;
            s_feedbackExpectedLength = 0;
        }
    }
}

/*
 *函数简介:舵机反馈包的默认处理接口
 *参数说明:feedback指向已经通过校验的反馈数据
 *返回类型:无
 *备注:弱函数默认不处理数据；后续添加反馈寄存器读取时可定义同名函数覆盖
 */
__weak void Servo_FeedbackReceived(const ServoFeedbackPacket_t *feedback)
{
    (void)feedback;
}
