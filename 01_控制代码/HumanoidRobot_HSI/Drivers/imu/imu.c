#include "imu.h"

#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

/* 保存一个完整的11字节JY61原始数据帧。 */
uint8_t JY61_RxBuffer[JY61_FRAME_LENGTH];

/* 保存最近一次校验正确并完成换算的IMU数据。 */
static IMU_Data_t s_imuData;

/* 流式接收使用的11字节组帧缓冲区和当前字节位置。 */
static uint8_t s_streamFrame[JY61_FRAME_LENGTH];
static uint8_t s_streamIndex;

static int16_t JY61_ReadInt16LE(const uint8_t *data);
static uint8_t JY61_IsSupportedFrameType(uint8_t frameType);
static uint8_t JY61_CheckFrame(const uint8_t *frame);
static void JY61_ParseFrame(const uint8_t *frame);

/*
 *函数简介:读取JY61协议中低字节在前的有符号16位原始数据
 *参数说明:data指向两个连续字节；data[0]为低8位，data[1]为高8位
 *返回类型:int16_t，返回组合后的有符号原始值
 *备注:先使用uint16_t完成移位和组合，再转换为int16_t以正确保留补码符号
 */
static int16_t JY61_ReadInt16LE(const uint8_t *data)
{
    uint16_t value;

    value = (uint16_t)data[0];
    value |= (uint16_t)((uint16_t)data[1] << 8);

    return (int16_t)value;
}

/*
 *函数简介:判断是否为当前机器人程序需要处理的JY61原始帧类型
 *参数说明:frameType为原始帧的第二个字节
 *返回类型:uint8_t，返回1表示支持，返回0表示不处理
 *备注:当前只处理0x51加速度、0x52角速度和0x53欧拉角三种原始数据帧
 */
static uint8_t JY61_IsSupportedFrameType(uint8_t frameType)
{
    if ((frameType == JY61_FRAME_ACCEL) ||
        (frameType == JY61_FRAME_GYRO) ||
        (frameType == JY61_FRAME_ANGLE))
    {
        return 1;
    }

    return 0;
}

/*
 *函数简介:检查一个11字节JY61原始数据帧是否正确
 *参数说明:frame指向已经接收完成的11字节原始数据帧
 *返回类型:uint8_t，返回1表示帧头、类型和校验和正确，返回0表示该帧无效
 *备注:校验和为前10个原始字节累加结果的低8位，应与第11个字节相等
 */
static uint8_t JY61_CheckFrame(const uint8_t *frame)
{
    uint8_t checksum = 0;
    uint8_t index;

    if (frame == NULL)
    {
        return 0;
    }

    if (frame[0] != JY61_FRAME_HEADER)
    {
        return 0;
    }

    if (JY61_IsSupportedFrameType(frame[1]) == 0)
    {
        return 0;
    }

    for (index = 0; index < (JY61_FRAME_LENGTH - 1); index++)
    {
        checksum = (uint8_t)(checksum + frame[index]);
    }

    return (checksum == frame[JY61_FRAME_LENGTH - 1]) ? 1 : 0;
}

/*
 *函数简介:根据原始帧类型换算并更新对应的IMU数据
 *参数说明:frame指向已经通过校验的11字节原始数据帧
 *返回类型:无
 *备注:0x51、0x52和0x53互相独立；收到哪种帧就只更新哪组三轴数据，
 *     没有收到的其他类型数据继续保留上一次有效值
 */
static void JY61_ParseFrame(const uint8_t *frame)
{
    int16_t rawX;
    int16_t rawY;
    int16_t rawZ;
    float valueX;
    float valueY;
    float valueZ;

    if (JY61_CheckFrame(frame) == 0)
    {
        return;
    }

    /* 三种原始帧的X、Y、Z数据都位于第3～第8字节。 */
    rawX = JY61_ReadInt16LE(&frame[2]);
    rawY = JY61_ReadInt16LE(&frame[4]);
    rawZ = JY61_ReadInt16LE(&frame[6]);

    switch (frame[1])
    {
        case JY61_FRAME_ACCEL:
            /*
             * 原始加速度范围为-32768～32767，对应正负16g。
             * 最后乘标准重力加速度，将单位从g换算为m/s^2。
             */
            valueX = ((float)rawX / 32768.0f)
                   * JY61_ACCEL_RANGE_G * JY61_GRAVITY_MPS2;
            valueY = ((float)rawY / 32768.0f)
                   * JY61_ACCEL_RANGE_G * JY61_GRAVITY_MPS2;
            valueZ = ((float)rawZ / 32768.0f)
                   * JY61_ACCEL_RANGE_G * JY61_GRAVITY_MPS2;

            taskENTER_CRITICAL();
            s_imuData.accel_x = valueX;
            s_imuData.accel_y = valueY;
            s_imuData.accel_z = valueZ;
            taskEXIT_CRITICAL();
            break;

        case JY61_FRAME_GYRO:
            /*
             * 原始角速度范围为-32768～32767，对应配置的角速度量程。
             * 换算后的单位为度每秒。
             */
            valueX = ((float)rawX / 32768.0f) * JY61_GYRO_RANGE_DPS;
            valueY = ((float)rawY / 32768.0f) * JY61_GYRO_RANGE_DPS;
            valueZ = ((float)rawZ / 32768.0f) * JY61_GYRO_RANGE_DPS;

            taskENTER_CRITICAL();
            s_imuData.gyro_x = valueX;
            s_imuData.gyro_y = valueY;
            s_imuData.gyro_z = valueZ;
            taskEXIT_CRITICAL();
            break;

        case JY61_FRAME_ANGLE:
            /*
             * 原始欧拉角范围为-32768～32767，对应正负180度。
             * X、Y、Z依次表示Roll、Pitch、Yaw。
             */
            valueX = ((float)rawX / 32768.0f) * JY61_ANGLE_RANGE_DEG;
            valueY = ((float)rawY / 32768.0f) * JY61_ANGLE_RANGE_DEG;
            valueZ = ((float)rawZ / 32768.0f) * JY61_ANGLE_RANGE_DEG;

            taskENTER_CRITICAL();
            s_imuData.roll = valueX;
            s_imuData.pitch = valueY;
            s_imuData.yaw = valueZ;
            taskEXIT_CRITICAL();
            break;

        default:
            /* 前面已经检查过帧类型，正常情况下不会执行到这里。 */
            break;
    }
}

/*
 *函数简介:连续接收并组装JY61的11字节原始数据帧
 *参数说明:data指向本次收到的串口原始数据；len表示数据中的有效字节数
 *返回类型:无
 *备注:组帧状态可以跨多次调用保留；帧头、类型或校验和错误时直接放弃当前帧，
 *     继续从后续原始数据中寻找下一个0x55
 */
void JY61_StreamInput(const uint8_t *data, uint16_t len)
{
    uint16_t dataIndex;
    uint8_t byte;

    if ((data == NULL) || (len == 0))
    {
        return;
    }

    for (dataIndex = 0; dataIndex < len; dataIndex++)
    {
        byte = data[dataIndex];

        /* 没有开始组帧时，只等待原始帧头0x55。 */
        if (s_streamIndex == 0)
        {
            if (byte == JY61_FRAME_HEADER)
            {
                s_streamFrame[0] = byte;
                s_streamIndex = 1;
            }

            continue;
        }

        /* 第二个字节必须是当前需要的0x51、0x52或0x53。 */
        if (s_streamIndex == 1)
        {
            if (JY61_IsSupportedFrameType(byte) != 0)
            {
                s_streamFrame[1] = byte;
                s_streamIndex = 2;
            }
            else if (byte == JY61_FRAME_HEADER)
            {
                /* 连续收到0x55时，把后一个0x55作为新的帧头。 */
                s_streamFrame[0] = byte;
                s_streamIndex = 1;
            }
            else
            {
                /* 类型不正确，直接放弃当前帧。 */
                s_streamIndex = 0;
            }

            continue;
        }

        s_streamFrame[s_streamIndex] = byte;
        s_streamIndex++;

        if (s_streamIndex >= JY61_FRAME_LENGTH)
        {
            if (JY61_CheckFrame(s_streamFrame) != 0)
            {
                JY61_ParseFrame(s_streamFrame);
            }

            /* 无论校验是否成功，这一帧都已经处理结束，重新等待0x55。 */
            s_streamIndex = 0;
        }
    }
}

/*
 *函数简介:把一段任意长度的JY61串口原始数据送入协议解析器
 *参数说明:data指向原始数据；len表示原始数据中的有效字节数
 *返回类型:无
 *备注:该函数保留为通用解析入口，内部直接使用11字节流式组帧器
 */
void IMU_Parse(uint8_t *data, uint16_t len)
{
    JY61_StreamInput(data, len);
}

/*
 *函数简介:处理一个已经接收完成的11字节JY61原始数据帧
 *参数说明:buffer指向固定11字节缓冲区，帧类型可以是0x51、0x52或0x53
 *返回类型:无
 *备注:该接口适合已经在其他位置完成11字节组帧的情况，校验失败时直接返回
 */
void JY61_Process(uint8_t *buffer)
{
    if (buffer == NULL)
    {
        return;
    }

    JY61_ParseFrame(buffer);
}

/*
 *函数简介:获取当前最新的IMU测量数据快照
 *参数说明:无
 *返回类型:IMU_Data_t，包含三轴加速度、三轴角速度和三个欧拉角
 *备注:返回的是结构体副本；临界区保证复制期间解析任务不会修改结构体内容
 */
IMU_Data_t IMU_GetData(void)
{
    IMU_Data_t snapshot;

    taskENTER_CRITICAL();
    snapshot = s_imuData;
    taskEXIT_CRITICAL();

    return snapshot;
}
