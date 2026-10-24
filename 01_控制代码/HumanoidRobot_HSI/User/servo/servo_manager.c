#include "servo_manager.h"

#include <stddef.h>
#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

#include "main.h"
#include "servo_bus.h"
#include "servo_config.h"
#include "servo_protocol.h"

/*
 * 舵机模块内部唯一的关节数据结构。
 * 它同时保存机械配置和当前待发送目标，对用户层完全隐藏。
 */
typedef struct
{
    uint8_t id;
    int8_t direction;
    uint16_t centerPosition;
    uint16_t minPosition;
    uint16_t maxPosition;

    float targetAngle;
    uint16_t targetSpeed;
    uint8_t targetAcceleration;
    uint8_t pending;
} ServoState_t;

/*
 * 队列内部保存已经组好的SCS数据包。
 * 队列长度为1，新姿态可以覆盖尚未发送的旧姿态。
 */
typedef struct
{
    uint8_t data[SERVO_TX_BUFFER_SIZE];
    uint16_t length;
} ServoMotionPacket_t;

/*
 * ID1~ID5 沿用原工程左腿实测机械范围。
 * ID6~ID10、ID13 沿用既有软件限位；装机后仍必须按照
 * Docs/14舵机ID与标定说明.md 逐个核对中位、方向和机械止挡。
 *
 * direction 的意义是“动作表正角度对应的电机方向”，按各关节安装方向设置。
 * 2026-10-16：按用户要求，ID11、ID12统一使用2048零位，范围0~4095。
 * 保留ID11的-1方向、ID12的+1方向；腿部限位和通用位置截断保持不变。
 * 用户已确认手臂机械可360度旋转；新零位的下垂姿态及动作路径仍需实机验证。
 * 任何机械安装方向变化都只修改本表，不修改动作组。
 */
static ServoState_t s_servo[SERVO_COUNT] =
{
    /* ID 方向 中位  最小  最大  角度  速度                      加速度                     待发送 */
    {  1,  1, 2048, 1799, 2609, 0.0f, SERVO_DEFAULT_SPEED, SERVO_DEFAULT_ACCELERATION, 0 },
    {  2,  -1, 2048, 1589, 2520, 0.0f, SERVO_DEFAULT_SPEED, SERVO_DEFAULT_ACCELERATION, 0 },
    {  3,  -1, 2048, 1705, 2954, 0.0f, SERVO_DEFAULT_SPEED, SERVO_DEFAULT_ACCELERATION, 0 },
    {  4,  1, 2048, 1628, 2873, 0.0f, SERVO_DEFAULT_SPEED, SERVO_DEFAULT_ACCELERATION, 0 },
    {  5,  1, 2048, 1720, 2490, 0.0f, SERVO_DEFAULT_SPEED, SERVO_DEFAULT_ACCELERATION, 0 },
    {  6, -1, 2048, 1476, 2464, 0.0f, SERVO_DEFAULT_SPEED, SERVO_DEFAULT_ACCELERATION, 0 },
    {  7, 1, 2048, 1633, 2873, 0.0f, SERVO_DEFAULT_SPEED, SERVO_DEFAULT_ACCELERATION, 0 },
    {  8,  -1, 2048, 1660, 2607, 0.0f, SERVO_DEFAULT_SPEED, SERVO_DEFAULT_ACCELERATION, 0 },
    {  9, 1, 2048, 1512, 2581, 0.0f, SERVO_DEFAULT_SPEED, SERVO_DEFAULT_ACCELERATION, 0 },
    { 10, -1, 2048, 1352, 2266, 0.0f, SERVO_DEFAULT_SPEED, SERVO_DEFAULT_ACCELERATION, 0 },
    { 11, -1, 2048,    0, 4095, 0.0f, SERVO_DEFAULT_SPEED, SERVO_DEFAULT_ACCELERATION, 0 },
    { 12,  1, 2048,    0, 4095, 0.0f, SERVO_DEFAULT_SPEED, SERVO_DEFAULT_ACCELERATION, 0 },
    { 13,  1, 2048,     0, 4096, 0.0f, SERVO_DEFAULT_SPEED, SERVO_DEFAULT_ACCELERATION, 0 }
};

static QueueHandle_t s_motionQueue;
static SemaphoreHandle_t s_servoMutex;
static uint8_t s_packetBuffer[SERVO_TX_BUFFER_SIZE];

static int16_t Servo_FindIndexById(uint8_t id);
static uint16_t Servo_AngleToPositionByIndex(float angle, uint8_t index);
static void Servo_WriteRegister(uint8_t id,
                                uint8_t address,
                                const uint8_t *data,
                                uint8_t dataLength);
static void Servo_SetAllTorque(uint8_t enable);

/*
 *函数简介:根据舵机ID查找内部数组下标
 *参数说明:id为需要查找的舵机ID
 *返回类型:int16_t，找到时返回0到SERVO_COUNT-1，未找到返回-1
 *备注:所有角度换算都必须先通过本函数完成ID映射，禁止把ID直接当数组下标使用
 */
static int16_t Servo_FindIndexById(uint8_t id)
{
    uint8_t index;

    /* 只接受当前启用范围内的舵机ID。 */
    if ((id == 0U) || (id > SERVO_ACTIVE_COUNT))
    {
        return -1;
    }

    for (index = 0; index < SERVO_COUNT; index++)
    {
        if (s_servo[index].id == id)
        {
            return (int16_t)index;
        }
    }

    return -1;
}

/*
 *函数简介:使用指定关节配置把角度换算成舵机位置值
 *参数说明:angle为目标角度；index为已经确认有效的内部数组下标
 *返回类型:uint16_t，返回经过关节机械范围保护后的位置值
 *备注:本函数不进行ID查找，也不操作互斥量，只供舵机管理模块内部调用
 */
static uint16_t Servo_AngleToPositionByIndex(float angle, uint8_t index)
{
    float position;
    int8_t direction;

    direction = (s_servo[index].direction < 0) ? -1 : 1;
    position = (float)s_servo[index].centerPosition +
               (angle / SERVO_DEGREES_PER_POSITION) * (float)direction;

    if (position <= (float)s_servo[index].minPosition)
    {
        return s_servo[index].minPosition;
    }

    if (position >= (float)s_servo[index].maxPosition)
    {
        return s_servo[index].maxPosition;
    }

    return (uint16_t)(position + 0.5f);
}

/*
 *函数简介:初始化STS3215舵机控制模块
 *参数说明:无
 *返回类型:无
 *备注:队列长度为1，只保存最新同步目标；创建失败时进入Error_Handler()，
 *     避免系统在舵机通信对象无效的情况下继续运行
 */
void Servo_Init(void)
{
    uint8_t index;

    Servo_Bus_Init();
    memset(s_packetBuffer, 0, sizeof(s_packetBuffer));

    for (index = 0; index < SERVO_COUNT; index++)
    {
        s_servo[index].targetAngle = 0.0f;
        s_servo[index].targetSpeed = SERVO_DEFAULT_SPEED;
        s_servo[index].targetAcceleration = SERVO_DEFAULT_ACCELERATION;
        s_servo[index].pending = 0;
    }

    s_motionQueue = xQueueCreate(1, sizeof(ServoMotionPacket_t));
    s_servoMutex = xSemaphoreCreateMutex();

    if ((s_motionQueue == NULL) || (s_servoMutex == NULL))
    {
        Error_Handler();
    }
}

/*
 *函数简介:保存一个舵机的待发送运动目标
 *参数说明:id为舵机ID；speed为速度；acceleration为加速度；angle为角度
 *返回类型:uint8_t，成功返回1，ID无效或模块未初始化返回0
 *备注:这里只修改内存并设置pending标志，不启动DMA，因此可以安全地连续设置多个舵机
 */
uint8_t Servo_SetTarget(uint8_t id,
                        uint16_t speed,
                        uint8_t acceleration,
                        float angle)
{
    int16_t index;

    if (s_servoMutex == NULL)
    {
        return 0;
    }

    index = Servo_FindIndexById(id);
    if (index < 0)
    {
        return 0;
    }

    if (xSemaphoreTake(s_servoMutex, pdMS_TO_TICKS(2)) != pdTRUE)
    {
        return 0;
    }

    s_servo[index].targetAngle = angle;
    s_servo[index].targetSpeed = speed;
    s_servo[index].targetAcceleration = acceleration;
    s_servo[index].pending = 1;

    (void)xSemaphoreGive(s_servoMutex);
    return 1;
}

/*
 *函数简介:提交所有已经设置的舵机同步目标
 *参数说明:无
 *返回类型:uint8_t，成功放入最新目标队列返回1，没有目标或组包失败返回0
 *备注:函数只把已经组好的数据包复制进队列；ServoTask稍后读取队列并启动USART3 DMA
 */
uint8_t Servo_SendTargets(void)
{
    ServoSyncTarget_t targets[SERVO_COUNT];
    ServoMotionPacket_t packet;
    uint8_t index;
    uint8_t count;

    if ((s_servoMutex == NULL) || (s_motionQueue == NULL))
    {
        return 0;
    }

    if (xSemaphoreTake(s_servoMutex, pdMS_TO_TICKS(2)) != pdTRUE)
    {
        return 0;
    }

    count = 0;
    for (index = 0; index < SERVO_COUNT; index++)
    {
        if (s_servo[index].pending == 0)
        {
            continue;
        }

        targets[count].id = s_servo[index].id;
        targets[count].acceleration = s_servo[index].targetAcceleration;
        targets[count].position =
            Servo_AngleToPositionByIndex(s_servo[index].targetAngle, index);
        targets[count].speed = s_servo[index].targetSpeed;
        count++;
    }

    if (count == 0)
    {
        (void)xSemaphoreGive(s_servoMutex);
        return 0;
    }

    packet.length = Servo_Protocol_BuildSyncMotion(targets,
                                                    count,
                                                    packet.data,
                                                    sizeof(packet.data));
    if (packet.length == 0)
    {
        (void)xSemaphoreGive(s_servoMutex);
        return 0;
    }

    for (index = 0; index < SERVO_COUNT; index++)
    {
        s_servo[index].pending = 0;
    }

    (void)xSemaphoreGive(s_servoMutex);

    /*
     * xQueueOverwrite()专门用于长度为1的队列。
     * 如果旧姿态还没有被ServoTask读取，新姿态会覆盖旧姿态，避免积压过期动作。
     */
    return (xQueueOverwrite(s_motionQueue, &packet) == pdPASS) ? 1 : 0;
}

/*
 *函数简介:立即提交一个舵机的运动目标
 *参数说明:id为舵机ID；speed为速度；acceleration为加速度；angle为角度
 *返回类型:uint8_t，成功返回1，失败返回0
 *备注:多舵机同步控制不要重复调用本函数，应多次设置目标后只发送一次
 */
uint8_t Servo_Control(uint8_t id,
                      uint16_t speed,
                      uint8_t acceleration,
                      float angle)
{
    if (Servo_SetTarget(id, speed, acceleration, angle) == 0)
    {
        return 0;
    }

    return Servo_SendTargets();
}

/*
 *函数简介:把关节角度换算成STS3215位置值
 *参数说明:angle为角度；id为舵机ID
 *返回类型:uint16_t，返回经过方向和机械范围限制的位置值
 *备注:使用ID查找配置，不再要求用户记住ID1对应数组下标0之类的内部关系
 */
uint16_t Joint_AngleToPosition(float angle, uint8_t id)
{
    int16_t index;

    index = Servo_FindIndexById(id);
    if (index < 0)
    {
        return SERVO_POSITION_CENTER;
    }

    return Servo_AngleToPositionByIndex(angle, (uint8_t)index);
}

/*
 *函数简介:生成并发送一条普通寄存器写指令
 *参数说明:id为舵机ID；address为寄存器地址；data为写入数据；dataLength为数据长度
 *返回类型:无
 *备注:该函数只用于扭矩和原始位置调试；总线忙时不会覆盖正在发送的数据
 */
static void Servo_WriteRegister(uint8_t id,
                                uint8_t address,
                                const uint8_t *data,
                                uint8_t dataLength)
{
    uint16_t packetLength;

    if ((Servo_FindIndexById(id) < 0) || (data == NULL) ||
        (s_servoMutex == NULL))
    {
        return;
    }

    if (xSemaphoreTake(s_servoMutex, pdMS_TO_TICKS(2)) != pdTRUE)
    {
        return;
    }

    packetLength = Servo_Protocol_BuildWrite(id,
                                              address,
                                              data,
                                              dataLength,
                                              s_packetBuffer,
                                              sizeof(s_packetBuffer));
    if (packetLength > 0)
    {
        (void)Servo_Bus_Send(s_packetBuffer, packetLength);
    }

    (void)xSemaphoreGive(s_servoMutex);
}

/*
 *函数简介:开启一个舵机的扭矩输出
 *参数说明:id为目标舵机ID
 *返回类型:无
 *备注:向0x28写入0x01
 */
void Servo_EnableTorque(uint8_t id)
{
    uint8_t enable;

    enable = 0x01;
    Servo_WriteRegister(id,
                        SERVO_REGISTER_TORQUE_ENABLE,
                        &enable,
                        SERVO_TORQUE_DATA_LENGTH);
}

/*
 *函数简介:关闭一个舵机的扭矩输出
 *参数说明:id为目标舵机ID
 *返回类型:无
 *备注:向0x28写入0x00
 */
void Servo_DisableTorque(uint8_t id)
{
    uint8_t enable;

    enable = 0x00;
    Servo_WriteRegister(id,
                        SERVO_REGISTER_TORQUE_ENABLE,
                        &enable,
                        SERVO_TORQUE_DATA_LENGTH);
}

/*
 *函数简介:同步设置全部舵机的扭矩状态
 *参数说明:enable为1时开启扭矩，为0时关闭扭矩
 *返回类型:无
 *备注:只生成并发送一条广播SYNC WRITE数据包
 */
static void Servo_SetAllTorque(uint8_t enable)
{
    uint8_t ids[SERVO_COUNT];
    uint8_t values[SERVO_COUNT];
    uint8_t index;
    uint16_t packetLength;

    if (s_servoMutex == NULL)
    {
        return;
    }

    if (xSemaphoreTake(s_servoMutex, pdMS_TO_TICKS(2)) != pdTRUE)
    {
        return;
    }

    for (index = 0; index < SERVO_ACTIVE_COUNT; index++)
    {
        ids[index] = s_servo[index].id;
        values[index] = (enable != 0) ? 0x01 : 0x00;
    }

    packetLength = Servo_Protocol_BuildSyncByte(
        SERVO_REGISTER_TORQUE_ENABLE,
        ids,
        values,
        SERVO_ACTIVE_COUNT,
        s_packetBuffer,
        sizeof(s_packetBuffer));

    if (packetLength > 0)
    {
        (void)Servo_Bus_Send(s_packetBuffer, packetLength);
    }

    (void)xSemaphoreGive(s_servoMutex);
}

/*
 *函数简介:同步开启全部舵机扭矩
 *参数说明:无
 *返回类型:无
 *备注:由ServoTask启动USART3接收DMA后调用一次
 */
void Servo_EnableAll(void)
{
    Servo_SetAllTorque(1);
}

/*
 *函数简介:同步关闭全部舵机扭矩
 *参数说明:无
 *返回类型:无
 *备注:发送广播卸力指令
 */
void Servo_DisableAll(void)
{
    Servo_SetAllTorque(0);
}

/*
 *函数简介:设置一个舵机的原始位置值
 *参数说明:id为舵机ID；position为STS3215原始位置值
 *返回类型:无
 *备注:位置值会限制在0到4095，随后按小端顺序写入0x2A和0x2B
 */
void Servo_SetPosition(uint8_t id, uint16_t position)
{
    uint8_t data[SERVO_POSITION_DATA_LENGTH];

    if (position > SERVO_POSITION_MAX)
    {
        position = SERVO_POSITION_MAX;
    }

    data[0] = (uint8_t)(position & 0x00FF);
    data[1] = (uint8_t)(position >> 8);

    Servo_WriteRegister(id,
                        SERVO_REGISTER_TARGET_POSITION,
                        data,
                        SERVO_POSITION_DATA_LENGTH);
}

/*
 *函数简介:让ServoTask取出并发送队列中的最新同步目标
 *参数说明:无
 *返回类型:uint8_t，成功启动DMA返回1，队列为空或发送失败返回0
 *备注:只有ServoTask调用本函数，保证正常运动命令不会从多个任务同时操作USART3 DMA
 */
uint8_t Servo_Manager_SendNextTarget(void)
{
    ServoMotionPacket_t packet;

    if (s_motionQueue == NULL)
    {
        return 0;
    }

    if (xQueueReceive(s_motionQueue, &packet, 0) != pdPASS)
    {
        return 0;
    }

    return (Servo_Bus_Send(packet.data, packet.length) == HAL_OK) ? 1 : 0;
}

/*
 *函数简介:读取USART3 DMA发送忙状态
 *参数说明:无
 *返回类型:uint8_t，1表示忙，0表示空闲
 *备注:直接返回舵机总线层状态
 */
uint8_t Servo_IsTxBusy(void)
{
    return Servo_Bus_IsTxBusy();
}

