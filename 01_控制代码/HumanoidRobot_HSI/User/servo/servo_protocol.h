#ifndef SERVO_PROTOCOL_H
#define SERVO_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "servo_config.h"

/*
 * 该结构体只在舵机模块内部使用。
 * 用户层和步态层不需要创建或操作该结构体。
 */
typedef struct
{
    uint8_t id;
    uint8_t acceleration;
    uint16_t position;
    uint16_t speed;
} ServoSyncTarget_t;

typedef struct
{
    uint8_t id;
    uint8_t error;
    uint8_t parameters[SERVO_FEEDBACK_PARAMETER_MAX];
    uint8_t parameterLength;
} ServoFeedbackPacket_t;

/*
 *函数简介:生成一条SCS普通写指令数据包
 *参数说明:id为舵机ID；address为寄存器地址；data为写入数据；
 *         dataLength为数据长度；output为输出缓存；outputSize为缓存容量
 *返回类型:uint16_t，成功返回数据包长度，参数错误或缓存不足返回0
 *备注:本函数只负责填写帧头、长度、指令和校验和，不负责启动UART DMA
 */
uint16_t Servo_Protocol_BuildWrite(uint8_t id,
                                   uint8_t address,
                                   const uint8_t *data,
                                   uint8_t dataLength,
                                   uint8_t *output,
                                   uint16_t outputSize);

/*
 *函数简介:生成多个STS3215舵机的同步运动数据包
 *参数说明:targets为舵机目标数组；count为本次参与运动的舵机数量；
 *         output为输出缓存；outputSize为缓存容量
 *返回类型:uint16_t，成功返回数据包长度，参数错误或缓存不足返回0
 *备注:从0x29开始连续同步写入加速度、位置、两个保留字节和速度；只包含本次
 *     Servo_SetTarget()设置过的舵机，不要求用户填写固定14项运动结构体
 */
uint16_t Servo_Protocol_BuildSyncMotion(const ServoSyncTarget_t *targets,
                                        uint8_t count,
                                        uint8_t *output,
                                        uint16_t outputSize);

/*
 *函数简介:生成多个舵机同步写入一个单字节寄存器的数据包
 *参数说明:address为寄存器地址；ids为ID数组；values为数据数组；count为数量；
 *         output为输出缓存；outputSize为缓存容量
 *返回类型:uint16_t，成功返回数据包长度，参数错误或缓存不足返回0
 *备注:当前用于Servo_EnableAll()和Servo_DisableAll()统一设置扭矩状态
 */
uint16_t Servo_Protocol_BuildSyncByte(uint8_t address,
                                      const uint8_t *ids,
                                      const uint8_t *values,
                                      uint8_t count,
                                      uint8_t *output,
                                      uint16_t outputSize);

/*
 *函数简介:向SCS反馈解析器输入一段USART3接收数据
 *参数说明:data为接收数据；length为有效字节数
 *返回类型:无
 *备注:完整帧判断和校验在ServoTask上下文完成，不在UART中断中解析协议
 */
void Servo_Protocol_InputFeedback(const uint8_t *data, uint16_t length);

/*
 *函数简介:处理一包已经校验正确的舵机反馈数据
 *参数说明:feedback指向反馈包副本
 *返回类型:无
 *备注:当前提供弱函数空实现，后续读取位置或负载时可以定义同名函数覆盖
 */
void Servo_FeedbackReceived(const ServoFeedbackPacket_t *feedback);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_PROTOCOL_H */
