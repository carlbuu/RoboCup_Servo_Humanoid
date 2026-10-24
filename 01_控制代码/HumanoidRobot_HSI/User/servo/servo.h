#ifndef SERVO_H
#define SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 *函数简介:初始化STS3215舵机控制模块
 *参数说明:无
 *返回类型:无
 *备注:创建内部FreeRTOS队列和互斥量，并初始化USART3舵机总线的软件状态；
 *     本函数不会发送位置命令，也不会让舵机突然运动
 */
void Servo_Init(void);

/*
 *函数简介:保存一个舵机的待发送运动目标
 *参数说明:id为舵机ID；speed为运行速度；acceleration为运行加速度；
 *         angle为相对中立位置的目标角度，单位为度，允许正数、负数和小数
 *返回类型:uint8_t，保存成功返回1，ID无效或模块未初始化返回0
 *备注:本函数只修改STM32内存，不会立即发送串口数据；连续设置多个舵机后，
 *     调用Servo_SendTargets()即可使用一条SYNC WRITE指令同步发送
 */
uint8_t Servo_SetTarget(uint8_t id,
                        uint16_t speed,
                        uint8_t acceleration,
                        float angle);

/*
 *函数简介:提交所有已经通过Servo_SetTarget()保存的舵机目标
 *参数说明:无
 *返回类型:uint8_t，成功提交返回1，没有待发送目标或提交失败返回0
 *备注:内部自动完成角度换算、机械范围限位和SYNC WRITE组包，再使用
 *     xQueueOverwrite()提交给ServoTask；真正的USART3 DMA发送只在ServoTask中执行
 */
uint8_t Servo_SendTargets(void);

/*
 *函数简介:立即提交一个舵机的运动目标
 *参数说明:id为舵机ID；speed为运行速度；acceleration为运行加速度；
 *         angle为相对中立位置的目标角度，单位为度
 *返回类型:uint8_t，提交成功返回1，参数错误或提交失败返回0
 *备注:本函数等价于依次调用Servo_SetTarget()和Servo_SendTargets()；适合单舵机
 *     调试，多舵机同步运动应先多次调用Servo_SetTarget()，最后只调用一次发送函数
 */
uint8_t Servo_Control(uint8_t id,
                      uint16_t speed,
                      uint8_t acceleration,
                      float angle);

/*
 *函数简介:把关节角度换算成STS3215位置值
 *参数说明:angle为相对中立位置的角度，单位为度；id为舵机ID
 *返回类型:uint16_t，返回经过方向换算和该关节机械范围限制后的位置值
 *备注:换算公式为center+angle/0.087*direction；ID无效时返回中心位置2048
 */
uint16_t Joint_AngleToPosition(float angle, uint8_t id);

/*
 *函数简介:开启一个舵机的扭矩输出
 *参数说明:id为目标舵机ID
 *返回类型:无
 *备注:向扭矩开关寄存器0x28写入0x01，并使用USART3 DMA发送
 */
void Servo_EnableTorque(uint8_t id);

/*
 *函数简介:关闭一个舵机的扭矩输出
 *参数说明:id为目标舵机ID
 *返回类型:无
 *备注:向扭矩开关寄存器0x28写入0x00，并使用USART3 DMA发送
 */
void Servo_DisableTorque(uint8_t id);

/*
 *函数简介:同步开启全部已配置舵机的扭矩输出
 *参数说明:无
 *返回类型:无
 *备注:使用一条SYNC WRITE广播指令完成，不逐个发送14条命令
 */
void Servo_EnableAll(void);

/*
 *函数简介:同步关闭全部已配置舵机的扭矩输出
 *参数说明:无
 *返回类型:无
 *备注:适合机器人停止控制或人工调整机械结构前统一卸力
 */
void Servo_DisableAll(void);

/*
 *函数简介:设置一个舵机的原始位置值
 *参数说明:id为舵机ID；position为0到4095的STS3215原始位置值
 *返回类型:无
 *备注:该接口用于底层调试；正常机器人动作建议使用角度接口Servo_Control()
 */
void Servo_SetPosition(uint8_t id, uint16_t position);

/*
 *函数简介:读取USART3 DMA发送忙状态
 *参数说明:无
 *返回类型:uint8_t，1表示正在发送，0表示总线空闲
 *备注:只读取当前状态，不会阻塞等待DMA完成
 */
uint8_t Servo_IsTxBusy(void);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_H */
