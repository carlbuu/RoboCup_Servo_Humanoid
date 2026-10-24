#ifndef SERVO_MANAGER_H
#define SERVO_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "servo.h"

/*
 *函数简介:让ServoTask取出并发送队列中的最新同步目标
 *参数说明:无
 *返回类型:uint8_t，成功启动DMA发送返回1，队列为空或启动失败返回0
 *备注:这是舵机模块内部接口；用户程序和步态库只需要包含servo.h，不要直接调用
 */
uint8_t Servo_Manager_SendNextTarget(void);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_MANAGER_H */
