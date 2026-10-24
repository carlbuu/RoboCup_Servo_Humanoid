#ifndef SERVO_TASK_H
#define SERVO_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 *函数简介:FreeRTOS舵机控制任务入口
 *参数说明:argument为FreeRTOS任务参数，本模块当前不使用
 *返回类型:无
 *备注:任务优先级配置为Normal，周期20ms；负责读取最新运动队列、生成SYNC WRITE、
 *     启动USART3 TX DMA并等待发送完成，同时在任务上下文处理反馈数据和错误恢复
 */
void ServoTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_TASK_H */
