#include "servo_task.h"

#include "FreeRTOS.h"
#include "task.h"

#include "main.h"
#include "motion_groups.h"
#include "robot_joints.h"
#include "servo.h"
#include "servo_bus.h"
#include "servo_config.h"
#include "servo_manager.h"

/*
 *函数简介:FreeRTOS舵机控制任务入口
 *参数说明:argument为任务入口参数，当前不使用
 *返回类型:无
 *备注:任务每20ms检查一次最新运动目标；只有读到新目标时才启动USART3 DMA，
 *     固定姿态不会被无意义地重复发送；DMA完成由中断通知，不使用忙等待
 */
void ServoTask(void *argument)
{
    TickType_t lastWakeTime;
    const MotionGroup_t *stand;
    uint8_t joint;
    uint8_t id;

    (void)argument;

    /*
     * 将当前任务句柄交给总线层。
     * USART3发送完成、接收IDLE和错误中断才能通知到本任务。
     */
    Servo_Bus_BindTask(xTaskGetCurrentTaskHandle());

    /*
     * 在调度器启动后开启USART3循环DMA接收和IDLE中断。
     * 中断只搬运反馈数据，SCS协议解析仍然在本任务中执行。
     */
    if (Servo_Bus_StartReceive() != HAL_OK)
    {
        Error_Handler();
    }

    /* 给URT-2和舵机电源预留稳定时间。 */
    vTaskDelay(pdMS_TO_TICKS(100));

    /*
     * 扭矩关闭时先写入低速站立目标。否则使能后舵机可能立即追逐
     * 上一次保留的目标位置，造成上电瞬间猛动。
     */
    stand = MotionGroups_Get(motion_stand);
    if ((stand == NULL) || (stand->frames == NULL) || (stand->frameCount == 0U))
    {
        Error_Handler();
    }

    for (joint = 0U; joint < ROBOT_JOINT_COUNT; joint++)
    {
        id = ROBOT_SERVO_ID(joint);
        if ((id == 0U) || (id > SERVO_ACTIVE_COUNT))
        {
            continue;
        }

        if (Servo_SetTarget(id,
                            SERVO_STARTUP_STAND_SPEED,
                            SERVO_STARTUP_STAND_ACCELERATION,
                            (float)stand->frames[0].angleDeg10[joint] * 0.1f) == 0U)
        {
            Error_Handler();
        }
    }

    if ((Servo_SendTargets() == 0U) ||
        (Servo_Manager_SendNextTarget() == 0U) ||
        (Servo_Bus_WaitTxComplete(
            pdMS_TO_TICKS(SERVO_TX_WAIT_TIMEOUT_MS)) != pdTRUE))
    {
        Error_Handler();
    }

    /* 目标和速度已经写入舵机，再统一使能扭矩。 */
    Servo_EnableAll();
    if (Servo_IsTxBusy() != 0)
    {
        (void)Servo_Bus_WaitTxComplete(
            pdMS_TO_TICKS(SERVO_TX_WAIT_TIMEOUT_MS));
    }

    lastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        /* UART错误后的DMA停止和重新启动只能在任务上下文执行。 */
        Servo_Bus_Service();

        /* 反馈协议解析在任务中执行，不占用USART3中断时间。 */
        Servo_Bus_ProcessReceivedData();

        /*
         * 队列中有新姿态时才发送。
         * 队列为空表示上层没有提交新目标，本周期保持当前舵机状态即可。
         */
        if (Servo_Manager_SendNextTarget() != 0)
        {
            (void)Servo_Bus_WaitTxComplete(
                pdMS_TO_TICKS(SERVO_TX_WAIT_TIMEOUT_MS));
        }

        /*
         * vTaskDelayUntil()使用固定唤醒时间，避免循环执行时间累积造成周期漂移。
         * 以后步态库每20ms提交新姿态时，系统自然形成50Hz舵机更新频率。
         */
        vTaskDelayUntil(&lastWakeTime,
                        pdMS_TO_TICKS(SERVO_TASK_PERIOD_MS));
    }
}
