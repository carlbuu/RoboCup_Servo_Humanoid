#include "robot_walk_config.h"
#include "robot_control.h"
#include "motion_player.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#if ROBOT_RUN_MODE != ROBOT_MODE_ACTION_DEBUG
#include "robot_walk_vision.h"
#else
static RobotControlStatus_t s_actionStatus;
static uint32_t s_actionStartupMs;
static uint8_t s_actionStarted;
#endif

/**
 * @brief 初始化动作调试或视觉控制模式
 * @param 无
 * @retval 无
 */
void RobotControl_Init(void)
{
#if ROBOT_RUN_MODE == ROBOT_MODE_ACTION_DEBUG
    RobotControlStatus_t initial;
    memset(&initial, 0, sizeof(initial));
    s_actionStartupMs = 0U;
    s_actionStarted = 0U;
    MotionPlayer_Init();
    initial.state = ROBOT_STATE_RUNNING;
    initial.motion = MotionPlayer_GetMotion();
    taskENTER_CRITICAL();
    s_actionStatus = initial;
    taskEXIT_CRITICAL();
#else
    RobotWalkVision_Init();
#endif
}

/* Explicit lifecycle reset, guarded by the vision scheduler's standing check. */
uint8_t RobotControl_BeginNewRun(void)
{
#if ROBOT_RUN_MODE == ROBOT_MODE_ACTION_DEBUG
    return 0U;
#else
    return RobotWalkVision_BeginNewRun();
#endif
}

/**
 * @brief 按固定时间间隔推进已选模式
 * @param elapsedMs 本次推进的毫秒数，0不推进
 * @retval 无
 * @note 动作调试只启动一次所选动作，动作表loop字段决定是否循环。
 */
void RobotControl_Update(uint16_t elapsedMs)
{
#if ROBOT_RUN_MODE == ROBOT_MODE_ACTION_DEBUG
    RobotControlStatus_t next;
    const MotionGroup_t *group;
    if (elapsedMs == 0U || s_actionStatus.state == ROBOT_STATE_FAULT) { return; }
    next = s_actionStatus;
    if (s_actionStarted == 0U)
    {
        s_actionStartupMs += elapsedMs;
        if (s_actionStartupMs >= ROBOT_ACTION_START_DELAY_MS)
        {
            group = MotionGroups_Get(ROBOT_DEBUG_MOTION);
            if ((unsigned int)ROBOT_DEBUG_MOTION >= (unsigned int)motion_group_count ||
                group == NULL || group->frameCount == 0U)
            {
                next.state = ROBOT_STATE_FAULT;
            }
            else
            {
                MotionPlayer_Start(ROBOT_DEBUG_MOTION);
                s_actionStarted = 1U;
            }
        }
    }
    if (next.state != ROBOT_STATE_FAULT)
    {
        (void)MotionPlayer_Update(elapsedMs);
        next.state = MotionPlayer_IsActive() != 0U
            ? ROBOT_STATE_RUNNING : ROBOT_STATE_FINISHED;
        next.motion = MotionPlayer_GetMotion();
        next.frameIndex = MotionPlayer_GetFrameIndex();
        next.completedCycles = MotionPlayer_GetCompletedCycles();
    }
    taskENTER_CRITICAL();
    s_actionStatus = next;
    taskEXIT_CRITICAL();
#else
    RobotWalkVision_Update(elapsedMs);
#endif
}

/**
 * @brief 等待舵机总线稳定后运行所选模式
 * @param argument 未使用
 * @retval 无
 */
void RobotTask(void *argument)
{
    TickType_t lastWakeTime;
    (void)argument;
    vTaskDelay(pdMS_TO_TICKS(ROBOT_STARTUP_SETTLE_MS));
    RobotControl_Init();
    lastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        RobotControl_Update(ROBOT_CONTROL_PERIOD_MS);
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(ROBOT_CONTROL_PERIOD_MS));
    }
}

/** @brief 获取一致状态快照；@param 无；@retval 状态副本。 */
RobotControlStatus_t RobotControl_GetStatus(void)
{
#if ROBOT_RUN_MODE == ROBOT_MODE_ACTION_DEBUG
    RobotControlStatus_t snapshot;
    taskENTER_CRITICAL();
    snapshot = s_actionStatus;
    taskEXIT_CRITICAL();
    return snapshot;
#else
    return RobotWalkVision_GetStatus();
#endif
}

/** @brief 返回运行模式名称；@param 无；@retval 静态字符串。 */
const char *RobotControl_GetModeName(void)
{
#if ROBOT_RUN_MODE == ROBOT_MODE_ACTION_DEBUG
    return "action_debug";
#elif ROBOT_RUN_MODE == ROBOT_MODE_VISION_DEBUG
    return "vision_debug";
#elif ROBOT_RUN_MODE == ROBOT_MODE_LINE_DEBUG
    return "line_debug";
#else
    return "full_run";
#endif
}

/** @brief 返回状态名称；@param state 机器人状态；@retval 静态字符串。 */
const char *RobotControl_GetStateName(RobotControlState_t state)
{
    static const char * const names[] = { "wait", "run", "finished", "fault" };
    return (unsigned int)state < 4U ? names[state] : "unknown";
}

/** @brief 返回视觉调度阶段；@param phase 阶段；@retval 静态字符串。 */
const char *RobotControl_GetWalkPhaseName(RobotWalkPhase_t phase)
{
    static const char * const names[] = {
        "startup", "moving", "to_stand", "settling", "shape_action",
        "recover", "blocked", "fault", "wait_vision",
        "shape_step", "shape_step_return", "turn_approach", "walk_turn"
    };
    return (unsigned int)phase < sizeof(names) / sizeof(names[0])
        ? names[phase] : "unknown";
}
