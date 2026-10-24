#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include <stdint.h>
#include "motion_groups.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    ROBOT_STATE_WAIT_START = 0,
    ROBOT_STATE_RUNNING,
    ROBOT_STATE_FINISHED,
    ROBOT_STATE_FAULT
} RobotControlState_t;

typedef enum
{
    ROBOT_WALK_STARTUP = 0,
    ROBOT_WALK_MOVING,
    ROBOT_WALK_TO_STAND,
    ROBOT_WALK_SETTLING,
    ROBOT_WALK_EVENT,
    ROBOT_WALK_RECOVER,
    ROBOT_WALK_BLOCKED,
    ROBOT_WALK_FAULT,
    ROBOT_WALK_MEASURING,
    ROBOT_WALK_SHAPE_STEP,
    ROBOT_WALK_SHAPE_STEP_RETURN,
    ROBOT_WALK_TURN_APPROACH,
    ROBOT_WALK_TURN_HANDOFF
} RobotWalkPhase_t;

typedef enum
{
    ROBOT_GAIT_SLOW = 0,
    ROBOT_GAIT_FAST
} RobotGaitStage_t;

typedef struct
{
    RobotGaitStage_t gaitStage;
    uint32_t forwardSteps; /* Software single-foot completions across actions. */
    uint32_t fastSwitchSteps; /* Actual total at switch, may exceed 30. */
    uint8_t fastSwitchPending;
    RobotControlState_t state;
    MotionId_t motion;
    uint8_t frameIndex;
    uint16_t completedCycles;
    uint16_t targetCycles;
    RobotWalkPhase_t walkPhase;
    uint8_t pendingShapes;
    uint32_t eventsStarted;
    uint32_t eventsCompleted;
    uint32_t eventsDropped;
    uint32_t activeCommandSequence;
    uint32_t lastCompletedCommandSequence;
    uint32_t activeShapeSequence;
    uint32_t walkBatch;
    uint8_t sampleWindowOpen;
    uint8_t sampleCount;
    uint8_t stopRequested;
    uint8_t rightSampleWindowOpen;
    uint8_t rightSampleCount;
    uint8_t pendingTurn;
    uint8_t turnFromWalk;
    /* 上次转向截断的行走批次，仅为软件播放记录，不代表真实落脚。 */
    uint32_t lastInterruptedCommandSequence;
    uint16_t lastWalkCompletedCycles;
    uint8_t lastWalkExitFrame;
    uint8_t lastWalkPartialSteps;
} RobotControlStatus_t;

/** @brief 四模式共用的机器人任务入口；@param argument 未使用；@retval 无。 */
void RobotTask(void *argument);
/** @brief 舵机总线稳定后初始化所选模式；@param 无；@retval 无。 */
void RobotControl_Init(void);
/* Explicit new run; robot-task caller only. Idle, completed stand and no queued
 * shape required. Keeps configured startup authorization, never interrupts motion. */
uint8_t RobotControl_BeginNewRun(void);
/** @brief 非阻塞推进当前模式；@param elapsedMs 本次推进毫秒数；@retval 无。 */
void RobotControl_Update(uint16_t elapsedMs);
/** @brief 获取一致状态快照；@param 无；@retval 机器人状态副本。 */
RobotControlStatus_t RobotControl_GetStatus(void);
/** @brief 返回运行模式名称；@param 无；@retval 静态字符串。 */
const char *RobotControl_GetModeName(void);
/** @brief 返回状态名称；@param state 机器人状态；@retval 静态字符串。 */
const char *RobotControl_GetStateName(RobotControlState_t state);
/** @brief 返回视觉调度阶段名称；@param phase 阶段；@retval 静态字符串。 */
const char *RobotControl_GetWalkPhaseName(RobotWalkPhase_t phase);

#ifdef __cplusplus
}
#endif
#endif /* ROBOT_CONTROL_H */
