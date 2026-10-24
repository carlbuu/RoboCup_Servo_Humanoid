#include "robot_walk_config.h"
#include "robot_walk_vision.h"
#if ROBOT_RUN_MODE != ROBOT_MODE_ACTION_DEBUG

#include "camera_walk_link.h"
#include "debug_uart.h"
#include "motion_player.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

typedef struct
{
    uint32_t sequence;
    uint8_t shape;
    uint8_t stepDone; /* 完整流程：本图形是否已完成额外完整直行周期及回站。 */
} VisionShapeEvent_t;

static RobotControlStatus_t s_work;
static RobotControlStatus_t s_snapshot;
static VisionShapeEvent_t s_pending[ROBOT_WALK_SHAPE_QUEUE_LENGTH];
static uint8_t s_pendingRead;
static uint8_t s_pendingCount;
#if ROBOT_RUN_MODE != ROBOT_MODE_LINE_DEBUG
static uint8_t s_activeShape;
#endif
static uint8_t s_activeCommand;
static uint8_t s_runRequested;
static uint8_t s_blocked;
static uint8_t s_startup;
#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN
static uint8_t s_waitLineAfterTurn;
static uint8_t s_twoStepsAfterRight;
static uint8_t s_shapeStepPaused;
#endif
static uint32_t s_holdMs;
#if (ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN) || (ROBOT_RUN_MODE == ROBOT_MODE_LINE_DEBUG)
static uint32_t s_measureTick;
#endif
static uint32_t s_measureElapsed;
static CameraWalkRequest_t s_decision;
static uint8_t s_decisionValid;
#if ROBOT_WALK_LATE_ACTIVE
static uint16_t s_batchCycleBase;
static uint32_t s_batchId;
static uint32_t s_windowTick;
static uint8_t s_windowOpen;
static uint8_t s_stopRequested;
static CameraWalkRequest_t s_votes[ROBOT_WALK_VOTE_WINDOW];
static uint8_t s_voteCount;
static uint8_t s_voteWrite;
static CameraWalkRequest_t s_voteWinner;
static uint8_t s_voteValid;
static CameraWalkRequest_t s_afterReturn;
static uint8_t s_afterReturnValid;
static uint32_t s_returnReviewTick;
typedef struct
{
    CameraWalkRequest_t samples[ROBOT_WALK_VOTE_WINDOW];
    CameraWalkRequest_t winner;
    uint8_t count;
    uint8_t write;
    uint8_t valid;
} VisionHandoffVote_t;
static VisionHandoffVote_t s_rightVotes;
static uint32_t s_rightWindowTick;
static uint32_t s_rightDeadline;
static uint8_t s_rightWindowOpen;
static CameraWalkRequest_t s_handoffRequest;
static uint32_t s_handoffReviewTick;
static uint32_t s_handoffDeadline;
static uint8_t s_leftApproach;
static void Vision_LateResetVotes(void);
static void Vision_LateSyncWindow(void);
static uint8_t Vision_LateFinishBatch(void);
static void Vision_HandoffReset(void);
static void Vision_HandoffCancelApproach(void);
static uint8_t Vision_HandoffPoseMatches(uint8_t command);
static void Vision_HandoffBoundary(void);
static uint8_t Vision_StartLeftApproach(const CameraWalkRequest_t *request);
#endif

static uint8_t Vision_CurrentFrameIsStand(void);

static MotionId_t Vision_WalkMotion(void)
{
    return s_work.gaitStage == ROBOT_GAIT_FAST ? motion_walk_forward_fast : motion_walk_forward;
}

static void Vision_FrameCompleted(MotionId_t motion, uint8_t frame)
{
    /* Context matters: shape extra cycles and standalone actions are excluded. */
    if (MotionPlayer_IsForwardMotion(motion) == 0U ||
        s_activeCommand != CAMERA_WALK_FORWARD ||
        (s_work.walkPhase != ROBOT_WALK_MOVING &&
         s_work.walkPhase != ROBOT_WALK_TURN_APPROACH) ||
        (frame != ROBOT_WALK_RIGHT_STEP_FRAME && frame != ROBOT_WALK_LEFT_STEP_FRAME))
    { return; }
    if (s_work.forwardSteps != UINT32_MAX) { s_work.forwardSteps++; }
    Debug_Printf("[STEP] gait=%s foot=%s frame=%u total=%lu pending=%u\r\n",
        s_work.gaitStage == ROBOT_GAIT_FAST ? "fast" : "slow",
        frame == ROBOT_WALK_RIGHT_STEP_FRAME ? "right" : "left", (unsigned int)frame,
        (unsigned long)s_work.forwardSteps, (unsigned int)s_work.fastSwitchPending);
    if (s_work.gaitStage == ROBOT_GAIT_SLOW && s_work.fastSwitchPending == 0U &&
        s_work.forwardSteps >= ROBOT_SLOW_STAGE_STEPS)
    {
        s_work.fastSwitchPending = 1U;
        Debug_Printf("[GAIT] threshold=%lu total=%lu switch=requested; finish support/priority tasks\r\n",
            (unsigned long)ROBOT_SLOW_STAGE_STEPS, (unsigned long)s_work.forwardSteps);
    }
}

static uint8_t Vision_TryFastStage(void)
{
    if (s_work.fastSwitchPending == 0U || s_work.gaitStage != ROBOT_GAIT_SLOW ||
        MotionPlayer_IsActive() != 0U || Vision_CurrentFrameIsStand() == 0U ||
        s_pendingCount != 0U || s_blocked != 0U)
    { return 0U; }
#if ROBOT_WALK_LATE_ACTIVE
    if (s_stopRequested != 0U || s_afterReturnValid != 0U) { return 0U; }
#endif
    s_work.gaitStage = ROBOT_GAIT_FAST;
    s_work.fastSwitchPending = 0U;
    s_work.fastSwitchSteps = s_work.forwardSteps;
    Debug_Printf("[GAIT] switch=done gait=fast total=%lu extra_support_steps=%lu stand=1\r\n",
        (unsigned long)s_work.forwardSteps,
        (unsigned long)(s_work.forwardSteps - ROBOT_SLOW_STAGE_STEPS));
    return 1U;
}

static void Vision_Publish(void)
{
    s_work.motion = MotionPlayer_GetMotion();
    s_work.frameIndex = MotionPlayer_GetFrameIndex();
    s_work.completedCycles = MotionPlayer_GetCompletedCycles();
    s_work.pendingShapes = s_pendingCount;
#if ROBOT_WALK_LATE_ACTIVE
    s_work.walkBatch = s_batchId;
    s_work.sampleWindowOpen = s_windowOpen;
    s_work.sampleCount = s_voteCount;
    s_work.stopRequested = s_stopRequested;
    s_work.rightSampleWindowOpen = s_rightWindowOpen;
    s_work.rightSampleCount = s_rightVotes.count;
#endif
    s_work.state = s_work.walkPhase == ROBOT_WALK_FAULT ? ROBOT_STATE_FAULT
        : ((s_work.walkPhase == ROBOT_WALK_MEASURING ||
            s_work.walkPhase == ROBOT_WALK_BLOCKED)
           ? ROBOT_STATE_WAIT_START : ROBOT_STATE_RUNNING);
    taskENTER_CRITICAL();
    s_snapshot = s_work;
    taskEXIT_CRITICAL();
}

static void Vision_Fault(void)
{
    s_work.walkPhase = ROBOT_WALK_FAULT;
    s_runRequested = 0U;
}

static uint8_t Vision_StartSingle(MotionId_t motion, RobotWalkPhase_t phase)
{
    const MotionGroup_t *group = MotionGroups_Get(motion);
    if (group == 0 || group->frameCount == 0U ||
        MotionPlayer_StartCycles(motion, 1U, 0U,
            (uint8_t)(group->frameCount - 1U),
            (uint8_t)(group->frameCount - 1U)) == 0U)
    {
        Vision_Fault();
        return 0U;
    }
    s_work.walkPhase = phase;
    return 1U;
}

static void Vision_StartWalk(void)
{
    uint16_t cycles;
    uint16_t steps = ROBOT_STM_WALK_BATCH_STEPS;
    (void)Vision_TryFastStage();
#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN
    /* 仅右转后的首个新FWD,2采用两步；普通行走仍使用原批次配置。 */
    if (s_twoStepsAfterRight != 0U && s_decision.command == CAMERA_WALK_FORWARD &&
        s_decision.arg == 2U) { steps = 2U; }
#endif
    if (steps == 0U || ROBOT_WALK_STEPS_PER_CYCLE == 0U ||
        steps > ROBOT_WALK_MAX_BATCH_STEPS ||
        steps % ROBOT_WALK_STEPS_PER_CYCLE != 0U)
    {
        Vision_Fault();
        return;
    }
    cycles = (uint16_t)(steps / ROBOT_WALK_STEPS_PER_CYCLE);
    if (MotionPlayer_StartCycles(Vision_WalkMotion(), cycles,
            ROBOT_WALK_LOOP_START_FRAME, ROBOT_WALK_CYCLE_END_FRAME,
            ROBOT_WALK_STOP_FRAME) == 0U)
    {
        Vision_Fault();
        return;
    }
#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN
    s_twoStepsAfterRight = 0U;
#endif
    Debug_Printf("[GAIT] walk/resume gait=%s total=%lu\r\n",
        s_work.gaitStage == ROBOT_GAIT_FAST ? "fast" : "slow", (unsigned long)s_work.forwardSteps);
    s_activeCommand = CAMERA_WALK_FORWARD;
    s_work.activeCommandSequence = s_decision.sequence;
    s_work.targetCycles = cycles;
    s_work.walkPhase = ROBOT_WALK_MOVING;
#if ROBOT_WALK_LATE_ACTIVE
    Vision_HandoffReset();
    s_batchCycleBase = MotionPlayer_GetCompletedCycles();
    s_batchId++;
    s_windowOpen = 0U;
    s_afterReturnValid = 0U;
    Vision_LateResetVotes();
    Debug_Printf("[WALK] batch=%lu start cycles=%u frame=0\r\n",
        (unsigned long)s_batchId, (unsigned int)cycles);
#endif
#if ROBOT_RUN_MODE == ROBOT_MODE_LINE_DEBUG
    Debug_Printf("[LINE] moving motion=%u cycles=%u gate=closed\r\n",
        (unsigned int)MotionPlayer_GetMotion(), (unsigned int)s_work.targetCycles);
#endif
}

#ifdef ROBOT_BACKWARD_MOTION
/** @brief 执行一次已验证后退步态；一周期必须定义为一步。 */
static void Vision_StartBack(const CameraWalkRequest_t *request)
{
    if (Vision_StartSingle(ROBOT_BACKWARD_MOTION, ROBOT_WALK_MOVING) != 0U)
    {
        s_activeCommand = CAMERA_WALK_BACK;
        s_work.activeCommandSequence = request->sequence;
        s_work.targetCycles = 1U;
        s_runRequested = 0U;
    }
}
#endif

static void Vision_StartTurn(const CameraWalkRequest_t *request)
{
    const MotionGroup_t *group;
    MotionId_t motion;
    const uint16_t cycles = 1U;
#if ROBOT_WALK_LATE_ACTIVE
    uint8_t sourcePhase = (uint8_t)s_work.walkPhase;
#endif
    motion = request->command == CAMERA_WALK_LEFT ? motion_turn_left : motion_turn_right;
    group = MotionGroups_Get(motion);
    if (group == 0 || group->frameCount == 0U ||
        MotionPlayer_StartCycles(motion, cycles, 0U,
            (uint8_t)(group->frameCount - 1U),
            (uint8_t)(group->frameCount - 1U)) == 0U)
    {
        Vision_Fault();
        return;
    }
    Debug_Printf("[GAIT] turn=full_from_stand gait=%s\r\n",
        s_work.gaitStage == ROBOT_GAIT_FAST ? "fast" : "slow");
    s_activeCommand = request->command;
    s_work.activeCommandSequence = request->sequence;
    s_work.targetCycles = cycles;
    s_work.walkPhase = ROBOT_WALK_MOVING;
    s_work.turnFromWalk = 0U;
#if ROBOT_WALK_LATE_ACTIVE
    Vision_HandoffReset();
    s_decisionValid = 0U;
    Vision_LateResetVotes();
    CameraWalk_ClearMotion(); /* 独立HOLD/BLOCKED不会被清除。 */
    Debug_Printf("[TURN] start cmd=%u seq=%lu rx=%lu first=%lu cycles=1 source_phase=%u\r\n",
        (unsigned int)request->command, (unsigned long)request->sequence,
        (unsigned long)request->receivedTick, (unsigned long)request->firstReceivedTick,
        (unsigned int)sourcePhase);
#endif
#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN
    s_twoStepsAfterRight = (uint8_t)(request->command == CAMERA_WALK_RIGHT);
    /* 右转后可请求两步；排队图形仍在站稳后优先执行。 */
    s_waitLineAfterTurn = 1U;
#endif
#if ROBOT_RUN_MODE == ROBOT_MODE_LINE_DEBUG
    Debug_Printf("[LINE] moving motion=%u cycles=%u gate=closed\r\n",
        (unsigned int)MotionPlayer_GetMotion(), (unsigned int)s_work.targetCycles);
#endif
}

#if ROBOT_WALK_LATE_ACTIVE
static uint8_t Vision_IsDirection(uint8_t command)
{
    return (uint8_t)(command == CAMERA_WALK_FORWARD || command == CAMERA_WALK_LEFT ||
        command == CAMERA_WALK_RIGHT || command == CAMERA_WALK_SHIFT_LEFT ||
        command == CAMERA_WALK_SHIFT_RIGHT);
}

static void Vision_LateResetVotes(void)
{
    s_voteCount = s_voteWrite = s_voteValid = 0U;
}

static void Vision_HandoffReset(void)
{
    memset(&s_rightVotes, 0, sizeof(s_rightVotes));
    s_rightWindowOpen = s_leftApproach = 0U;
    s_work.pendingTurn = CAMERA_WALK_HOLD;
    MotionPlayer_CancelWalkHandoff();
}

static void Vision_HandoffCancelApproach(void)
{
    if (s_leftApproach != 0U)
    {
        Debug_Printf("[HANDOFF] cancel LEFT seq=%lu; keep walk support path\r\n",
            (unsigned long)s_handoffRequest.sequence);
        s_leftApproach = 0U;
        s_work.walkPhase = ROBOT_WALK_MOVING;
        Vision_LateResetVotes();
    }
    s_work.pendingTurn = CAMERA_WALK_HOLD;
    s_rightWindowOpen = 0U;
    memset(&s_rightVotes, 0, sizeof(s_rightVotes));
    MotionPlayer_CancelWalkHandoff();
}

static uint32_t Vision_FrameRangeMs(uint8_t first, uint8_t last)
{
    const MotionGroup_t *walk = MotionGroups_Get(Vision_WalkMotion());
    uint32_t result = 0U;
    uint8_t frame;
    for (frame = first; frame <= last; ++frame)
    {
        uint32_t ms = walk->frames[frame].durationMs;
        if (ms == 0U) { ms = 1U; }
        result += ((ms + ROBOT_CONTROL_PERIOD_MS - 1U) / ROBOT_CONTROL_PERIOD_MS)
            * ROBOT_CONTROL_PERIOD_MS;
    }
    return result;
}

/* 腿和头必须与原转弯跨立一致；双臂由当前角度随原回站帧连续回零。 */
static uint8_t Vision_HandoffPoseMatches(uint8_t command)
{
    const MotionGroup_t *walk = MotionGroups_Get(Vision_WalkMotion());
    const MotionGroup_t *turn = MotionGroups_Get(command == CAMERA_WALK_RIGHT
        ? motion_turn_right : motion_turn_left);
    const MotionGroup_t *stand = MotionGroups_Get(motion_stand);
    uint8_t frame = command == CAMERA_WALK_RIGHT ? ROBOT_WALK_RIGHT_HANDOFF_FRAME
                                               : ROBOT_WALK_LEFT_HANDOFF_FRAME;
    uint8_t joint;
    if (s_work.gaitStage != ROBOT_GAIT_SLOW ||
        MotionPlayer_GetMotion() != motion_walk_forward ||
        walk->frameCount <= frame || turn->frameCount < 2U ||
        (uint16_t)ROBOT_WALK_STOP_FRAME + 1U >= walk->frameCount)
    { return 0U; }
    for (joint = 0U; joint < ROBOT_JOINT_COUNT; ++joint)
    {
        if ((joint < 10U || joint == 12U) &&
            walk->frames[frame].angleDeg10[joint] !=
                turn->frames[turn->frameCount - 2U].angleDeg10[joint])
        { return 0U; }
        if (turn->frames[turn->frameCount - 1U].angleDeg10[joint] !=
            stand->frames[0].angleDeg10[joint])
        { return 0U; }
    }
    return 1U;
}

static void Vision_RightAddSample(const CameraWalkRequest_t *request)
{
    uint8_t i;
    if (s_rightVotes.count != 0U &&
        s_rightVotes.samples[(s_rightVotes.write + ROBOT_WALK_VOTE_WINDOW - 1U) %
            ROBOT_WALK_VOTE_WINDOW].sequence == request->sequence) { return; }
    s_rightVotes.samples[s_rightVotes.write] = *request;
    s_rightVotes.write = (uint8_t)((s_rightVotes.write + 1U) % ROBOT_WALK_VOTE_WINDOW);
    if (s_rightVotes.count < ROBOT_WALK_VOTE_WINDOW) { s_rightVotes.count++; }
    /* 保留2/2确认，最新一票也必须同向；没有重复发送/缓存补票。 */
    s_rightVotes.valid = (uint8_t)(s_rightVotes.count >= ROBOT_WALK_VOTE_NEED &&
        request->command == CAMERA_WALK_RIGHT);
    for (i = 0U; i < s_rightVotes.count; ++i)
    {
        if (s_rightVotes.samples[i].command != CAMERA_WALK_RIGHT)
        { s_rightVotes.valid = 0U; }
    }
    s_rightVotes.winner = *request;
    s_work.pendingTurn = s_rightVotes.valid != 0U ? CAMERA_WALK_RIGHT : CAMERA_WALK_HOLD;
    Debug_Printf("[HANDOFF_VOTE] RIGHT count=%u valid=%u seq=%lu rx=%lu\r\n",
        (unsigned int)s_rightVotes.count, (unsigned int)s_rightVotes.valid,
        (unsigned long)request->sequence, (unsigned long)request->receivedTick);
}

static void Vision_StartWalkTurn(const CameraWalkRequest_t *request)
{
    CameraWalkRequest_t command = *request;
    MotionId_t motion = command.command == CAMERA_WALK_RIGHT ? motion_turn_right : motion_turn_left;
    const MotionGroup_t *group = MotionGroups_Get(motion);
    uint8_t exitFrame = MotionPlayer_GetFrameIndex();
    /* Defense in depth: a fast gait must never execute only a turn tail. */
    if (s_work.gaitStage != ROBOT_GAIT_SLOW || MotionPlayer_GetMotion() != motion_walk_forward)
    { Vision_Fault(); return; }
    s_work.lastWalkCompletedCycles =
        (uint16_t)(MotionPlayer_GetCompletedCycles() - s_batchCycleBase);
    s_work.lastWalkExitFrame = exitFrame;
    s_work.lastWalkPartialSteps = command.command == CAMERA_WALK_RIGHT ? 2U : 1U;
    s_work.lastInterruptedCommandSequence = s_work.activeCommandSequence;
    if (MotionPlayer_StartRange(motion, (uint8_t)(group->frameCount - 1U),
            (uint8_t)(group->frameCount - 1U)) == 0U)
    { Vision_Fault(); return; }
    Vision_HandoffReset();
    Vision_LateResetVotes();
    s_afterReturnValid = s_decisionValid = 0U;
    s_runRequested = 0U;
    s_activeCommand = command.command;
    s_work.activeCommandSequence = command.sequence;
    s_work.targetCycles = 1U;
    s_work.turnFromWalk = 1U;
    s_work.walkPhase = ROBOT_WALK_TURN_HANDOFF;
    CameraWalk_ClearMotion();
#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN
    s_waitLineAfterTurn = 1U;
    s_twoStepsAfterRight = (uint8_t)(command.command == CAMERA_WALK_RIGHT);
#endif
    Debug_Printf("[HANDOFF] start cmd=%u seq=%lu batch=%lu full_cycles=%u exit=%u partial_steps=%u tail=%u\r\n",
        (unsigned int)command.command, (unsigned long)command.sequence,
        (unsigned long)s_batchId, (unsigned int)s_work.lastWalkCompletedCycles,
        (unsigned int)exitFrame, (unsigned int)s_work.lastWalkPartialSteps,
        (unsigned int)(group->frameCount - 1U));
}

static uint8_t Vision_StartLeftApproach(const CameraWalkRequest_t *request)
{
    uint16_t completed = (uint16_t)(MotionPlayer_GetCompletedCycles() - s_batchCycleBase);
    if (s_work.gaitStage != ROBOT_GAIT_SLOW || MotionPlayer_GetMotion() != motion_walk_forward)
    { return 0U; }
    s_handoffRequest = *request;
    Vision_HandoffReset();
    if (MotionPlayer_ContinueWalkCycles(1U) == 0U ||
        MotionPlayer_ArmWalkHandoff(ROBOT_WALK_LEFT_HANDOFF_FRAME) == 0U)
    { Vision_Fault(); return 1U; }
    s_batchCycleBase = (uint16_t)(MotionPlayer_GetCompletedCycles() - completed);
    s_leftApproach = 1U;
    s_work.pendingTurn = CAMERA_WALK_LEFT;
    s_work.walkPhase = ROBOT_WALK_TURN_APPROACH;
    s_handoffReviewTick = (uint32_t)xTaskGetTickCount();
    s_handoffDeadline = s_handoffReviewTick +
        Vision_FrameRangeMs((uint8_t)(ROBOT_WALK_STOP_FRAME + 1U),
            (uint8_t)(MotionGroups_Get(motion_walk_forward)->frameCount - 1U)) +
        Vision_FrameRangeMs(ROBOT_WALK_LOOP_START_FRAME, ROBOT_WALK_LEFT_HANDOFF_FRAME) +
        ROBOT_WALK_HANDOFF_DELAY_BUDGET_MS;
    s_runRequested = 0U;
    s_windowOpen = s_afterReturnValid = s_decisionValid = 0U;
    Vision_LateResetVotes();
    CameraWalk_ClearMotion();
    Debug_Printf("[HANDOFF] LEFT approach seq=%lu path=12->4->5->6; FWD recheck open\r\n",
        (unsigned long)s_handoffRequest.sequence);
    return 1U;
}

static void Vision_HandoffBoundary(void)
{
    uint8_t frame = MotionPlayer_GetFrameIndex();
    if (s_work.gaitStage == ROBOT_GAIT_SLOW && s_stopRequested == 0U &&
        s_blocked == 0U && s_pendingCount == 0U)
    {
        if (s_leftApproach != 0U && frame == ROBOT_WALK_LEFT_HANDOFF_FRAME &&
            (int32_t)((uint32_t)xTaskGetTickCount() - s_handoffDeadline) <= 0)
        { Vision_StartWalkTurn(&s_handoffRequest); return; }
        if (s_work.walkPhase == ROBOT_WALK_MOVING &&
            frame == ROBOT_WALK_RIGHT_HANDOFF_FRAME && s_rightWindowOpen != 0U &&
            s_rightVotes.valid != 0U &&
            (int32_t)((uint32_t)xTaskGetTickCount() - s_rightDeadline) <= 0)
        { Vision_StartWalkTurn(&s_rightVotes.winner); return; }
    }
    /* 错过/取消时只向原下一帧续播，绝不从第12帧回拉到跨立。 */
    if (s_rightWindowOpen != 0U &&
        (int32_t)((uint32_t)xTaskGetTickCount() - s_rightDeadline) > 0)
    { Vision_LateResetVotes(); } /* 提交重试不能把过期方向带入回退路径。 */
    Vision_HandoffCancelApproach();
    if (MotionPlayer_ResumeWalkHandoff() == 0U) { Vision_Fault(); }
}

/* 只对独立接收帧投票，不把重复读取一份缓存当作新样本。最新票须同向。 */
static void Vision_LateAddSample(const CameraWalkRequest_t *request)
{
    uint8_t counts[8] = { 0U };
    uint8_t i, winner = 0U, maximum = 0U, tied = 0U;
    if (s_voteCount != 0U &&
        s_votes[(s_voteWrite + ROBOT_WALK_VOTE_WINDOW - 1U) % ROBOT_WALK_VOTE_WINDOW].sequence
        == request->sequence) { return; }
    s_votes[s_voteWrite] = *request;
    s_voteWrite = (uint8_t)((s_voteWrite + 1U) % ROBOT_WALK_VOTE_WINDOW);
    if (s_voteCount < ROBOT_WALK_VOTE_WINDOW) { s_voteCount++; }
    for (i = 0U; i < s_voteCount; ++i) { counts[s_votes[i].command]++; }
    for (i = 0U; i < 8U; ++i)
    {
        if (counts[i] > maximum) { maximum = counts[i]; winner = i; tied = 0U; }
        else if (counts[i] == maximum && maximum != 0U) { tied = 1U; }
    }
    s_voteValid = (uint8_t)(maximum >= ROBOT_WALK_VOTE_NEED && tied == 0U &&
        request->command == winner);
    if (s_voteValid != 0U) { s_voteWinner = *request; }
    Debug_Printf("[VOTE] batch=%lu count=%u latest=%u winner=%u votes=%u valid=%u seq=%lu rx=%lu\r\n",
        (unsigned long)s_batchId, (unsigned int)s_voteCount, (unsigned int)request->command,
        (unsigned int)winner, (unsigned int)maximum, (unsigned int)s_voteValid,
        (unsigned long)request->sequence, (unsigned long)request->receivedTick);
}

static void Vision_LateSyncWindow(void)
{
    uint16_t completed;
    uint8_t frame;
    if (s_work.walkPhase != ROBOT_WALK_MOVING || s_activeCommand != CAMERA_WALK_FORWARD ||
        MotionPlayer_IsActive() == 0U || s_stopRequested != 0U || s_pendingCount != 0U)
    { return; }
    completed = (uint16_t)(MotionPlayer_GetCompletedCycles() - s_batchCycleBase);
    frame = MotionPlayer_GetFrameIndex();
    if (s_work.gaitStage == ROBOT_GAIT_SLOW &&
        ROBOT_WALK_TURN_HANDOFF_ENABLE != 0U && s_rightWindowOpen == 0U &&
        (ROBOT_WALK_DECIDE_EACH_CYCLE != 0U || completed + 1U == s_work.targetCycles) &&
        frame >= ROBOT_WALK_RIGHT_SAMPLE_START_FRAME &&
        frame <= ROBOT_WALK_RIGHT_HANDOFF_FRAME &&
        Vision_HandoffPoseMatches(CAMERA_WALK_RIGHT) != 0U &&
        MotionPlayer_ArmWalkHandoff(ROBOT_WALK_RIGHT_HANDOFF_FRAME) != 0U)
    {
        memset(&s_rightVotes, 0, sizeof(s_rightVotes));
        s_rightWindowTick = (uint32_t)xTaskGetTickCount();
        s_rightDeadline = s_rightWindowTick +
            Vision_FrameRangeMs(ROBOT_WALK_RIGHT_SAMPLE_START_FRAME, ROBOT_WALK_RIGHT_HANDOFF_FRAME) +
            ROBOT_WALK_HANDOFF_DELAY_BUDGET_MS;
        s_rightWindowOpen = 1U;
        Debug_Printf("[HANDOFF_WINDOW] RIGHT open tick=%lu frame=%u\r\n",
            (unsigned long)s_rightWindowTick, (unsigned int)frame);
    }
    /* 每周期独立开窗；缩短播放器到本周期帧11，不打断任何支撑帧。
     * 直行边界随后从12->4续播；逻辑批次仍保持原4步及右转后2步特例。 */
    if (s_windowOpen == 0U &&
        (ROBOT_WALK_DECIDE_EACH_CYCLE != 0U || completed + 1U == s_work.targetCycles) &&
        frame >= ROBOT_WALK_SAMPLE_START_FRAME && frame <= ROBOT_WALK_SAMPLE_END_FRAME)
    {
        if (ROBOT_WALK_DECIDE_EACH_CYCLE != 0U &&
            MotionPlayer_RequestStopAtCycleEnd() == 0U) { Vision_Fault(); return; }
        Vision_LateResetVotes();
        s_windowTick = (uint32_t)xTaskGetTickCount();
        s_windowOpen = 1U;
        Debug_Printf("[WINDOW] batch=%lu cycle=%u open tick=%lu frame=%u\r\n",
            (unsigned long)s_batchId, (unsigned int)(completed + 1U),
            (unsigned long)s_windowTick, (unsigned int)frame);
    }
}

static void Vision_StartShift(const CameraWalkRequest_t *request)
{
    MotionId_t motion = request->command == CAMERA_WALK_SHIFT_LEFT
        ? motion_left_translation : motion_right_translation;
    if (Vision_StartSingle(motion, ROBOT_WALK_MOVING) == 0U) { return; }
    Debug_Printf("[GAIT] shift=from_stand gait=%s\r\n",
        s_work.gaitStage == ROBOT_GAIT_FAST ? "fast" : "slow");
    s_activeCommand = request->command;
    s_work.activeCommandSequence = request->sequence;
    s_work.targetCycles = 1U;
    s_runRequested = 0U;
#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN
    s_waitLineAfterTurn = 1U;
    s_twoStepsAfterRight = 0U;
#endif
    Debug_Printf("[CORRECT] shift=%u cycles=1 seq=%lu\r\n",
        (unsigned int)request->command, (unsigned long)request->sequence);
}

/* 返回1已续走/故障；0由原流程执行收脚。普通方向不提前打断支撑动作。 */
static uint8_t Vision_LateFinishBatch(void)
{
    uint16_t cycles = (uint16_t)(ROBOT_STM_WALK_BATCH_STEPS / ROBOT_WALK_STEPS_PER_CYCLE);
    uint16_t completed = (uint16_t)(MotionPlayer_GetCompletedCycles() - s_batchCycleBase);
    uint8_t batchDone = (uint8_t)(completed >= s_work.targetCycles);
    uint8_t command = s_voteValid != 0U ? s_voteWinner.command : CAMERA_WALK_FORWARD;
    s_windowOpen = 0U;
    s_afterReturnValid = 0U;
    Debug_Printf("[BOUNDARY] batch=%lu completed=%u adopt=%u valid=%u stop=%u shapes=%u cycle=%u tick=%lu\r\n",
        (unsigned long)s_batchId, (unsigned int)MotionPlayer_GetCompletedCycles(),
        (unsigned int)command, (unsigned int)s_voteValid, (unsigned int)s_stopRequested,
        (unsigned int)s_pendingCount, (unsigned int)completed,
        (unsigned long)xTaskGetTickCount());
    if (s_stopRequested != 0U) { return 0U; }
    if (s_pendingCount != 0U &&
        (ROBOT_SHAPE_STEP_BEFORE_ACTION == 0U || batchDone != 0U)) { return 0U; }
    if (s_voteValid != 0U && command != CAMERA_WALK_FORWARD)
    {
        if (s_work.gaitStage == ROBOT_GAIT_SLOW &&
            ROBOT_WALK_TURN_HANDOFF_ENABLE != 0U && command == CAMERA_WALK_LEFT &&
            s_pendingCount == 0U && Vision_HandoffPoseMatches(CAMERA_WALK_LEFT) != 0U)
        { return Vision_StartLeftApproach(&s_voteWinner); }
        s_afterReturn = s_voteWinner;
        s_afterReturnValid = 1U;
        s_returnReviewTick = (uint32_t)xTaskGetTickCount();
        s_runRequested = 0U;
        Debug_Printf("[BOUNDARY] return_then_correct cmd=%u seq=%lu\r\n",
            (unsigned int)command, (unsigned long)s_afterReturn.sequence);
        Vision_LateResetVotes(); /* 收脚复核只能使用边界之后的新FWD。 */
        return 0U;
    }
    /* A pending transition yields to STOP/shapes/confirmed correction above.
     * Otherwise finish frame 11, then use the original walk-to-stand table. */
    if (s_work.fastSwitchPending != 0U) { return 0U; }
    if (s_runRequested == 0U && s_pendingCount == 0U) { return 0U; }
    /* 无有效结果仍按已有运行授权续走，不添加通讯超时停机。 */
    if (s_voteValid != 0U) { s_decision = s_voteWinner; }
    if (MotionPlayer_ContinueWalkCycles(batchDone != 0U ? cycles :
            (uint16_t)(s_work.targetCycles - completed)) == 0U)
    { Vision_Fault(); return 1U; }
    if (s_work.activeCommandSequence != 0U)
    { s_work.lastCompletedCommandSequence = s_work.activeCommandSequence; }
    s_work.activeCommandSequence = s_voteValid != 0U ? s_voteWinner.sequence : 0U;
    if (batchDone != 0U)
    {
        s_batchCycleBase = MotionPlayer_GetCompletedCycles();
        s_work.targetCycles = cycles;
        s_batchId++;
    }
    else
    {
        /* Continue在累计数饱和时会重置计数；保留本批相对周期数。 */
        s_batchCycleBase = (uint16_t)(MotionPlayer_GetCompletedCycles() - completed);
    }
    Debug_Printf("[WALK] batch=%lu continue frame=%u return=0 stand=0 fallback=%u cycle_done=%u batch_done=%u\r\n",
        (unsigned long)s_batchId, (unsigned int)MotionPlayer_GetFrameIndex(),
        (unsigned int)(s_voteValid == 0U), (unsigned int)completed, (unsigned int)batchDone);
    Vision_LateResetVotes();
    return 1U;
}
#endif

/**
 * @brief 判断已完成帧的目标姿态是否与站立末帧一致。
 * @param 无。
 * @retval 1：无需重复回站；0：仍需执行回站动作。
 * @note 仅比较指令目标，不代表舵机实际到位；回位时间由动作组承担。
 */
static uint8_t Vision_CurrentFrameIsStand(void)
{
    const MotionGroup_t *group = MotionGroups_Get(MotionPlayer_GetMotion());
    const MotionGroup_t *stand = MotionGroups_Get(motion_stand);
    uint8_t frame = MotionPlayer_GetFrameIndex();
    uint8_t joint;
    if (group == NULL || group->frames == NULL || frame >= group->frameCount ||
        stand == NULL || stand->frames == NULL || stand->frameCount == 0U)
    {
        return 0U;
    }
    for (joint = 0U; joint < ROBOT_JOINT_COUNT; ++joint)
    {
        if (group->frames[frame].angleDeg10[joint] !=
            stand->frames[stand->frameCount - 1U].angleDeg10[joint])
        {
            return 0U;
        }
    }
    return 1U;
}

static void Vision_QueueShape(const CameraWalkRequest_t *request)
{
#if ROBOT_RUN_MODE == ROBOT_MODE_LINE_DEBUG
    (void)request; /* 巡线联调消费图形帧，但不排队、不打断步态。 */
#else
    uint8_t i;
    uint8_t slot;
    if (s_work.walkPhase == ROBOT_WALK_FAULT) { return; }
    /* 原协议没有图卡序号：只合并仍在执行/等待中的同类，不永久锁定。 */
    if (request->shape == s_activeShape) { return; }
    for (i = 0U; i < s_pendingCount; ++i)
    {
        if (s_pending[(s_pendingRead + i) % ROBOT_WALK_SHAPE_QUEUE_LENGTH].shape
            == request->shape) { return; }
    }
    if (s_pendingCount >= ROBOT_WALK_SHAPE_QUEUE_LENGTH)
    {
        s_work.eventsDropped++;
        return;
    }
    slot = (uint8_t)((s_pendingRead + s_pendingCount) % ROBOT_WALK_SHAPE_QUEUE_LENGTH);
    s_pending[slot].sequence = request->sequence;
    s_pending[slot].shape = request->shape;
    s_pending[slot].stepDone = 0U;
    s_pendingCount++;
    s_decisionValid = 0U; /* 图形排队后旧转向/前进建议失效。 */
#if !ROBOT_WALK_LATE_ACTIVE
    if (s_work.walkPhase == ROBOT_WALK_MOVING &&
        s_activeCommand == CAMERA_WALK_FORWARD)
    {
        (void)MotionPlayer_RequestStopAtCycleEnd();
    }
#else
    /* 图形独立缓存。新默认流程在当前安全周期结束收脚，不额外多走一周期。 */
    s_windowOpen = 0U;
    s_afterReturnValid = 0U;
    Vision_HandoffCancelApproach();
    Vision_LateResetVotes();
#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN
    if (ROBOT_SHAPE_STEP_BEFORE_ACTION == 0U &&
        s_work.walkPhase == ROBOT_WALK_MOVING && s_activeCommand == CAMERA_WALK_FORWARD)
    { (void)MotionPlayer_RequestStopAtCycleEnd(); }
    Debug_Printf("[SHAPE] queued id=%u seq=%lu phase=%u extra=%u pending=%u\r\n",
        (unsigned int)request->shape, (unsigned long)request->sequence,
        (unsigned int)s_work.walkPhase, (unsigned int)ROBOT_SHAPE_STEP_BEFORE_ACTION,
        (unsigned int)s_pendingCount);
#endif
#endif
#endif
}

#if ROBOT_WALK_LATE_ACTIVE
static void Vision_ReadCommand(const CameraWalkRequest_t *request)
{
    if (s_work.walkPhase == ROBOT_WALK_FAULT) { return; }
    if (request->command == CAMERA_WALK_HOLD || request->command == CAMERA_WALK_BLOCKED)
    {
        s_stopRequested = 1U;
        if (request->command == CAMERA_WALK_BLOCKED) { s_blocked = 1U; }
        s_runRequested = 0U;
        s_decisionValid = s_afterReturnValid = s_windowOpen = 0U;
        Vision_HandoffCancelApproach();
        Vision_LateResetVotes();
        if (s_work.walkPhase == ROBOT_WALK_MEASURING || s_work.walkPhase == ROBOT_WALK_BLOCKED)
        { s_measureTick = (uint32_t)xTaskGetTickCount(); }
#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN
        s_shapeStepPaused = s_waitLineAfterTurn = 1U;
#endif
        if ((s_work.walkPhase == ROBOT_WALK_MOVING && s_activeCommand == CAMERA_WALK_FORWARD) ||
            s_work.walkPhase == ROBOT_WALK_SHAPE_STEP)
        { (void)MotionPlayer_RequestStopAtCycleEnd(); }
        Debug_Printf("[STOP] cmd=%u seq=%lu phase=%u request=cycle_end\r\n",
            (unsigned int)request->command, (unsigned long)request->sequence,
            (unsigned int)s_work.walkPhase);
        return;
    }
    if (Vision_IsDirection(request->command) == 0U) { return; }
    if (s_leftApproach != 0U)
    {
        /* 与原收脚复核相同：只允许两条边界之后的新FWD取消，不用新TURN覆盖。 */
        if ((int32_t)(request->firstReceivedTick - s_handoffReviewTick) > 0 &&
            s_stopRequested == 0U && s_pendingCount == 0U)
        {
            if (request->command == CAMERA_WALK_FORWARD)
            {
                Vision_LateAddSample(request);
                if (s_voteValid != 0U && s_voteWinner.command == CAMERA_WALK_FORWARD)
                {
                    s_decision = s_voteWinner;
                    s_runRequested = 1U;
                    Vision_HandoffCancelApproach();
                }
            }
            else { Vision_LateResetVotes(); }
        }
        return;
    }
    if (s_work.walkPhase == ROBOT_WALK_MOVING && s_activeCommand == CAMERA_WALK_FORWARD &&
        s_rightWindowOpen != 0U && s_stopRequested == 0U && s_pendingCount == 0U &&
        (int32_t)(request->firstReceivedTick - s_rightWindowTick) > 0)
    { Vision_RightAddSample(request); }
    /* 普通纠偏只在完整周期末选取。收脚期间只允许连续新FWD取消待执行
     * 的纠偏，不接受新的TURN/SHIFT覆盖它，也不额外等待站稳后重收一次。 */
    if (s_work.walkPhase == ROBOT_WALK_TO_STAND && s_afterReturnValid != 0U &&
        s_stopRequested == 0U &&
        (int32_t)(request->firstReceivedTick - s_returnReviewTick) > 0)
    {
        if (request->command == CAMERA_WALK_FORWARD)
        {
            Vision_LateAddSample(request);
            Debug_Printf("[RECHECK] seq=%lu FWD valid=%u\r\n",
                (unsigned long)request->sequence, (unsigned int)s_voteValid);
        }
        else { Vision_LateResetVotes(); }
        return;
    }
    if (s_work.walkPhase == ROBOT_WALK_MOVING && s_activeCommand == CAMERA_WALK_FORWARD)
    {
        if (s_windowOpen == 0U || s_stopRequested != 0U || s_pendingCount != 0U ||
            (int32_t)(request->firstReceivedTick - s_windowTick) <= 0) { return; }
        Debug_Printf("[SAMPLE] batch=%lu cycle=%u frame=%u seq=%lu rx=%lu cmd=%u\r\n",
            (unsigned long)s_batchId,
            (unsigned int)(MotionPlayer_GetCompletedCycles() - s_batchCycleBase + 1U),
            (unsigned int)MotionPlayer_GetFrameIndex(), (unsigned long)request->sequence,
            (unsigned long)request->receivedTick, (unsigned int)request->command);
        Vision_LateAddSample(request);
        return;
    }
    /* 起步/纠偏/图形结束后的新姿态使用新测量；普通直行边界不经过此门控。 */
    if ((s_work.walkPhase == ROBOT_WALK_MEASURING || s_work.walkPhase == ROBOT_WALK_BLOCKED) &&
        (int32_t)(request->firstReceivedTick - s_measureTick) > 0)
    {
        Vision_LateAddSample(request);
        /* 先前通过的决定必须随最新票同步失效。所有本批请求处理完以后，
         * 再在Update提交恢复/动作，避免中间胜出票提前清掉停止锁存。 */
        s_decisionValid = s_voteValid;
        if (s_decisionValid != 0U)
        {
            s_decision = s_voteWinner;
        }
    }
}
#else
static void Vision_ReadCommand(const CameraWalkRequest_t *request)
{
    /* 图形联调只消费CMD，不让巡线建议启动、转向或锁住图形调试。 */
#if ROBOT_RUN_MODE == ROBOT_MODE_VISION_DEBUG
    (void)request;
#else
    if (s_work.walkPhase == ROBOT_WALK_FAULT) { return; }
#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN
    /* 暂停/纠偏时先不启动图形前的额外直行周期，新的FWD解除该条件。 */
    s_shapeStepPaused = (uint8_t)(request->command != CAMERA_WALK_FORWARD);
#endif
    if (s_blocked != 0U)
    {
#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN
        /* BLOCKED只暂停到安全站稳；新的方向可继续单次找线。 */
        if (s_work.walkPhase == ROBOT_WALK_BLOCKED &&
            (int32_t)(request->receivedTick - s_measureTick) > 0 &&
            (request->command == CAMERA_WALK_FORWARD ||
             request->command == CAMERA_WALK_LEFT ||
             request->command == CAMERA_WALK_RIGHT))
        {
            s_blocked = 0U;
            s_runRequested = (uint8_t)(request->command == CAMERA_WALK_FORWARD);
            s_work.walkPhase = ROBOT_WALK_MEASURING;
            s_decision = *request;
            s_decisionValid = 1U;
            Debug_Printf("[VISION] resume fresh_cmd=%u arg=%u\r\n",
                (unsigned int)request->command, (unsigned int)request->arg);
        }
#endif
        return;
    }
#if ROBOT_RUN_MODE == ROBOT_MODE_LINE_DEBUG
    /* 持续接收并消费串口，但运动期间的所有CMD都不参与决策。
     * 包括HOLD/BLOCKED；本站稳时间之前的旧帧也必须丢弃。 */
    if (s_work.walkPhase != ROBOT_WALK_MEASURING ||
        (int32_t)(request->receivedTick - s_measureTick) <= 0) { return; }
    {
        static const char * const names[] = { "HOLD", "FWD", "TURNL", "TURNR", "BLOCKED" };
        Debug_Printf("[LINE] accept=%s arg=%u\r\n",
            request->command <= CAMERA_WALK_BLOCKED ? names[request->command] : "unknown",
            (unsigned int)request->arg);
    }
#endif
    if (request->command == CAMERA_WALK_BLOCKED)
    {
        s_blocked = 1U;
        s_runRequested = 0U;
#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN
        s_waitLineAfterTurn = 1U; /* 暂停自动续走，排队图形仍在站稳后优先。 */
        Debug_Printf("[VISION] pause BLOCKED: stand then accept fresh direction\r\n");
#endif
    }
    else if (request->command == CAMERA_WALK_FORWARD)
    {
        s_runRequested = 1U;
    }
    else
    {
        s_runRequested = 0U;
    }
    if (s_work.walkPhase == ROBOT_WALK_MOVING &&
        s_activeCommand == CAMERA_WALK_FORWARD &&
        request->command != CAMERA_WALK_FORWARD)
    {
        (void)MotionPlayer_RequestStopAtCycleEnd();
    }
    /* 行走/转向/图形动作期间的建议只影响暂停，不直接排队开转。 */
    if (s_work.walkPhase == ROBOT_WALK_MEASURING &&
        (int32_t)(request->receivedTick - s_measureTick) > 0)
    {
        s_decision = *request;
        s_decisionValid = 1U;
    }
#endif
}

#endif

static void Vision_ProcessRequests(void)
{
    CameraWalkRequest_t request;
    uint8_t count;
    for (count = 0U; count < (ROBOT_WALK_LATE_ACTIVE ? CAMERA_WALK_SAMPLE_QUEUE_LENGTH + 17U : 17U); ++count)
    {
        if (CameraWalk_TakeRequest(&request) == 0U) { break; }
        if (request.type == CAMERA_WALK_SHAPE) { Vision_QueueShape(&request); }
        else if (request.type == CAMERA_WALK_COMMAND) { Vision_ReadCommand(&request); }
    }
}

static void Vision_StartEvent(void)
{
    VisionShapeEvent_t event = s_pending[s_pendingRead];
    s_pendingRead = (uint8_t)((s_pendingRead + 1U) % ROBOT_WALK_SHAPE_QUEUE_LENGTH);
    s_pendingCount--;
#if ROBOT_RUN_MODE != ROBOT_MODE_LINE_DEBUG
    s_activeShape = event.shape;
#endif
    s_work.activeShapeSequence = event.sequence;
    s_work.eventsStarted++;
#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN
    Debug_Printf("[EVENT] start shape=%u seq=%lu extra=%u\r\n",
        (unsigned int)event.shape, (unsigned long)event.sequence,
        (unsigned int)ROBOT_SHAPE_STEP_BEFORE_ACTION);
#endif
    CameraWalk_ClearMotion();
    s_decisionValid = 0U;
    s_runRequested = 0U; /* 图形结束后必须重新采样，禁止旧FWD自动续走。 */
#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN
    s_waitLineAfterTurn = 1U;
    s_twoStepsAfterRight = 0U; /* 图形动作取消旧的右转-两步序列。 */
#endif
    (void)Vision_StartSingle(MotionGroups_FromQr(event.shape), ROBOT_WALK_EVENT);
}

static void Vision_EnterMeasure(void)
{
    /* 清的是运动邮箱，图形FIFO不能清；只采用停稳之后收到的新建议。 */
    CameraWalk_ClearMotion();
    s_decisionValid = 0U;
#if (ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN) || (ROBOT_RUN_MODE == ROBOT_MODE_LINE_DEBUG)
    s_measureTick = (uint32_t)xTaskGetTickCount();
#endif
    s_measureElapsed = 0U;
#if ROBOT_WALK_LATE_ACTIVE
    s_windowOpen = 0U;
    Vision_HandoffReset();
    Vision_LateResetVotes();
#endif
    s_work.walkPhase = s_blocked != 0U ? ROBOT_WALK_BLOCKED : ROBOT_WALK_MEASURING;
#if ROBOT_RUN_MODE == ROBOT_MODE_LINE_DEBUG
    Debug_Printf("[LINE] stand_ready gate=%s\r\n", s_blocked != 0U ? "blocked" : "open");
#endif
}

/** @brief 图形仍留在队首；完成额外完整直行周期、回站和停稳后才出队执行。 */
static uint8_t Vision_DispatchPendingEvent(void)
{
    if (s_pendingCount == 0U) { return 0U; }
#if ROBOT_WALK_LATE_ACTIVE
    if (s_stopRequested != 0U || s_blocked != 0U) { return 0U; }
#endif
#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN
    if (ROBOT_SHAPE_STEP_BEFORE_ACTION != 0U &&
        s_pending[s_pendingRead].stepDone == 0U)
    {
        if (s_blocked != 0U || s_shapeStepPaused != 0U) { return 0U; }
        /* 沿用完整直行周期：左右腿各迈一次，再按原直行出口收脚。
         * 不截取与左转相同的0..6分段，也不从右跨立直接回零。 */
        if (MotionPlayer_StartCycles(Vision_WalkMotion(), 1U,
                ROBOT_WALK_LOOP_START_FRAME, ROBOT_WALK_CYCLE_END_FRAME,
                ROBOT_WALK_STOP_FRAME) == 0U)
        {
            Vision_Fault();
            return 1U;
        }
        s_decisionValid = 0U;
        s_runRequested = 0U;
        s_twoStepsAfterRight = 0U;
        s_waitLineAfterTurn = 1U;
        s_work.targetCycles = 1U; /* 完整左右腿周期，现有配置按2步计。 */
        s_work.walkPhase = ROBOT_WALK_SHAPE_STEP;
        Debug_Printf("[VISION] shape=%u extra_walk start cycles=1 steps=%u stop_frame=%u\r\n",
            (unsigned int)s_pending[s_pendingRead].shape,
            (unsigned int)ROBOT_WALK_STEPS_PER_CYCLE,
            (unsigned int)ROBOT_WALK_STOP_FRAME + 1U);
        return 1U;
    }
#endif
    Vision_StartEvent();
    return 1U;
}

static void Vision_Stable(void)
{
    uint8_t wasStartup = s_startup;
    s_startup = 0U;
    if (s_work.activeShapeSequence != 0U)
    {
#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN
        Debug_Printf("[EVENT] done shape=%u seq=%lu\r\n",
            (unsigned int)s_activeShape, (unsigned long)s_work.activeShapeSequence);
#endif
        s_work.eventsCompleted++;
        s_work.activeShapeSequence = 0U;
#if ROBOT_RUN_MODE != ROBOT_MODE_LINE_DEBUG
        s_activeShape = 0U;
#endif
    }
    if (s_work.activeCommandSequence != 0U)
    {
        s_work.lastCompletedCommandSequence = s_work.activeCommandSequence;
        s_work.activeCommandSequence = 0U;
    }
    if (Vision_DispatchPendingEvent() != 0U) { return; }
#if ROBOT_WALK_LATE_ACTIVE
    if (s_afterReturnValid != 0U && s_stopRequested == 0U && s_pendingCount == 0U)
    {
        CameraWalkRequest_t command = s_afterReturn;
        s_afterReturnValid = 0U;
        if (s_voteValid != 0U && s_voteWinner.command == CAMERA_WALK_FORWARD)
        {
            command = s_voteWinner;
            Debug_Printf("[CORRECT] cancel pending=%u old_seq=%lu fresh_FWD=%lu\r\n",
                (unsigned int)s_afterReturn.command, (unsigned long)s_afterReturn.sequence,
                (unsigned long)command.sequence);
        }
        s_decisionValid = 0U;
        Vision_LateResetVotes();
        if (command.command == CAMERA_WALK_FORWARD)
        {
            s_decision = command;
            s_runRequested = 1U;
            Vision_StartWalk();
            return;
        }
        Debug_Printf("[CORRECT] cached cmd=%u seq=%lu after_return=1\r\n",
            (unsigned int)command.command, (unsigned long)command.sequence);
        if (command.command == CAMERA_WALK_LEFT || command.command == CAMERA_WALK_RIGHT)
        { Vision_StartTurn(&command); }
        else { Vision_StartShift(&command); }
        return;
    }
#endif
    if (Vision_TryFastStage() != 0U && s_runRequested != 0U)
    { Vision_StartWalk(); return; }
    if (wasStartup != 0U)
    {
        if (s_runRequested != 0U && s_blocked == 0U)
        {
            CameraWalk_ClearMotion();
            Vision_StartWalk();
            return;
        }
    }
    Vision_EnterMeasure();
}

/** @brief 0毫秒直接完成，避免通过SETTLING阶段再多等一个控制周期。 */
static void Vision_WaitOrFinish(uint32_t durationMs)
{
    if (durationMs == 0U) { Vision_Stable(); }
    else
    {
        s_holdMs = durationMs;
        s_work.walkPhase = ROBOT_WALK_SETTLING;
    }
}

/** @brief 上电站稳后按模式等待图形或启动完整流程；@param 无；@retval 无。 */
void RobotWalkVision_Init(void)
{
    memset(&s_work, 0, sizeof(s_work));
    memset(s_pending, 0, sizeof(s_pending));
    memset(&s_decision, 0, sizeof(s_decision));
    s_pendingRead = s_pendingCount = 0U;
#if ROBOT_RUN_MODE != ROBOT_MODE_LINE_DEBUG
    s_activeShape = 0U;
#endif
    s_activeCommand = CAMERA_WALK_HOLD;
    s_runRequested = (ROBOT_RUN_MODE == ROBOT_MODE_LINE_DEBUG ||
        (ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN && ROBOT_STM_AUTOSTART != 0U)) ? 1U : 0U;
    s_blocked = 0U;
    s_startup = 1U;
#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN
    s_waitLineAfterTurn = 0U;
    s_twoStepsAfterRight = 0U;
    s_shapeStepPaused = 0U;
#endif
    s_holdMs = s_measureElapsed = 0U;
#if (ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN) || (ROBOT_RUN_MODE == ROBOT_MODE_LINE_DEBUG)
    s_measureTick = 0U;
#endif
    s_decisionValid = 0U;
#if ROBOT_WALK_LATE_ACTIVE
    s_batchCycleBase = 0U;
    s_batchId = s_windowTick = s_returnReviewTick = 0U;
    s_stopRequested = s_windowOpen = s_afterReturnValid = 0U;
    Vision_HandoffReset();
    Vision_LateResetVotes();
    Debug_Printf("[VISION_BUILD] %s %s mode=%u late=%u vote=%u/%u shape_extra=%u each_cycle=%u handoff=%u\r\n",
        __DATE__, __TIME__, (unsigned int)ROBOT_RUN_MODE,
        (unsigned int)ROBOT_WALK_LATE_VISION, (unsigned int)ROBOT_WALK_VOTE_NEED,
        (unsigned int)ROBOT_WALK_VOTE_WINDOW, (unsigned int)ROBOT_SHAPE_STEP_BEFORE_ACTION,
        (unsigned int)ROBOT_WALK_DECIDE_EACH_CYCLE,
        (unsigned int)ROBOT_WALK_TURN_HANDOFF_ENABLE);
#endif
    MotionPlayer_Init();
    MotionPlayer_SetFrameCompletedCallback(Vision_FrameCompleted);
    Debug_Printf("[GAIT] new_run gait=slow total=0 threshold=%lu autostart=%u\r\n",
        (unsigned long)ROBOT_SLOW_STAGE_STEPS, (unsigned int)ROBOT_STM_AUTOSTART);
    (void)Vision_StartSingle(motion_stand, ROBOT_WALK_STARTUP);
    Vision_Publish();
}

/** @brief STM计步并安全落脚，视觉决定暂停/恢复与停稳后的转向；@param elapsedMs推进毫秒；@retval 无。 */
void RobotWalkVision_Update(uint16_t elapsedMs)
{
#if ROBOT_WALK_LATE_ACTIVE
    Vision_LateSyncWindow();
#endif
    Vision_ProcessRequests();
    if (elapsedMs == 0U || s_work.walkPhase == ROBOT_WALK_FAULT)
    {
        Vision_Publish();
        return;
    }
#if ROBOT_WALK_LATE_ACTIVE
    if ((s_work.walkPhase == ROBOT_WALK_MEASURING || s_work.walkPhase == ROBOT_WALK_BLOCKED) &&
        s_decisionValid != 0U)
    {
        Debug_Printf("[DECISION] measure cmd=%u seq=%lu rx=%lu\r\n",
            (unsigned int)s_decision.command, (unsigned long)s_decision.sequence,
            (unsigned long)s_decision.receivedTick);
        s_blocked = s_stopRequested = 0U;
        s_runRequested = (uint8_t)(s_decision.command == CAMERA_WALK_FORWARD);
        s_work.walkPhase = ROBOT_WALK_MEASURING;
#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN
        s_shapeStepPaused = 0U;
#endif
    }
#endif
    if (s_work.walkPhase == ROBOT_WALK_MEASURING ||
        s_work.walkPhase == ROBOT_WALK_BLOCKED)
    {
        if (Vision_DispatchPendingEvent() != 0U) { /* 已启动单步或图形。 */ }
        else if (s_blocked == 0U && (ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN ||
                                     ROBOT_RUN_MODE == ROBOT_MODE_LINE_DEBUG))
        {
            s_measureElapsed += elapsedMs;
            if (s_decisionValid != 0U)
            {
                s_decisionValid = 0U;
                if (s_decision.command == CAMERA_WALK_FORWARD)
                {
#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN
                    s_waitLineAfterTurn = 0U;
                    if (s_pendingCount != 0U) { (void)Vision_DispatchPendingEvent(); }
                    else
#endif
                    { Vision_StartWalk(); }
                }
#ifdef ROBOT_BACKWARD_MOTION
                else if (s_decision.command == CAMERA_WALK_BACK)
                {
                    Vision_StartBack(&s_decision);
                }
#endif
                else if (s_decision.command == CAMERA_WALK_LEFT ||
                         s_decision.command == CAMERA_WALK_RIGHT)
                {
                    Vision_StartTurn(&s_decision);
                }
#if ROBOT_WALK_LATE_ACTIVE
                else if (s_decision.command == CAMERA_WALK_SHIFT_LEFT ||
                         s_decision.command == CAMERA_WALK_SHIFT_RIGHT)
                { Vision_StartShift(&s_decision); }
#endif
            }
            else if (ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN &&
#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN
                     s_waitLineAfterTurn == 0U &&
#endif
                     s_measureElapsed >= ROBOT_STM_CORRECTION_WAIT_MS &&
                     s_runRequested != 0U) { Vision_StartWalk(); }
        }
        else if (s_blocked != 0U) { s_work.walkPhase = ROBOT_WALK_BLOCKED; }
    }
    else if (s_work.walkPhase == ROBOT_WALK_SETTLING)
    {
        if ((uint32_t)elapsedMs >= s_holdMs) { s_holdMs = 0U; Vision_Stable(); }
        else { s_holdMs -= elapsedMs; }
    }
    else
    {
        (void)MotionPlayer_Update(elapsedMs);
        /* Also recheck priority inputs received during the endpoint submission. */
        Vision_ProcessRequests();
        if (s_work.fastSwitchPending != 0U &&
            (s_work.walkPhase == ROBOT_WALK_MOVING || s_work.walkPhase == ROBOT_WALK_TURN_APPROACH) &&
            s_activeCommand == CAMERA_WALK_FORWARD)
        { (void)MotionPlayer_RequestStopAtCycleEnd(); }
#if ROBOT_WALK_LATE_ACTIVE
        if (MotionPlayer_WalkHandoffReached() != 0U)
        {
            /* 帧末先复核已入队停止/图形/新FWD，再消费方向；本次不再推进第二帧。 */
            Vision_ProcessRequests();
            Vision_HandoffBoundary();
            Vision_Publish();
            return;
        }
        Vision_LateSyncWindow();
#endif
        if (MotionPlayer_IsActive() == 0U)
        {
#if ROBOT_WALK_LATE_ACTIVE
            if (s_work.walkPhase == ROBOT_WALK_TURN_HANDOFF)
            {
                if (Vision_CurrentFrameIsStand() == 0U) { Vision_Fault(); }
                else
                {
                    Debug_Printf("[HANDOFF] done cmd=%u seq=%lu; fresh measurement required\r\n",
                        (unsigned int)s_activeCommand,
                        (unsigned long)s_work.activeCommandSequence);
                    Vision_WaitOrFinish(ROBOT_WALK_TURN_SETTLE_MS);
                }
                Vision_Publish();
                return;
            }
#endif
#if ROBOT_RUN_MODE == ROBOT_MODE_FULL_RUN
            if (s_work.walkPhase == ROBOT_WALK_SHAPE_STEP)
            {
                /* 完整周期退出后采用原直行收脚表，保持落脚和重心转移顺序。 */
                Debug_Printf("[VISION] extra_walk done: walk_tostand\r\n");
                (void)Vision_StartSingle(motion_walk_tostand, ROBOT_WALK_SHAPE_STEP_RETURN);
            }
            else if (s_work.walkPhase == ROBOT_WALK_SHAPE_STEP_RETURN)
            {
                if (s_pendingCount == 0U) { Vision_Fault(); }
                else
                {
                    s_pending[s_pendingRead].stepDone = 1U;
                    Vision_WaitOrFinish(ROBOT_WALK_SETTLE_MS);
                }
            }
            else
#endif
            if (s_work.walkPhase == ROBOT_WALK_MOVING)
            {
                if (s_activeCommand == CAMERA_WALK_FORWARD)
                {
#if ROBOT_WALK_LATE_ACTIVE
                    if (Vision_LateFinishBatch() == 0U)
#endif
                    { (void)Vision_StartSingle(motion_walk_tostand, ROBOT_WALK_TO_STAND); }
                }
                else if ((s_activeCommand == CAMERA_WALK_LEFT ||
                          s_activeCommand == CAMERA_WALK_RIGHT
#if ROBOT_WALK_LATE_ACTIVE
                           || s_activeCommand == CAMERA_WALK_SHIFT_LEFT ||
                              s_activeCommand == CAMERA_WALK_SHIFT_RIGHT
#endif
                          ) &&
                         Vision_CurrentFrameIsStand() != 0U)
                {
                    /* 回站过渡保留在转向动作组中；默认不再额外停稳。
                     * 直接完成本次指令并清旧方向，后续只接受新视觉建议。 */
#if ROBOT_WALK_LATE_ACTIVE
                    if (s_activeCommand == CAMERA_WALK_LEFT || s_activeCommand == CAMERA_WALK_RIGHT)
                    {
                        Debug_Printf("[TURN] done cmd=%u seq=%lu cycles=%u\r\n",
                            (unsigned int)s_activeCommand,
                            (unsigned long)s_work.activeCommandSequence,
                            (unsigned int)MotionPlayer_GetCompletedCycles());
                    }
#endif
                    Vision_WaitOrFinish(ROBOT_WALK_TURN_SETTLE_MS);
                }
                else { (void)Vision_StartSingle(motion_stand, ROBOT_WALK_RECOVER); }
            }
            else if (s_work.walkPhase == ROBOT_WALK_EVENT)
            {
                /* 图形组末帧已回站时使用组内回位，不再重复播放站立。
                 * 未回站的组仍走原恢复路径；完成后仍清旧视觉方向。 */
                if (Vision_CurrentFrameIsStand() != 0U &&
                    (ROBOT_SKIP_DUPLICATE_SHAPE_STAND != 0U ||
                     MotionPlayer_GetMotion() == motion_qr_left_leg))
                {
                    Vision_WaitOrFinish(ROBOT_WALK_SETTLE_MS);
                }
                else
                {
                    (void)Vision_StartSingle(motion_stand, ROBOT_WALK_RECOVER);
                }
            }
            else
            {
                Vision_WaitOrFinish(s_work.walkPhase == ROBOT_WALK_STARTUP
                    ? ROBOT_WALK_STARTUP_SETTLE_MS : ROBOT_WALK_SETTLE_MS);
            }
        }
    }
    Vision_Publish();
}

uint8_t RobotWalkVision_BeginNewRun(void)
{
    if ((s_work.walkPhase != ROBOT_WALK_MEASURING && s_work.walkPhase != ROBOT_WALK_BLOCKED) ||
        MotionPlayer_IsActive() != 0U || s_pendingCount != 0U ||
        Vision_CurrentFrameIsStand() == 0U)
    { return 0U; }
    CameraWalk_ClearMotion();
    RobotWalkVision_Init();
    return 1U;
}

RobotControlStatus_t RobotWalkVision_GetStatus(void)
{
    RobotControlStatus_t snapshot;
    taskENTER_CRITICAL();
    snapshot = s_snapshot;
    taskEXIT_CRITICAL();
    return snapshot;
}
#endif
