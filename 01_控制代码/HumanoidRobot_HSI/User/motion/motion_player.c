#include "motion_player.h"
#include "robot_walk_config.h"

#include <string.h>

#include "servo.h"
#include "servo_config.h"

#define MOTION_SERVO_SPEED         1200U	
#define MOTION_FAST_WALK_SPEED     2000U
#define MOTION_SERVO_ACCELERATION  250U
/* 双手举起第一帧专用速度，仅用于ID11、ID12。 */
#define MOTION_BOTH_ARMS_SPEED     2600U
/*
	group	当前正在播放的动作组，
	motionId	当前动作编号，这个动作的具体数据
	frameIndex	当前播放到第几帧
	active	是否正在播放；1 是，0 是一次性动作已结束
	frameElapsedMs	当前帧已播放的时间
	startAngle[]	当前帧开始时的 13 个关节角度
	targetAngle[]	当前帧结束时的 13 个目标角度
	currentAngle[]	本次插值计算出的当前角度
*/
typedef struct
{
    const MotionGroup_t *group;
    MotionId_t motionId;
    uint8_t frameIndex;
    uint8_t active;
    uint16_t frameElapsedMs;
    uint8_t finitePlayback;
    uint8_t loopStartFrame;
    uint8_t cycleEndFrame;
    uint8_t stopFrame;
    uint8_t stopPending;
    uint8_t handoffArmed;
    uint8_t handoffReached;
    uint8_t handoffFrame;
    uint16_t targetCycles;
    uint16_t completedCycles;
    float startAngle[ROBOT_JOINT_COUNT];
    float targetAngle[ROBOT_JOINT_COUNT];
    float currentAngle[ROBOT_JOINT_COUNT];
} MotionPlayerState_t;

static MotionPlayerState_t s_player;
static MotionFrameCompletedCallback_t s_frameCompleted;

uint8_t MotionPlayer_IsForwardMotion(MotionId_t motion)
{
    return (uint8_t)(motion == motion_walk_forward || motion == motion_walk_forward_fast);
}

void MotionPlayer_SetFrameCompletedCallback(MotionFrameCompletedCallback_t callback)
{
    s_frameCompleted = callback;
}

/**
 * @brief    加载指定关键帧，以当前计算角度为起点，读取目标角度并清零帧计时
 * @param    frameIndex ：关键帧下标，从0开始，必须小于当前动作组的帧数
 * @retval   无
 * @note     调用前必须设置有效的动作组；本函数只准备插值数据，不发送舵机指令
 */
static void MotionPlayer_LoadFrame(uint8_t frameIndex)
{
    uint8_t joint;
    const MotionFrame_t *frame;

    frame = &s_player.group->frames[frameIndex];
    for (joint = 0U; joint < ROBOT_JOINT_COUNT; joint++)
    {
        s_player.startAngle[joint] = s_player.currentAngle[joint];
        s_player.targetAngle[joint] =
            (float)frame->angleDeg10[joint] * 0.1f;
    }

    s_player.frameElapsedMs = 0U;
}

/**
 * @brief    初始化动作播放器，选择站立动作并加载第0帧
 * @param    无
 * @retval   无
 * @note     当前计算角度直接初始化为站立角度，并非读取舵机实际位置；本函数不发送指令
 */
void MotionPlayer_Init(void)
{
    uint8_t joint;
    const MotionGroup_t *stand;

    memset(&s_player, 0, sizeof(s_player));
    s_frameCompleted = NULL;
    stand = MotionGroups_Get(motion_stand);

    for (joint = 0U; joint < ROBOT_JOINT_COUNT; joint++)
    {
        s_player.currentAngle[joint] =
            (float)stand->frames[0].angleDeg10[joint] * 0.1f;
    }

    s_player.motionId = motion_stand;
    s_player.group = stand;
    s_player.active = 1U;
    s_player.cycleEndFrame = (uint8_t)(stand->frameCount - 1U);
    s_player.stopFrame = s_player.cycleEndFrame;
    MotionPlayer_LoadFrame(0U);
}

/**
 * @brief    切换到指定动作的第0帧；若该动作仍在播放，则继续播放而不重新开始
 * @param    id ：动作编号，使用MotionId_t中定义的有效动作枚举值
 * @retval   无
 * @note     应先初始化播放器；后续由MotionPlayer_Update推进动作并提交舵机目标
 */
void MotionPlayer_Start(MotionId_t id)
{
    if ((s_player.active != 0U) && (s_player.motionId == id))
    {
        return;
    }

    s_player.motionId = id;
    s_player.group = MotionGroups_Get(id);
    s_player.frameIndex = 0U;
    s_player.active = 1U;
    s_player.finitePlayback = 0U;
    s_player.loopStartFrame =
        ((MotionPlayer_IsForwardMotion(id) != 0U) &&
         (s_player.group->frameCount > ROBOT_WALK_LOOP_START_FRAME))
        ? ROBOT_WALK_LOOP_START_FRAME : 0U;
    s_player.cycleEndFrame = (uint8_t)(s_player.group->frameCount - 1U);
    s_player.stopFrame = s_player.cycleEndFrame;
    s_player.stopPending = 0U;
    s_player.handoffArmed = s_player.handoffReached = 0U;
    s_player.targetCycles = 0U;
    s_player.completedCycles = 0U;
    MotionPlayer_LoadFrame(0U);
}

/**
 * @brief    按指定回环、计数和退出帧启动有限次数的动作
 * @param    id ：动作编号；cycles ：周期数，必须大于0
 * @param    loopStartFrame ：回环帧；cycleEndFrame ：计数帧；stopFrame ：退出帧
 * @retval   1：成功；0：参数无效，原播放状态不变
 * @note     每次调用都重新开始，流程只在进入新步骤时调用一次。
 */
uint8_t MotionPlayer_StartCycles(MotionId_t id, uint16_t cycles,
                                uint8_t loopStartFrame,
                                uint8_t cycleEndFrame, uint8_t stopFrame)
{
    const MotionGroup_t *group;

    if (((unsigned int)id >= (unsigned int)motion_group_count) ||
        (cycles == 0U))
    {
        return 0U;
    }

    group = MotionGroups_Get(id);
    if ((group == 0) || (group->frames == 0) || (group->frameCount == 0U) ||
        (loopStartFrame >= group->frameCount) ||
        (cycleEndFrame >= group->frameCount) ||
        (stopFrame >= group->frameCount) ||
        (loopStartFrame > cycleEndFrame) || (loopStartFrame > stopFrame))
    {
        return 0U;
    }

    s_player.motionId = id;
    s_player.group = group;
    s_player.frameIndex = 0U;
    s_player.active = 1U;
    s_player.finitePlayback = 1U;
    s_player.loopStartFrame = loopStartFrame;
    s_player.cycleEndFrame = cycleEndFrame;
    s_player.stopFrame = stopFrame;
    s_player.stopPending = 0U;
    s_player.handoffArmed = s_player.handoffReached = 0U;
    s_player.targetCycles = cycles;
    s_player.completedCycles = 0U;
    MotionPlayer_LoadFrame(0U);
    return 1U;
}

uint8_t MotionPlayer_StartRange(MotionId_t id, uint8_t firstFrame, uint8_t lastFrame)
{
    const MotionGroup_t *group = MotionGroups_Get(id);
    if ((unsigned int)id >= (unsigned int)motion_group_count ||
        firstFrame > lastFrame || lastFrame >= group->frameCount)
    { return 0U; }
    if (MotionPlayer_StartCycles(id, 1U, firstFrame, lastFrame, lastFrame) == 0U)
    { return 0U; }
    s_player.frameIndex = firstFrame;
    MotionPlayer_LoadFrame(firstFrame);
    return 1U;
}

uint8_t MotionPlayer_ArmWalkHandoff(uint8_t frame)
{
    if (s_player.active == 0U || s_player.finitePlayback == 0U ||
        s_player.motionId != motion_walk_forward || s_player.group == NULL ||
        frame >= s_player.group->frameCount || frame >= s_player.cycleEndFrame ||
        s_player.handoffReached != 0U)
    { return 0U; }
    s_player.handoffFrame = frame;
    s_player.handoffArmed = 1U;
    return 1U;
}

void MotionPlayer_CancelWalkHandoff(void) { s_player.handoffArmed = 0U; }
uint8_t MotionPlayer_WalkHandoffReached(void) { return s_player.handoffReached; }

uint8_t MotionPlayer_ResumeWalkHandoff(void)
{
    if (s_player.handoffReached == 0U || s_player.active != 0U ||
        s_player.motionId != motion_walk_forward || s_player.group == NULL ||
        (uint16_t)s_player.frameIndex + 1U >= s_player.group->frameCount)
    { return 0U; }
    s_player.handoffReached = s_player.handoffArmed = 0U;
    s_player.frameIndex++;
    s_player.active = 1U;
    MotionPlayer_LoadFrame(s_player.frameIndex);
    return 1U;
}

/**
 * @brief    推进当前帧计时，线性插值各关节角度并提交同步目标，到帧末时切换下一帧
 * @param    elapsedMs ：本次推进的时间，单位为毫秒；当前控制任务每20ms调用一次并传入20
 * @retval   1 ：所有关节目标设置成功，且同步目标已成功提交到发送队列
 * @retval   0 ：播放器未激活、动作组为空，或目标设置、提交失败
 * @note     返回值不表示舵机已到位；帧末目标提交失败时保留该帧并重试
 */
uint8_t MotionPlayer_Update(uint16_t elapsedMs)
{
    uint8_t joint;
    uint8_t allTargetsAccepted;
    uint8_t acceleration;
    uint16_t durationMs;
    uint16_t speed;
	uint16_t jointSpeed;  /* 当前关节本次使用的速度 */
    float ratio;

    if ((s_player.active == 0U) || (s_player.group == 0))
    {
        return 0U;
    }

    durationMs = s_player.group->frames[s_player.frameIndex].durationMs;
    if (durationMs == 0U)
    {
        durationMs = 1U;
    }

    if ((uint32_t)s_player.frameElapsedMs + elapsedMs >= durationMs)
    {
        s_player.frameElapsedMs = durationMs;
    }
    else
    {
        s_player.frameElapsedMs =
            (uint16_t)(s_player.frameElapsedMs + elapsedMs);
    }

    ratio = (float)s_player.frameElapsedMs / (float)durationMs;
    allTargetsAccepted = 1U;
    speed = (s_player.motionId == motion_stand)
        ? SERVO_STARTUP_STAND_SPEED : (s_player.motionId == motion_walk_forward_fast
            ? MOTION_FAST_WALK_SPEED : MOTION_SERVO_SPEED);
    acceleration = (s_player.motionId == motion_stand)
        ? SERVO_STARTUP_STAND_ACCELERATION : MOTION_SERVO_ACCELERATION;

    for (joint = 0U; joint < ROBOT_JOINT_COUNT; joint++)
    {
        s_player.currentAngle[joint] = s_player.startAngle[joint] +
            (s_player.targetAngle[joint] - s_player.startAngle[joint]) * ratio;

        /* 动作表为13列；仅向当前启用范围内的舵机提交目标。 */
        if (ROBOT_SERVO_ID(joint) > SERVO_ACTIVE_COUNT)
        {
            continue;
        }

        /* 每个关节先使用默认速度。 */
		jointSpeed = speed;

		/* 双手举起、放下使用专用速度；中间保持帧不变。 */
		if (((s_player.motionId == motion_qr_both_arms) ||
		 (s_player.motionId == motion_qr_left_arm) ||
		 (s_player.motionId == motion_qr_right_arm)) &&
		((ROBOT_SERVO_ID(joint) == 11U) ||
		 (ROBOT_SERVO_ID(joint) == 12U)))
		{
			jointSpeed = MOTION_BOTH_ARMS_SPEED;
		}

		if (Servo_SetTarget(ROBOT_SERVO_ID(joint),
							jointSpeed,
							acceleration,
							s_player.currentAngle[joint]) == 0U)
		{
			allTargetsAccepted = 0U;
		}
    }

    if (allTargetsAccepted != 0U)
    {
        allTargetsAccepted = Servo_SendTargets();
    }

    if ((s_player.frameElapsedMs >= durationMs) &&
        (allTargetsAccepted != 0U))
    {
        /* A successful endpoint leaves this frame (or deactivates the player).
         * Failed submissions never reach here; resume loads the NEXT frame. */
        if (s_frameCompleted != NULL)
        { s_frameCompleted(s_player.motionId, s_player.frameIndex); }
        if ((s_player.stopPending == 0U) &&
            (s_player.frameIndex == s_player.cycleEndFrame))
        {
            if (s_player.completedCycles < 65535U)
            {
                s_player.completedCycles++;
            }
            if ((s_player.finitePlayback != 0U) &&
                (s_player.completedCycles >= s_player.targetCycles))
            {
                s_player.stopPending = 1U;
            }
        }

        /* 最终退出帧也必须先成功提交；例如行走落脚帧或左转中立帧。 */
        if (s_player.handoffArmed != 0U &&
            s_player.frameIndex == s_player.handoffFrame)
        {
            /* 不虚增完整周期；端点仅成功入队，不代表实际舵机到位。 */
            s_player.handoffArmed = 0U;
            s_player.handoffReached = 1U;
            s_player.active = 0U;
        }
        else if ((s_player.stopPending != 0U) &&
            (s_player.frameIndex == s_player.stopFrame))
        {
            s_player.active = 0U;
        }
        else if ((uint16_t)s_player.frameIndex + 1U >= s_player.group->frameCount)
        {
            if ((s_player.finitePlayback != 0U) || (s_player.group->loop != 0U))
            {
                s_player.frameIndex = s_player.loopStartFrame;
                MotionPlayer_LoadFrame(s_player.frameIndex);
            }
            else
            {
                s_player.active = 0U;
            }
        }
        else
        {
            s_player.frameIndex++;
            MotionPlayer_LoadFrame(s_player.frameIndex);
        }
    }

    return allTargetsAccepted;
}

/**
 * @brief    查询动作播放器是否处于播放状态
 * @param    无
 * @retval   1 ：正在播放；原有无限循环模式会一直保持此状态
 * @retval   0 ：尚未初始化，或单次/指定次数播放已结束
 * @note     此状态由软件播放计时决定，不代表舵机实际运动是否完成
 */
uint8_t MotionPlayer_IsActive(void)
{
    return s_player.active;
}

/**
 * @brief    查询本次播放已完成的软件周期数
 * @param    无
 * @retval   已完成周期数，达到65535后饱和
 */
uint16_t MotionPlayer_GetCompletedCycles(void)
{
    return s_player.completedCycles;
}

/**
 * @brief 缩短有限播放到当前周期结束，保留原计数帧和退出帧
 * @param 无
 * @retval 1已请求安全退出，0不适用
 */
/**
 * @brief 直行在退出帧完成后续播，保留当前插值角度及12->4的原支撑路径。
 * @param cycles 新批次周期数；必须大于0。
 * @retval 1成功；0不是已完成的有限直行出口，原状态不变。
 */
uint8_t MotionPlayer_ContinueWalkCycles(uint16_t cycles)
{
    if (cycles == 0U || s_player.active != 0U || s_player.finitePlayback == 0U ||
        MotionPlayer_IsForwardMotion(s_player.motionId) == 0U || s_player.group == NULL ||
        s_player.frameIndex != ROBOT_WALK_STOP_FRAME ||
        s_player.cycleEndFrame != ROBOT_WALK_CYCLE_END_FRAME ||
        s_player.stopFrame != s_player.cycleEndFrame ||
        (uint16_t)s_player.stopFrame + 1U >= s_player.group->frameCount)
    {
        return 0U;
    }
    /* 长时间运行到计数上限时，仅重置软件累计数，不改变姿态和动作顺序。 */
    if ((uint32_t)s_player.completedCycles + cycles > 65535U)
    {
        s_player.completedCycles = 0U;
    }
    s_player.targetCycles = (uint16_t)(s_player.completedCycles + cycles);
    s_player.stopPending = 0U;
    s_player.active = 1U;
    s_player.frameIndex = (uint8_t)(s_player.stopFrame + 1U);
    MotionPlayer_LoadFrame(s_player.frameIndex);
    return 1U;
}

uint8_t MotionPlayer_RequestStopAtCycleEnd(void)
{
    if ((s_player.active == 0U) || (s_player.finitePlayback == 0U))
    {
        return 0U;
    }
    if (s_player.stopPending == 0U)
    {
        s_player.targetCycles = (uint16_t)(s_player.completedCycles + 1U);
    }
    return 1U;
}

/**
 * @brief    查询当前播放或最后退出的帧下标
 * @param    无
 * @retval   从0开始的帧下标
 */
uint8_t MotionPlayer_GetFrameIndex(void)
{
    return s_player.frameIndex;
}

/**
 * @brief    获取播放器保存的当前动作编号
 * @param    无
 * @retval   当前动作编号，例如motion_stand或motion_walk_forward
 * @note     非循环动作结束后仍保留该动作编号，可配合MotionPlayer_IsActive查询播放状态
 */
MotionId_t MotionPlayer_GetMotion(void)
{
    return s_player.motionId;
}

/**
 * @brief    获取当前动作组的名称，供日志或状态显示使用
 * @param    无
 * @retval   动作名称字符串指针，例如"stand"或"walk"；动作组为空时返回"none"
 * @note     返回字符串为只读数据，调用者不应修改或释放
 */
const char *MotionPlayer_GetName(void)
{
    if (s_player.group == 0)
    {
        return "none";
    }

    return s_player.group->name;
}
