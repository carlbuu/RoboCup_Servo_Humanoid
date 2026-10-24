#ifndef MOTION_PLAYER_H
#define MOTION_PLAYER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "motion_groups.h"

/* Called exactly once after a timed frame endpoint is successfully submitted.
 * Runs in the player task; callback must not start/advance another motion.
 * Software completion only, NOT servo position or foot-contact feedback. */
typedef void (*MotionFrameCompletedCallback_t)(MotionId_t motion, uint8_t frame);
void MotionPlayer_SetFrameCompletedCallback(MotionFrameCompletedCallback_t callback);
uint8_t MotionPlayer_IsForwardMotion(MotionId_t motion);

/* 初始化插值器并把当前姿态设为比赛站立姿态。 */
void MotionPlayer_Init(void);

/*
 * 切换动作。若同一循环动作已经在运行，不会重复从第一帧开始，
 * 因此视觉控制任务可以每个周期安全地给出期望动作。
 */
void MotionPlayer_Start(MotionId_t id);

/**
 * @brief    强制从第0帧启动指定次数的播放，不受动作表loop字段影响
 * @param    id ：动作编号；cycles ：周期数，必须大于0
 * @param    loopStartFrame ：回环帧；cycleEndFrame ：周期计数帧；stopFrame ：退出帧
 * @retval   1：启动成功；0：编号、帧下标或周期数无效，保持原播放器状态
 * @note     下标从0开始。到达计数帧累计周期；达到次数后继续到退出帧。
 *           回环帧必须不大于计数帧和退出帧；第一次播放包含第0帧起的准备段。
 */
uint8_t MotionPlayer_StartCycles(MotionId_t id, uint16_t cycles,
                                uint8_t loopStartFrame,
                                uint8_t cycleEndFrame, uint8_t stopFrame);

/* 单次播放指定闭区间，保留全部关节当前插值角度；不重置为关键帧目标。 */
uint8_t MotionPlayer_StartRange(MotionId_t id, uint8_t firstFrame, uint8_t lastFrame);
/* 仅有限行走：指定帧末成功提交后暂停，由调度器选择衔接或原路续播。 */
uint8_t MotionPlayer_ArmWalkHandoff(uint8_t frame);
void MotionPlayer_CancelWalkHandoff(void);
uint8_t MotionPlayer_WalkHandoffReached(void);
uint8_t MotionPlayer_ResumeWalkHandoff(void);

/* 每 20 ms 调用一次，插值并同步发送 13 个舵机目标。 */
uint8_t MotionPlayer_Update(uint16_t elapsedMs);

/* 播放结束后返回0；指定次数播放也会结束；原有无限循环播放始终返回1。 */
uint8_t MotionPlayer_IsActive(void);

/* 已完成的软件播放周期数；不代表舵机位置反馈。 */
uint16_t MotionPlayer_GetCompletedCycles(void);
/**
 * @brief 请求在本轮计数/退出帧结束有限周期播放，不立即切换姿态
 * @param 无
 * @retval 1请求有效，0当前不是有限周期动作或已经结束
 */
uint8_t MotionPlayer_RequestStopAtCycleEnd(void);
/** @brief 已完成有限直行后保留姿态续播12->4；返回1成功，0不适用。 */
uint8_t MotionPlayer_ContinueWalkCycles(uint16_t cycles);
/* 当前帧下标，从0开始；结束后保留最后退出帧。 */
uint8_t MotionPlayer_GetFrameIndex(void);

MotionId_t MotionPlayer_GetMotion(void);
const char *MotionPlayer_GetName(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTION_PLAYER_H */

