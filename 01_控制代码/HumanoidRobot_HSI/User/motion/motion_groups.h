#ifndef MOTION_GROUPS_H
#define MOTION_GROUPS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "robot_joints.h"

/* 角度使用 0.1 度整数，避免在 Flash 动作表中保存大量浮点常量。 */
typedef struct
{
    uint16_t durationMs;//到达这一帧目标姿态所用时间，单位毫秒
    int16_t angleDeg10[ROBOT_JOINT_COUNT];//13 个关节目标角度，单位是 0.1°。13 个关节目标角度，单位是 0.1°。
} MotionFrame_t;

typedef enum
{
	motion_stand = 0,
	motion_walk_forward,
	
	motion_turn_left,
	motion_turn_right,
	
	motion_qr_left_arm,
	motion_qr_right_arm,
	
	motion_qr_left_leg,
	motion_qr_right_leg,
	
	motion_qr_both_arms,
	
	motion_qr_head_shake,
	
	motion_step_over,
	
	motion_band_over,
	motion_walk_tostand,
	
	motion_left_translation,
	motion_right_translation,
	
    motion_walk_forward_fast, /* Append: preserve all existing action IDs. */
	motion_group_count
	
	
} MotionId_t;

typedef struct
{
    const char *name; 			//保存动作名称
    const MotionFrame_t *frames; //指向这个动作包含的所有“动作帧”。比如走路可能包含 10 帧舵机角度数据。
    uint8_t frameCount;			//表示这个动作一共有多少帧
    uint8_t loop;//表示这个动作是否循环播放，通常：- 0：不循环 - 1：循环
} MotionGroup_t;

/* 根据动作编号取得只读动作组；编号无效时返回站立动作。 */
const MotionGroup_t *MotionGroups_Get(MotionId_t id);

/* 将二维码数字 1~6 映射到规则规定的动作。无效数字返回站立。 */
MotionId_t MotionGroups_FromQr(uint8_t qrNumber);

#ifdef __cplusplus
}
#endif

#endif /* MOTION_GROUPS_H */
