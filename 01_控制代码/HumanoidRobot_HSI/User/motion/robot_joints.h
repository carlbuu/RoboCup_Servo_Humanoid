#ifndef ROBOT_JOINTS_H
#define ROBOT_JOINTS_H

#include <stdint.h>

/* 动作数组按关节顺序排列；数组下标不等于舵机 ID。 */
typedef enum
{
    left_hip_ROLL = 0,       /* ID5  左髋侧摆 */
    left_hip_PITCH,          /* ID4  左髋前后 */
    left_knee_PITCH,         /* ID3  左膝前后 */
    left_ankle_PITCH,        /* ID2  左踝前后 */
    left_ankle_ROLL,         /* ID1  左踝侧摆 */
	
    rightT_hip_ROLL,         /* ID6  右髋侧摆 */
    rightT_hip_PITCH,        /* ID7  右髋前后 */
    right_knee_PITCH,        /* ID8  右膝前后 */
    right_ankle_PITCH,        /* ID9  右踝前后 */
    right_ankle_ROLL,         /* ID10 右踝侧摆 */
	
    leftshoulder_PITCH,      /* ID11 左臂前举 */
    rightshoulder_PITCH,     /* ID12 右臂前举 */
    head_YAW,                /* ID13 头部左右 */
    ROBOT_JOINT_COUNT
} RobotJoint_t;

/* 只计算实参一次；非法下标返回无效 ID 0，由舵机层拒绝。 */
static __inline uint8_t RobotJoint_ServoId(uint8_t joint)
{
    if (joint >= (uint8_t)ROBOT_JOINT_COUNT)
    {
        return 0U;
    }
    return (joint <= (uint8_t)left_ankle_ROLL)
        ? (uint8_t)(5U - joint)
        : (uint8_t)(joint + 1U);
}

#define ROBOT_SERVO_ID(joint) RobotJoint_ServoId((uint8_t)(joint))

/* 枚举常量可用于 C 整数常量表达式；负数组长度使编译失败。 */
typedef char RobotJointCountMustBe13[
    (ROBOT_JOINT_COUNT == 13) ? 1 : -1];

#endif /* ROBOT_JOINTS_H */
