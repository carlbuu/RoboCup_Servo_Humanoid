#ifndef CAMERA_WALK_LINK_H
#define CAMERA_WALK_LINK_H

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    CAMERA_WALK_COMMAND = 1,
    CAMERA_WALK_SHAPE = 2
} CameraWalkRequestType_t;

typedef enum
{
    CAMERA_WALK_HOLD = 0,
    CAMERA_WALK_FORWARD = 1,
    CAMERA_WALK_LEFT = 2,
    CAMERA_WALK_RIGHT = 3,
    CAMERA_WALK_BLOCKED = 4,
    CAMERA_WALK_BACK = 5,
    CAMERA_WALK_SHIFT_LEFT = 6,
    CAMERA_WALK_SHIFT_RIGHT = 7
} CameraWalkCommand_t;

typedef struct
{
    uint32_t sequence;
    uint32_t receivedTick;
    uint8_t type;
    uint8_t command;
    uint8_t arg;
    uint8_t shape;
    uint32_t firstReceivedTick; /* 完整指令第一段字节的接收时间，拒绝跨窗口旧帧。 */
} CameraWalkRequest_t;

/**
 * @brief RobotTask取出请求，新策略优先停止、图形FIFO、普通样本FIFO；旧策略取最新建议
 * @param request 接收请求的地址
 * @retval 1已取出；0队列为空或地址无效
 */
uint8_t CameraWalk_TakeRequest(CameraWalkRequest_t *request);
/** @brief 清旧方向建议，保留未消费HOLD/BLOCKED和图形；@param 无；@retval 无。 */
void CameraWalk_ClearMotion(void);
/** @brief 摄像头任务周期发送软件调度状态；不代表舵机实际到位。 */
void CameraWalk_SendStatus(void);

#ifdef __cplusplus
}
#endif
#endif /* CAMERA_WALK_LINK_H */
