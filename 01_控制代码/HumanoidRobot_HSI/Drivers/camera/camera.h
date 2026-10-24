#ifndef CAMERA_H
#define CAMERA_H

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define CAMERA_RX_BUFFER_SIZE     256U
#define CAMERA_FRAME_MAX          96U
#define CAMERA_SERVICE_PERIOD_MS  10U

typedef enum
{
    CAMERA_SHAPE_NONE = 0,
    CAMERA_SHAPE_CIRCLE = 1,
    CAMERA_SHAPE_STAR = 2,
    CAMERA_SHAPE_SQUARE = 3,
    CAMERA_SHAPE_DIAMOND = 4,
    CAMERA_SHAPE_CROSS = 5,
    CAMERA_SHAPE_TRIANGLE = 6
} CameraShape_t;

typedef enum
{
    CAMERA_STATE_RECOGNIZING = 0,
    CAMERA_STATE_SUCCESS
} CameraState_t;

typedef struct
{
    CameraState_t state;
    CameraShape_t shape;
    uint32_t recognitionSequence; /* STM32本地计数，原视觉协议没有事件序号。 */
    uint32_t receivedTick;
} CameraStatus_t;

typedef struct
{
    uint32_t validFrames;
    uint32_t checksumErrors;
    uint32_t formatErrors;
    uint32_t overflowErrors;
    uint32_t shapeFrames;
    uint32_t commandFrames;
    uint32_t requestQueueFull;
    uint32_t commandCoalesced; /* 尚未消费的巡线建议被新建议覆盖的次数。 */
} CameraStatistics_t;

extern uint8_t Camera_RxBuffer[CAMERA_RX_BUFFER_SIZE];

/** @brief 清空原版geometry_vision7解析器与请求队列；@param 无；@retval 无。 */
void Camera_Init(void);
/**
 * @brief 解析USART2 DMA数据块，支持半帧、粘包和错误帧重新同步
 * @param data 接收字节；len 字节数
 * @retval 无
 * @note CAMERA_PROTOCOL_BINARY选择原版ASCII或二进制；仅CameraTask调用。
 */
void Camera_Process(uint8_t *data, uint16_t len);
/** @brief DMA任务传入IDLE接收时间；测试/非DMA输入可使用Camera_Process。 */
void Camera_ProcessTimed(uint8_t *data, uint16_t len, uint32_t receivedTick);
/** @brief 获取一致识别状态快照；@param 无；@retval 状态副本，不清除结果。 */
CameraStatus_t Camera_GetStatus(void);
/** @brief 获取最近识别图形；@param 无；@retval 图形编号。 */
CameraShape_t Camera_GetShape(void);
/** @brief 获取接收统计；@param 无；@retval 统计副本。 */
CameraStatistics_t Camera_GetStatistics(void);
/** @brief 返回状态名称；@param state 状态；@retval 静态字符串。 */
const char *Camera_GetStateName(CameraState_t state);
/** @brief 返回图形名称；@param shape 图形编号；@retval 静态字符串。 */
const char *Camera_GetShapeName(CameraShape_t shape);

#ifdef __cplusplus
}
#endif
#endif /* CAMERA_H */
