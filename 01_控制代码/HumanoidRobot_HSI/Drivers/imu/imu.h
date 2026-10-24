#ifndef IMU_H
#define IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* JY61每个原始数据帧固定为11字节。 */
#define JY61_FRAME_LENGTH          11

/* JY61协议固定帧头。 */
#define JY61_FRAME_HEADER          0x55

/* 三种需要处理的子帧类型。 */
#define JY61_FRAME_ACCEL           0x51
#define JY61_FRAME_GYRO            0x52
#define JY61_FRAME_ANGLE           0x53

/* 加速度量程为正负16g。 */
#define JY61_ACCEL_RANGE_G         16.0f

/* 标准重力加速度，用于把g换算为m/s^2。 */
#define JY61_GRAVITY_MPS2          9.80665f

/*
 * JY61/HWT101常用角速度量程为正负2000度每秒。
 * 如果使用说明书中特别标注的WT9011G4K型号，请修改为4000.0f。
 */
#define JY61_GYRO_RANGE_DPS        2000.0f

/* 欧拉角输出范围为正负180度。 */
#define JY61_ANGLE_RANGE_DEG       180.0f

typedef struct
{
    /* 加速度，单位为m/s^2。 */
    float accel_x;
    float accel_y;
    float accel_z;

    /* 角速度，单位为度每秒。 */
    float gyro_x;
    float gyro_y;
    float gyro_z;

    /* 欧拉角，单位为度。 */
    float roll;
    float pitch;
    float yaw;

} IMU_Data_t;

/*
 * 保留一个11字节原始帧缓冲区，方便以后查看或单独处理一帧原始数据。
 * 当前FreeRTOS任务使用流式组帧，不依赖DMA数据块边界。
 */
extern uint8_t JY61_RxBuffer[JY61_FRAME_LENGTH];

/*
 *函数简介:把一段任意长度的JY61串口数据送入流式协议解析器
 *参数说明:data指向本次收到的原始数据；len表示原始数据中的有效字节数
 *返回类型:无
 *备注:数据可以是半帧、一帧或多帧；函数自动寻找帧头，每个11字节帧独立处理
 */
void IMU_Parse(uint8_t *data, uint16_t len);

/*
 *函数简介:连续接收并组装JY61的11字节子帧
 *参数说明:data指向原始串口数据块；len表示数据块中的有效字节数
 *返回类型:无
 *备注:该接口不依赖UART IDLE数据块边界；遇到错误数据时直接放弃当前帧，等待
 *     后续数据中的下一个0x55帧头
 */
void JY61_StreamInput(const uint8_t *data, uint16_t len);

/*
 *函数简介:获取最近一次解析成功的IMU数据快照
 *参数说明:无
 *返回类型:IMU_Data_t，包含三轴加速度、三轴角速度和三个欧拉角
 *备注:返回的是结构体副本；函数内部使用FreeRTOS临界区保证数据读取一致
 */
IMU_Data_t IMU_GetData(void);

/*
 *函数简介:处理一个已经接收完成的11字节JY61原始数据帧
 *参数说明:buffer指向固定11字节缓冲区，帧类型可以是0x51、0x52或0x53
 *返回类型:无
 *备注:函数根据原始帧类型独立更新对应数据，其他字段保留上一次有效值
 */
void JY61_Process(uint8_t *buffer);

#ifdef __cplusplus
}
#endif

#endif /* IMU_H */
