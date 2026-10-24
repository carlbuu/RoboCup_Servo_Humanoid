#ifndef SERVO_CONFIG_H
#define SERVO_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif


/* ========================== 舵机总线与任务基础参数 ========================== */

#define SERVO_COUNT                         13  /* 总线上连接的舵机数量，本项目使用13个STS3215舵机 */
/* 当前启用ID1~13，覆盖双腿、双臂和头部。 */
#define SERVO_ACTIVE_COUNT                  13
#if (SERVO_ACTIVE_COUNT < 1) || (SERVO_ACTIVE_COUNT > SERVO_COUNT)
#error Invalid SERVO_ACTIVE_COUNT
#endif
#define SERVO_UART_BAUDRATE                 1000000/* 舵机通信波特率，STS3215默认支持1Mbps高速通信 */
#define SERVO_TASK_PERIOD_MS                20  /* 舵机控制任务周期，20ms对应50Hz控制频率 */
#define SERVO_TX_WAIT_TIMEOUT_MS            5    /* UART发送完成等待超时时间(ms)，防止DMA发送异常导致任务阻塞 */

/* ========================== SCS协议固定字段 ========================== */

#define SERVO_PACKET_HEADER                 0xFF/* SCS协议帧头，连续两个0xFF表示数据帧开始 */
#define SERVO_BROADCAST_ID                  0xFE/* 广播ID，发送给所有舵机；广播指令不产生应答 */
#define SERVO_INSTRUCTION_WRITE             0x03/* WRITE DATA指令，将数据写入指定控制表地址 */
#define SERVO_INSTRUCTION_SYNC_WRITE        0x83/* SYNC WRITE同步写指令，用于同时控制多个舵机 */

/* ========================== STS3215控制寄存器地址 ========================== */

/*
 * 扭矩开关寄存器地址：
 * 0x00 关闭扭矩输出
 * 0x01 开启扭矩输出
 */
#define SERVO_REGISTER_TORQUE_ENABLE        0x28

/*
 * 加速度寄存器地址：
 * 控制舵机运动加速度
 * 数据长度1字节
 */
#define SERVO_REGISTER_ACCELERATION         0x29

/*
 * 目标位置寄存器地址：
 * 从0x2A开始连续两个字节
 * 低字节在前，高字节在后
 */
#define SERVO_REGISTER_TARGET_POSITION      0x2A

/*
 * 运行速度寄存器地址：
 * 从0x2E开始连续两个字节
 * 低字节在前，高字节在后
 */
#define SERVO_REGISTER_RUNNING_SPEED        0x2E

/* ========================== 控制寄存器数据长度 ========================== */


#define SERVO_TORQUE_DATA_LENGTH            1/* 扭矩开关数据长度，单位：byte */
#define SERVO_ACCELERATION_DATA_LENGTH      1/* 加速度数据长度，单位：byte */
#define SERVO_POSITION_DATA_LENGTH          2/* 目标位置数据长度，单位：byte，16位数据 */
#define SERVO_SPEED_DATA_LENGTH             2/* 运行速度数据长度，单位：byte，16位数据 */



/*
 * SYNC WRITE同步运动控制的数据长度。
 *
 * 从0x29开始连续写入：
 *
 * 0x29 Acceleration 加速度
 * 0x2A Position_L  位置低8位
 * 0x2B Position_H  位置高8位
 * 0x2C Reserve_L   保留
 * 0x2D Reserve_H   保留
 * 0x2E Speed_L     速度低8位
 * 0x2F Speed_H     速度高8位
 *
 * 每个舵机共7字节数据。
 */
#define SERVO_SYNC_MOTION_DATA_LENGTH       7



/* ========================== DMA和通信缓存 ========================== */

/*
 * UART发送缓存大小。
 *
 * 最大同步控制帧：
 * 固定部分8字节
 * + 13个舵机 × (ID 1字节 + 数据7字节)
 * = 112字节
 *
 * 设置128字节预留扩展空间。
 */
#define SERVO_TX_BUFFER_SIZE                128


/*
 * USART DMA接收缓存大小。
 *
 * 用于接收舵机反馈数据。
 * 采用DMA循环接收 + IDLE空闲中断方式。
 */
#define SERVO_RX_DMA_BUFFER_SIZE            256


/*
 * 软件接收环形缓存大小。
 *
 * 用于保存DMA搬运后的数据，
 * 防止短时间大量数据进入造成丢失。
 */
#define SERVO_RX_RING_BUFFER_SIZE           512


/*
 * 单个反馈包最大参数长度。
 *
 * 当前用于预留读取：
 * 位置、速度、负载、电压、温度等数据。
 */
#define SERVO_FEEDBACK_PARAMETER_MAX        32


/*
 * 单个完整反馈帧最大长度。
 *
 * 包括：
 * FF FF
 * ID
 * Length
 * Error
 * Parameter
 * Checksum
 */
#define SERVO_FEEDBACK_FRAME_MAX            40



/* ========================== STS3215位置换算参数 ========================== */

/*
 * STS3215中心位置。
 *
 * 12bit位置范围：
 * 0 ~ 4095
 *
 * 中位：
 * 2048
 */
#define SERVO_POSITION_CENTER               2048


/* 软件位置保护最小值，防止异常数据超范围 */
#define SERVO_POSITION_MIN                  0


/* 软件位置保护最大值，防止异常数据超范围 */
#define SERVO_POSITION_MAX                  4095


/*
 * 单个位置单位对应角度。
 *
 * STS3215：
 * 1 position = 0.087°
 */
#define SERVO_DEGREES_PER_POSITION          0.087f



/* ========================== 默认运动参数 ========================== */

/*
 * 默认运行速度。
 *
 * 单位：
 * STS3215协议速度单位
 *
 * 仅作为初始化默认值，
 * 不会自动发送给舵机。
 */
#define SERVO_DEFAULT_SPEED                 1000

/* 默认加速度，仅用于初始化内存，不会在上电时自动发送位置命令 */
#define SERVO_DEFAULT_ACCELERATION          20

/* 上电回站立姿态专用参数；先写入舵机，再开启扭矩。 */
#define SERVO_STARTUP_STAND_SPEED           250U
#define SERVO_STARTUP_STAND_ACCELERATION    10U



/* ========================== FreeRTOS任务通知 ========================== */

/*
 * ServoTask通知位定义。
 *
 * 每个位表示一种事件：
 *
 * bit0:
 * DMA发送完成
 *
 * bit1:
 * 收到舵机反馈数据
 *
 * bit2:
 * UART通信错误
 */


/* USART DMA发送完成通知 */
#define SERVO_NOTIFY_TX_COMPLETE            0x00000001UL


/* USART接收到反馈数据通知 */
#define SERVO_NOTIFY_RX_DATA                0x00000002UL


/* UART通信异常通知，例如DMA错误、帧错误 */
#define SERVO_NOTIFY_UART_ERROR             0x00000004UL



#ifdef __cplusplus
}
#endif

#endif /* SERVO_CONFIG_H */

