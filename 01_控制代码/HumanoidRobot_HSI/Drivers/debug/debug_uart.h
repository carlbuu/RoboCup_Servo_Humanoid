#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 *函数简介:创建调试输出使用的FreeRTOS互斥量
 *参数说明:无
 *返回类型:无
 *备注:必须在首次调用Debug_Printf()之前执行
 */
void Debug_UART_Init(void);

/*
 *函数简介:格式化并通过UART4输出调试文本
 *参数说明:format为printf格式字符串，后续参数与标准printf一致
 *返回类型:无
 *备注:缓冲区容量256字节，使用互斥量保护并采用带超时的阻塞发送，禁止在ISR调用
 */
void Debug_Printf(char *format, ...);

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_UART_H */
