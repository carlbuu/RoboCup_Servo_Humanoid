#include "debug_uart.h"

#include <stdarg.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "usart.h"

#define DEBUG_PRINTF_BUFFER_SIZE  256
#define DEBUG_UART_TIMEOUT_MS     100

static SemaphoreHandle_t s_debugMutex;

/*
 *函数简介:初始化调试串口输出所需的FreeRTOS互斥量
 *参数说明:无
 *返回类型:无
 *备注:互斥量用于防止多个任务同时格式化和发送日志；必须在首次调用
 *     Debug_Printf()之前执行，创建失败时进入Error_Handler()
 */
void Debug_UART_Init(void)
{
    /*
     * 互斥量防止多个任务同时格式化和发送调试文本。
     * Debug_Printf() 只允许在任务或普通函数中调用，不能在 ISR 中调用。
     */
    s_debugMutex = xSemaphoreCreateMutex();
    if (s_debugMutex == NULL)
    {
        Error_Handler();
    }
}

/*
 *函数简介:按照printf格式通过UART4输出调试信息
 *参数说明:format为格式字符串，后续可变参数与标准printf规则一致
 *返回类型:无
 *备注:内部使用256字节临时缓冲区，超长文本会被截断；互斥量保证多任务输出
 *     不交叉；UART4采用带100毫秒超时的阻塞发送，因此禁止在中断中调用
 */
void Debug_Printf(char *format, ...)
{
    char buffer[DEBUG_PRINTF_BUFFER_SIZE];
    va_list args;
    int length;

    if (format == NULL)
    {
        return;
    }

    (void)xSemaphoreTake(s_debugMutex, portMAX_DELAY);

    va_start(args, format);
    length = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (length > 0)
    {
        uint16_t txLength = (length >= (int)sizeof(buffer))
                          ? (uint16_t)(sizeof(buffer) - 1)
                          : (uint16_t)length;

        /*
         * 调试串口仅低频输出日志，允许阻塞发送。
         * 项目禁止的是阻塞式 UART 接收；USART1/2 接收始终使用 DMA。
         */
        (void)HAL_UART_Transmit(&huart4,
                                (uint8_t *)buffer,
                                txLength,
                                DEBUG_UART_TIMEOUT_MS);
    }

    (void)xSemaphoreGive(s_debugMutex);
}

/*
 *函数简介:将Keil/ARM Compiler标准库的单字符printf输出重定向到UART4
 *参数说明:ch为待输出字符；stream为标准库文件流指针，本函数中不使用
 *返回类型:int，发送后返回原字符ch，保持标准fputc()接口语义
 *备注:使用阻塞式HAL_UART_Transmit()发送单字节，只用于调试输出；
 *     禁止在中断中调用，高频日志建议改为调试发送队列或DMA方式
 */
int fputc(int ch, FILE *stream)
{
    uint8_t value = (uint8_t)ch;

    (void)stream;
    (void)HAL_UART_Transmit(&huart4, &value, 1, DEBUG_UART_TIMEOUT_MS);
    return ch;
}

/*
 *函数简介:为使用__io_putchar接口的C库重定向单字符输出到UART4
 *参数说明:ch为待输出字符
 *返回类型:int，发送后返回原字符ch
 *备注:该接口兼容部分printf重定向实现，采用100毫秒超时的阻塞发送；
 *     与fputc()作用相同，用户可根据所使用的C库保留对应入口
 */
int __io_putchar(int ch)
{
    uint8_t value = (uint8_t)ch;

    (void)HAL_UART_Transmit(&huart4, &value, 1, DEBUG_UART_TIMEOUT_MS);
    return ch;
}
