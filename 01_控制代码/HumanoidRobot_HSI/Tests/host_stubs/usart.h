#ifndef BINARY_TEST_USART_H
#define BINARY_TEST_USART_H
#include <stdint.h>
typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
#define HAL_UART_STATE_READY   0x20U
#define HAL_UART_STATE_BUSY_TX 0x21U
typedef struct { volatile uint32_t gState; } UART_HandleTypeDef;
extern UART_HandleTypeDef huart2;
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart,
                                      const uint8_t *data, uint16_t len);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *uart);
#endif
