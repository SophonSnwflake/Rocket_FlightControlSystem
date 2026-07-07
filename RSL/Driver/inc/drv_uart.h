#ifndef __DRV_UART_H
#define __DRV_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usart.h"
#include <stdint.h>

#define UART_RX_BUFFER_SIZE 512

/**
 * @brief UART通信接收处理回调函数类型定义
 * @param pRxData 接收数据指针
 * @param rxDataLength 接收数据长度
 */
typedef void (*UART_Call_Back)(uint8_t *pRxData, uint16_t rxDataLength);

/**
 * @brief UART通信处理结构体
 */
typedef struct
{
    UART_HandleTypeDef *huart;
    uint16_t rxDataLimit;
    UART_Call_Back rxCallbackFunction;
    uint8_t rxBuffer[UART_RX_BUFFER_SIZE];
} UART_Manage_Object_t;

void UART_Init(UART_HandleTypeDef *huart, UART_Call_Back rxCallbackFunction, uint16_t rxDataLimit);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_UART_H */


