#ifndef __DRV_UART_H
#define __DRV_UART_H

#ifdef __cplusplus
extern "C"{
#endif

#define UART_BUFFER_SIZE 1024

#include "RSL_common.h"
#include "usart.h"
#include "board_config.h"

typedef void (*UART_Call_Back)(uint8_t *pRxData, uint16_t rxDataLength);

typedef struct{
    UART_HandleTypeDef *huart;
    uint8_t rxBuffer[UART_BUFFER_SIZE];
    uint16_t rxDataLimit;
    UART_Call_Back rxCallBackFunction;
}   UART_Controller;

void UART_Init(UART_HandleTypeDef *huart, UART_Call_Back rxCallBackFunction, uint16_t rxDataLimit);
void UART_reinit(UART_HandleTypeDef *huart);
HAL_StatusTypeDef UART_SendData(UART_HandleTypeDef *huart, uint8_t *pTxData, uint16_t txDataLength);




#ifdef __cplusplus
}
#endif

#endif