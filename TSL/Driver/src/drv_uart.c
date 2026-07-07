#include "drv_uart.h"
#include <stddef.h>

#define UART_INSTANCE_COUNT 3

static UART_Manage_Object_t s_uart_manage_object[UART_INSTANCE_COUNT];

static USART_TypeDef *const uartInstances[UART_INSTANCE_COUNT] = {
    USART1,
    USART2,
    USART6,
};

static UART_Manage_Object_t *UART_Get_Object(UART_HandleTypeDef *huart)
{
    for (int i = 0; i < UART_INSTANCE_COUNT; i++)
    {
        if (huart->Instance == uartInstances[i])
        {
            return &s_uart_manage_object[i];
        }
    }
    return NULL;
}

void UART_Init(UART_HandleTypeDef *huart, UART_Call_Back rxCallbackFunction, uint16_t rxDataLimit)
{
    UART_Manage_Object_t *uart_obj = UART_Get_Object(huart);
    if (uart_obj != NULL)
    {
        if (rxDataLimit > UART_RX_BUFFER_SIZE)
        {
            rxDataLimit = UART_RX_BUFFER_SIZE;
        }
        uart_obj->huart              = huart;
        uart_obj->rxCallbackFunction = rxCallbackFunction;
        uart_obj->rxDataLimit        = rxDataLimit;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_obj->rxBuffer, uart_obj->rxDataLimit);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    UART_Manage_Object_t *uart_obj = UART_Get_Object(huart);
    if (uart_obj == NULL)
    {
        return;
    }
    if (uart_obj->rxCallbackFunction != NULL)
    {
        uart_obj->rxCallbackFunction(uart_obj->rxBuffer, Size);
    }
    HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_obj->rxBuffer, uart_obj->rxDataLimit);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    UART_Manage_Object_t *uart_obj = UART_Get_Object(huart);
    if (uart_obj != NULL)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_obj->rxBuffer, uart_obj->rxDataLimit);
        if (uart_obj->rxCallbackFunction != NULL)
        {
            uart_obj->rxCallbackFunction(NULL, 0);
        }
    }
}
