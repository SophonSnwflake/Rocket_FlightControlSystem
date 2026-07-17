#include "drv_uart.h"
#include "stm32f411xe.h"
#include "stm32f4xx_hal_uart.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "semphr.h"

static SemaphoreHandle_t s_printfMutex = NULL;

static void ensure_printf_mutex(void)
{
    if (s_printfMutex == NULL) {
        taskENTER_CRITICAL();
        if (s_printfMutex == NULL) {
            s_printfMutex = xSemaphoreCreateMutex();
        }
        taskEXIT_CRITICAL();
    }
}

UART_Controller s_uart_controller[8];

static USART_TypeDef *const uartInstances[8] = {
#ifdef USE_USART1
    USART1,
#endif
#ifdef USE_USART2
    USART2,
#endif
#ifdef USE_USART3
    USART3,
#endif
#ifdef USE_USART4
    USART4,
#endif
#ifdef USE_USART5
    USART5,
#endif
#ifdef USE_USART6
    USART6,
#endif
#ifdef USE_USART7
    USART7,
#endif
#ifdef USE_USART8
    USART8,
#endif
};


static UART_Controller *get_uart_controller(UART_HandleTypeDef *huart){
    for(uint8_t i = 0;i < 8;i ++){
        if(huart->Instance == uartInstances[i])
        {
            return &s_uart_controller[i];
        }
    }
    return NULL;
}

void UART_Init(UART_HandleTypeDef *huart, UART_Call_Back rxCallBackFunction, uint16_t rxDataLimit){
    UART_Controller *uart_obj = get_uart_controller(huart);
    if (uart_obj != NULL){
        uart_obj->huart = huart;
        uart_obj->rxCallBackFunction = rxCallBackFunction;
        uart_obj->rxDataLimit = rxDataLimit;
        HAL_UARTEx_ReceiveToIdle_DMA(huart,uart_obj->rxBuffer,rxDataLimit);
    }
}

void UART_reinit(UART_HandleTypeDef *huart){
    UART_Controller *uart_obj = get_uart_controller(huart);
    if(uart_obj != NULL){
        HAL_UARTEx_ReceiveToIdle_DMA(huart,uart_obj->rxBuffer,uart_obj->rxDataLimit);
    }
}

HAL_StatusTypeDef UART_SendData(UART_HandleTypeDef *huart, uint8_t *pTxData, uint16_t txDataLength){
    return HAL_UART_Transmit_DMA(huart,pTxData,txDataLength);
}
   
// 全局变量记录Size
volatile uint16_t g_last_size = 0;
volatile uint32_t g_callback_count = 0;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    g_last_size = Size;        // 记录，不打印
    g_callback_count++;
    UART_Controller *uart_obj = get_uart_controller(huart);
    if (uart_obj != NULL) {
        if (uart_obj->rxCallBackFunction != NULL) {
            uart_obj->rxCallBackFunction(uart_obj->rxBuffer, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_obj->rxBuffer, uart_obj->rxDataLimit);
    }
}

/**
 * @brief HAL库UART接收错误中断
 * @param huart UART编号
 * @details 发生错误重启DMA接收
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    UART_Controller *uart_obj = get_uart_controller(huart);
    if (uart_obj != NULL) {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_obj->rxBuffer, uart_obj->rxDataLimit);
        if (uart_obj->rxCallBackFunction != NULL) {
            uart_obj->rxCallBackFunction(NULL, 0);
        }
    }
}

int _write(int file, char *ptr, int len)
{
    (void)file;
    ensure_printf_mutex();
    xSemaphoreTake(s_printfMutex, portMAX_DELAY);
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    xSemaphoreGive(s_printfMutex);
    return len;
}