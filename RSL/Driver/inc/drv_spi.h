/**
 ******************************************************************************
 * @file           : drv_spi.c
 * @brief          : SPI驱动
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef __DRV_SPI_H
#define __DRV_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "RSL_common.h"
#include "spi.h"

#define SPI_BUFFER_SIZE 256 // SPI接收缓冲区最大长度

typedef void (*SPI_Callback)(uint8_t *pData, uint16_t length);

typedef struct {
    SPI_HandleTypeDef *hspi;
    uint8_t rxBuffer[SPI_BUFFER_SIZE]; // 接收缓冲区
    uint16_t rxDataLength;
    SPI_Callback callback;
} SPI_Controller_t;

void SPI_Init(SPI_HandleTypeDef *hspi, SPI_Callback callback);
void SPI_SwitchCallBackFunction(SPI_HandleTypeDef *hspi, SPI_Callback callback);
HAL_StatusTypeDef SPI_Transmit_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t length);
HAL_StatusTypeDef SPI_Receive_IT(SPI_HandleTypeDef *hspi, uint16_t length);
HAL_StatusTypeDef SPI_TransmitReceive_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_SPI_H */