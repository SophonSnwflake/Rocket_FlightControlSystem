#ifndef __DRV_SPI_H
#define __DRV_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "RSL_common.h"
#include "board_config.h"
#include "spi.h"


#define RX_BUFFER_SIZE 256

typedef void (*SPI_Callback)(uint8_t *pData, uint16_t length);

typedef struct{
    SPI_HandleTypeDef *hspi;
    uint8_t rxBuffer[RX_BUFFER_SIZE];
    uint16_t rxLength;
    SPI_Callback callback;
}SPI_Controller_t;

void SPI_Init(SPI_HandleTypeDef *hspi, SPI_Callback callback);
void SPI_SwitchCallBackFunction(SPI_HandleTypeDef *hspi, SPI_Callback callback);
HAL_StatusTypeDef SPI_Transmit(SPI_HandleTypeDef *hspi, const uint8_t *pData, uint16_t length, uint32_t timeout);
HAL_StatusTypeDef SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t length, uint32_t timeout);
HAL_StatusTypeDef SPI_TransmitReceive(SPI_HandleTypeDef *hspi, const uint8_t *pTxData, uint8_t *pRxData,uint16_t length,   uint32_t timeout);

HAL_StatusTypeDef SPI_Transmit_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t length);
HAL_StatusTypeDef SPI_Receive_IT(SPI_HandleTypeDef *hspi, uint16_t length);
HAL_StatusTypeDef SPI_TransmitReceive_IT(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint16_t length);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);

//总线锁
void              SPI_BusInit(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef SPI_BusLock(SPI_HandleTypeDef *hspi, uint32_t timeoutMs);
void              SPI_BusUnlock(SPI_HandleTypeDef *hspi);


#ifdef __cplusplus
}
#endif

#endif