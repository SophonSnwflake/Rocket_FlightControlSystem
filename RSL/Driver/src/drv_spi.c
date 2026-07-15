#include "drv_spi.h"

SPI_Controller_t spiController[6] = {0};

static SPI_TypeDef *const SPI_Instances[6] ={
#ifdef USE_SPI1
    SPI1,
#endif
#ifdef USE_SPI2
    SPI2,
#endif
#ifdef USE_SPI3
    SPI3,
#endif
#ifdef USE_SPI4
    SPI4,
#endif
#ifdef USE_SPI5
    SPI5,
#endif
#ifdef USE_SPI6
    SPI6,
#endif
};

static SPI_Controller_t *SPI_GetManageObject(SPI_HandleTypeDef *hspi){
    for(uint8_t i = 0; i < 6; i++){
        if(SPI_Instances[i] == hspi->Instance){
            return &spiController[i];
        }
    }
    return NULL;
}

void SPI_Init(SPI_HandleTypeDef *hspi, SPI_Callback callback){
    SPI_Controller_t *spi_controller = SPI_GetManageObject(hspi);
    if(spi_controller != NULL){
        spi_controller->hspi = hspi;
        spi_controller->callback = callback;
    }
}

void SPI_SwitchCallBackFunction(SPI_HandleTypeDef *hspi, SPI_Callback callback){
    SPI_Controller_t *spi_controller = SPI_GetManageObject(hspi);
    if(spi_controller != NULL){
        spi_controller->callback = callback;
    }
}

HAL_StatusTypeDef SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t length,uint32_t timeout){
    SPI_Controller_t *spi_controller = SPI_GetManageObject(hspi);
    if(spi_controller != NULL){
        return HAL_SPI_Transmit(hspi, pData,length,timeout);
    }
    return HAL_ERROR;
}

HAL_StatusTypeDef SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t length, uint32_t timeout){
    SPI_Controller_t *spi_controller = SPI_GetManageObject(hspi);
    if(spi_controller != NULL){
        return HAL_SPI_Receive(hspi,pData,length,timeout);
    }
    return HAL_ERROR;
}

HAL_StatusTypeDef SPI_TransmitReceive(SPI_HandleTypeDef *hspi, const uint8_t *pTxData, uint8_t *pRxData,uint16_t length, uint32_t timeout){
    SPI_Controller_t *spi_controller = SPI_GetManageObject(hspi);
    if(spi_controller != NULL){
        return HAL_SPI_TransmitReceive(hspi, pTxData,pRxData,length,timeout);
    }
    return HAL_ERROR;
}

HAL_StatusTypeDef SPI_Transmit_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t length){
    SPI_Controller_t *spi_controller = SPI_GetManageObject(hspi);
    if(spi_controller != NULL){
        return HAL_SPI_Transmit_IT(hspi, pData, length);
    }
    return HAL_ERROR;
}

HAL_StatusTypeDef SPI_Receive_IT(SPI_HandleTypeDef *hspi, uint16_t length){
    SPI_Controller_t *spi_controller = SPI_GetManageObject(hspi);
    if(spi_controller != NULL){
        return HAL_SPI_Receive_IT(hspi, spi_controller->rxBuffer, length);
    }
    return HAL_ERROR;
}
HAL_StatusTypeDef SPI_TransmitReceive_IT(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t length){
    SPI_Controller_t *spi_controller = SPI_GetManageObject(hspi);
    if(spi_controller != NULL){
        return HAL_SPI_TransmitReceive_IT(hspi, pTxData, pRxData, length);
    }
    return HAL_ERROR;
}


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
    SPI_Controller_t *spi_controller = SPI_GetManageObject(hspi);
    if(spi_controller != NULL && spi_controller->callback != NULL){
        spi_controller->callback(spi_controller->rxBuffer, hspi->RxXferSize);
    }
}