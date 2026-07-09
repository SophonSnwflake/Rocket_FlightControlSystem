/**
 ******************************************************************************
 * @file           : drv_spi.c
 * @brief          : SPI驱动
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

 #include "drv_spi.h"

SPI_Controller_t s_spi_controller_objects[6] = {0}; // SPI管理对象

 /**
 * @brief SPI管理实例数组
 * @note 根据board_config.h中的宏定义配置决定使用的SPI实例
 */
static SPI_TypeDef *const spiInstances[6] = {
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
    SPI6
#endif
};

/**
 * @brief 获取SPI对象
 * @param hspi SPI句柄
 * @return SPI管理对象指针
 */
static SPI_Controller_t *SPI_GetManageObject(SPI_HandleTypeDef *hspi){
    for (int i = 0;i<6;i++)
    {
        if(hspi->Instance == spiInstances[i]){
            return &s_spi_controller_objects[i];
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

HAL_StatusTypeDef SPI_TransmitReceive_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t length){
    SPI_Controller_t *spi_controller = SPI_GetManageObject(hspi);
    if(spi_controller != NULL){
        return HAL_SPI_TransmitReceive_IT(hspi, pData, spi_controller->rxBuffer, length);
    }
    return HAL_ERROR;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
    SPI_Controller_t *spi_controller = SPI_GetManageObject(hspi);
    if(spi_controller != NULL && spi_controller->callback != NULL){
        spi_controller->callback(spi_controller->rxBuffer, spi_controller->rxDataLength);
    }
}