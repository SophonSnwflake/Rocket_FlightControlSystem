#include "dvc_vofa.hpp"
#include "drv_uart.h"

VoFa::VoFa(uint8_t parameterCount, UART_HandleTypeDef *huart){
    m_parameterCount = parameterCount;
    m_huart = huart;
}


/**
 * @brief 将一个float压入缓冲区。
 * @param index 缓冲区中的索引。
 * @param value 要压入的float值。
 *
 * @return true 压入成功。
 * @return false 压入失败。
 *
 * @note 对parameter数组的访问可能导致数据竞争，尽量不在多线程内调用
 */
bool VoFa::setChannel(uint8_t index, fp32 value){
    if (m_parameterCount == 0 || m_huart == nullptr || m_parameterCount > PARAMETER_MAX_COUNT || index >= m_parameterCount) {
        return false;
    }
    parameter[index] = value;
    return true;
}

/**
 * @brief voFa发送循环，放在定时循环内调用
 */
bool VoFa::voFaLoop(){
    if (m_parameterCount == 0 || 
        m_huart == nullptr ||
        m_parameterCount > PARAMETER_MAX_COUNT) {
        return false;
    }

    for (uint8_t i = 0; i < m_parameterCount; ++i){
        std::memcpy(&m_txDataBuffer[i * sizeof(fp32)], &parameter[i], sizeof(fp32));
    }

    m_txDataBuffer[m_parameterCount * sizeof(fp32)] = 0x00;
    m_txDataBuffer[m_parameterCount * sizeof(fp32) + 1] = 0x00;
    m_txDataBuffer[m_parameterCount * sizeof(fp32) + 2] = 0x80;
    m_txDataBuffer[m_parameterCount * sizeof(fp32) + 3] = 0x7F;

    HAL_StatusTypeDef result = UART_SendData(m_huart, m_txDataBuffer, m_parameterCount * 4 + 4);

    return result == HAL_OK;

}