#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring> 
#include <cstdlib>

#include "std_typedef.h" 
#include "drv_uart.h" 
#include "FreeRTOS.h" 
#include "task.h" 
#include "usart.h"
#include "RSL_common.h"

class VoFa{
public:
    static constexpr size_t PARAMETER_MAX_COUNT = 100;
    uint8_t m_txDataBuffer[PARAMETER_MAX_COUNT * sizeof(fp32) + sizeof(fp32)];

private:
    uint8_t m_parameterCount = 0;
    fp32 parameter[PARAMETER_MAX_COUNT] {};
    UART_HandleTypeDef *m_huart;
public:
    VoFa(uint8_t parameterCount, UART_HandleTypeDef *huart);
    bool setChannel(uint8_t index, fp32 value);
    bool voFaLoop();

};

