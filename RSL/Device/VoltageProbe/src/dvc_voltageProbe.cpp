#include "dvc_voltageProbe.hpp"

// TODO: 引入串口调试，用完删除！
#include "drv_uart.h"
// TODO: 引入串口调试，用完删除！

VoltageProbe::VoltageProbe(ADC_HandleTypeDef *hadc) : 
        m_hadc(hadc) {}

bool VoltageProbe::init()
{
    return m_hadc != nullptr;
}
float VoltageProbe::readVoltage()
{
    if (m_hadc == nullptr){
        return -1.0f;
    }

    if (HAL_ADC_Start(m_hadc) != HAL_OK){
        return -1.0f;
    }

    if (HAL_ADC_PollForConversion(m_hadc, 10) != HAL_OK){
        return -1.0f;
    }

    uint32_t raw = HAL_ADC_GetValue(m_hadc);

    printf("Raw ADC Value: %lu\r\n", raw); // TODO: 引入串口调试，用完删除！

    float adcVoltage = static_cast<float>(raw) / ADC_MAX_VALUE * STATIC_VOLTAGE_REF * DIVIDER_RATIO;

    return adcVoltage;
}