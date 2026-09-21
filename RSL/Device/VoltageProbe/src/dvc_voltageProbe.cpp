#include "dvc_voltageProbe.hpp"


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


    float adcVoltage = static_cast<float>(raw) / ADC_MAX_VALUE * STATIC_VOLTAGE_REF * DIVIDER_RATIO;

    return adcVoltage;
}