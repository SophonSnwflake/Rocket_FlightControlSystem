#pragma once

#include "FreeRTOS.h" 
#include "task.h" 
#include "RSL_common.h"
#include "adc.h"

class VoltageProbe {
private:
    static constexpr fp32 STATIC_VOLTAGE_REF = 3.3f;
    static constexpr float ADC_MAX_VALUE = 4095.0f;
    static constexpr float DIVIDER_RATIO = 2.0f;
    ADC_HandleTypeDef *m_hadc;

public:
    VoltageProbe(ADC_HandleTypeDef *hadc);
    bool init();
    float readVoltage();
};