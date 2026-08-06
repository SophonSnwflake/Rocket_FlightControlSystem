#pragma once

#include "RSL_common.h"
#include "cmsis_os.h"
#include "stm32f411xe.h"
#include "cmsis_os2.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "drv_time.h"

class ActiveBuzzer {
private:
    GPIO_TypeDef* m_port;
    uint16_t m_pin;
    bool m_initState;
public:
    ActiveBuzzer(GPIO_TypeDef* port, uint16_t pin, bool initState = true);
    virtual ~ActiveBuzzer() = default;
    void handleChipping(bool startorStop);
};