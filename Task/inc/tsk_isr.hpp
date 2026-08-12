#pragma once

#include "app_rocket.hpp"

extern "C" {
void uart2Callback(uint8_t *pRxData, uint16_t rxDataLength);

void uart1Callback(uint8_t *pRxData, uint16_t rxDataLength);
} // extern "C"