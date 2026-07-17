#pragma once

#include "crt_rocket.hpp"

extern "C" {
void uart2Callback(uint8_t *pRxData, uint16_t rxDataLength);
} // extern "C"