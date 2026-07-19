#include "tsk_isr.hpp"

extern NEOM9N_UART gnss;

void uart2Callback(uint8_t *pRxData, uint16_t rxDataLength){
    gnss.receiveGNSSMessageFromUART(pRxData, rxDataLength);
}
