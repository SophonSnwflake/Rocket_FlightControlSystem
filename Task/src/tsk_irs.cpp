#include "tsk_isr.hpp"

extern NEOM9N_UART m_gnss;

void uart2Callback(uint8_t *pRxData, uint16_t rxDataLength){
    m_gnss.receiveGNSSMessageFromUART(pRxData, rxDataLength);
}