#include "app_rocket.hpp"
#include "tsk_isr.hpp"

extern NEOM9N_UART gnss;
extern Rocket rocket;

void uart2Callback(uint8_t *pRxData, uint16_t rxDataLength){
    rocket.receiveUARTGNSSData(pRxData, rxDataLength);
}


void uart1Callback(uint8_t *pRxData, uint16_t rxDataLength){
    rocket.receiveUARTCommandData(pRxData, rxDataLength);
}
