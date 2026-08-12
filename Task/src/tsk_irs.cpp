#include "app_rocket.hpp"
#include "tsk_isr.hpp"

extern NEOM9N_UART gnss;
extern Rocket rocket;

void uart2Callback(uint8_t *pRxData, uint16_t rxDataLength){
    gnss.receiveGNSSMessageFromUART(pRxData, rxDataLength);
}

// 调试用
volatile uint32_t uart1RxCount = 0;
// 调试用

void uart1Callback(uint8_t *pRxData, uint16_t rxDataLength){
    ++uart1RxCount;
    rocket.handleUARTmessageForCommand(pRxData, rxDataLength);
}
