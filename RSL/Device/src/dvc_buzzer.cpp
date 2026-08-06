#include "dvc_buzzer.hpp"

ActiveBuzzer::ActiveBuzzer(GPIO_TypeDef* port, uint16_t pin, bool initState) : m_port(port), m_pin(pin), m_initState(initState) {
    HAL_GPIO_WritePin(m_port, m_pin, m_initState ? GPIO_PIN_SET : GPIO_PIN_RESET);


}

void ActiveBuzzer::handleChipping(bool startorStop) {
    HAL_GPIO_WritePin(m_port, m_pin, startorStop ? (m_initState ? GPIO_PIN_RESET : GPIO_PIN_SET) : (m_initState ? GPIO_PIN_SET : GPIO_PIN_RESET));
}
