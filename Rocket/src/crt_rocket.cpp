#include "crt_rocket.hpp"

static const char* resultName(Flash::Result r)
{
    switch (r) {
    case Flash::Result::OK:                  return "OK";
    case Flash::Result::INVALID_ARGUMENT:    return "INVALID_ARGUMENT";
    case Flash::Result::OUT_OF_RANGE:        return "OUT_OF_RANGE";
    case Flash::Result::UNALIGNED:           return "UNALIGNED";
    case Flash::Result::NOT_INITIALIZED:     return "NOT_INITIALIZED";
    case Flash::Result::DEVICE_NOT_FOUND:    return "DEVICE_NOT_FOUND";
    case Flash::Result::BUSY:                return "BUSY";
    case Flash::Result::TIME_OUT:            return "TIME_OUT";
    case Flash::Result::COMMUNICATION_ERROR: return "COMMUNICATION_ERROR";
    case Flash::Result::PROGRAM_ERROR:       return "PROGRAM_ERROR";
    case Flash::Result::ERASE_ERROR:         return "ERASE_ERROR";
    case Flash::Result::WRITE_PROTECTED:     return "WRITE_PROTECTED";
    case Flash::Result::VERIFY_FAILED:       return "VERIFY_FAILED";
    default:                                 return "UNKNOWN";
    }
}

extern volatile uint16_t g_last_size;
extern volatile uint32_t g_callback_count;

Rocket::Rocket(IMU *imu, GNSS *gnss, W25Q128 *flash):
    m_imu(imu),
    m_gnss(gnss),
    m_flash(flash)

{}

void Rocket::Init(){
    DWT_Init();
    SPI_BusInit(&hspi1);
    SPI_Init(&hspi1, nullptr);
    UART_Init(&huart1,nullptr,100);
    UART_Init(&huart2,uart2Callback,1024);
    m_imu->init();
    m_flash->Init();

}

void Rocket::rocketTotalLoop(){
    
}


