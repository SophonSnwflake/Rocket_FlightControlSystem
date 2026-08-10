#include "app_rocket.hpp"
#include "stm32f4xx_hal_gpio.h"

#include "stm32f4xx_hal_gpio.h"

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

Rocket::Rocket(IMU *imu, GNSS *gnss, W25Q128 *flash, SX1268 *lora, BMP388 *barometer, ActiveBuzzer *buzzer, RocketLog::FlightLogger *logger):
    m_imu(imu),
    m_gnss(gnss),
    m_flash(flash),
    m_lora(lora),
    m_barometer(barometer),
    m_buzzer(buzzer),
    m_logger(logger),
    m_isInitedCompleted(false),
    m_isLoggerInitCompleted(false)
    

{}

Rocket::RocketError Rocket::Init(){
    if(m_isInitedCompleted) return RocketError::HasInited;
    DWT_Init();
    SPI_BusInit(&hspi1);
    SPI_BusInit(&hspi2);
    SPI_Init(&hspi1, nullptr);
    UART_Init(&huart1,nullptr,100);
    UART_Init(&huart2,uart2Callback,1024);
    RocketError state;
    bool isDeviceInithasError = false;

    printf("Rocket Flight Control System is Online!\r\n");
    printf("Software Git Hash:\r\n");
    printf("After\r\n");
    printf("5dda80503932436286b3e6a3f249b4f85334145d\r\n");
    
    m_buzzer->handleChipping(true);
    osDelay(80U);
    m_buzzer->handleChipping(false);
    osDelay(80U);
    m_buzzer->handleChipping(true);
    osDelay(80U);
    m_buzzer->handleChipping(false);

    uint8_t initTimes = 0;


    while(isDeviceInithasError != true && initTimes < 5){
    // state = initIMU();
    // if (state != RocketError::OK){isDeviceInithasError = true;}

    state = initLoRa();
    if (state != RocketError::OK){isDeviceInithasError = true;}

    state = initFlash();
    if (state != RocketError::OK){isDeviceInithasError = true;}
    
    initTimes ++;
    printf("initTime:%d!\r\n", initTimes);
    }

    if (isDeviceInithasError != true){
        printf("DeviceInitFailed!\r\n");
        return RocketError::DeviceError;
    }
    else{
        m_isInitedCompleted = true;
        printf("DeviceInitSuccess!\r\n");
        return RocketError::OK;
    }
    
}

// Rocket::RocketError Rocket::initLogger(){
//     m_logger->
// }

Rocket::RocketError Rocket::initIMU(){
    if(m_imu->init()){
        printf("IMUInitSuccess!\r\n");
        return RocketError::OK;
    }else{
        printf("IMUInitFailed!\r\n");
        return RocketError::DeviceError;
    }
}

Rocket::RocketError Rocket::initFlash(){
    Flash::Result flashState;
    flashState = m_flash->Init();
    if(flashState == Flash::Result::OK){
        printf("FlashInitSuccess!\r\n");
        return RocketError::OK;
    }else{
        printf("FlashInitFailed!\r\n");
        return RocketError::DeviceError;
    }
}

Rocket::RocketError Rocket::initLoRa(){
    LoRa::ConfigLoRa_t loraConfig{
    434.0f,                 // frequency
    125.0f,                 // bandwidthKhz
    9U,                     // spreadingFactor
    7U,                     // codingRate
    LORA_SYNC_WORD_PRIVATE, // syncWord
    10,                     // power
    8U                      // preambleLength
    };
    LoRa::LoraError LoRaState;
    LoRaState = m_lora->beginLoRa(loraConfig);
    if(LoRaState == LoRa::LoraError::OK){
        printf("LoraInitSuccess!\r\n");
        return RocketError::OK;
    }else{
        printf("LoraInitFailed!\r\n");
        return RocketError::DeviceError;
    }
}

bool i;

void Rocket::rocketTotalLoop()
{
    // m_imu->solveAttitude();

    static const uint8_t message[] = "Hello SX1268";

    LoRa::LoraError error =
        m_lora->transmit(message, sizeof(message), 0x00U);

    if (error == LoRa::LoraError::OK) {
        printf("TX done\r\n");
    } else {
        printf("TX error: %u\r\n", static_cast<unsigned>(error));
    }

    printf("Hello!\r\n");

    // m_buzzer->handleChipping(i);
    // osDelay(1000U);
}

void Rocket::imuLoop(){
    m_imu->solveAttitude();
}


