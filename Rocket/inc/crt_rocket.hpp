#pragma once
#include "RSL_common.h"
#include "alg_general.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "dvc_imu.hpp"
#include "alg_ahrs.hpp"
#include "drv_uart.h"
#include "dvc_gnss.hpp"
#include "tsk_isr.hpp"
#include "drv_time.h"
#include "dvc_flash.hpp"
#include "dvc_flash.hpp"
#include "dvc_lora.hpp"
#include "dvc_barometer.hpp"
#include "dvc_buzzer.hpp"
#include "mid_logger.hpp"
#include "mid_logger_writer.hpp"
#include <cstdint>

class Rocket
{
public:
    typedef enum {
        SELF_TEST       = 0,
        READY_TO_LAUNCH = 1,
        ERROR           = 2,
        FLYING_UP       = 3,
        LANDING         = 4
    }LaunchPhase;

    enum class RocketError : uint8_t{
        OK = 0,
        CommFail,        
        BadParam, 
        DeviceError,
        HasInited,
        NotInited,
    };


private:
    IMU *m_imu;
    GNSS *m_gnss;
    W25Q128 *m_flash;
    SX1268 *m_lora;
    BMP388 *m_barometer;
    ActiveBuzzer *m_buzzer;
    RocketLog::FlightLogger *m_logger;
    LaunchPhase m_launchPhase;
    
private:
    bool m_isInitedCompleted = false;
    bool m_isLoggerInitCompleted = false;

public:
    Rocket(IMU *imu, GNSS *gnss, W25Q128 *flash, SX1268 *lora, BMP388 *barometer, ActiveBuzzer *buzzer, RocketLog::FlightLogger *logger);
    virtual ~Rocket() = default;
    bool isInitCompleted() {return m_isInitedCompleted;}
    RocketError Init();
    void rocketTotalLoop();
    void imuLoop();
    bool selfTest();

private:
    RocketError initLogger();
    RocketError initLoRa();
    RocketError initFlash();
    RocketError initIMU();
               
};