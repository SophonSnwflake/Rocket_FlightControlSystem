#pragma once
#include "RSL_common.h"
#include "alg_general.hpp"
#include "FreeRTOS.h"
#include "app_command.hpp"
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
#include "app_logger.hpp"
#include "mid_logger_writer.hpp"
#include <cstdint>

class RocketCommand; 

class Rocket
{
public:
    static constexpr size_t COMMAND_RX_BUFFER_SIZE = 256;
    enum class LaunchPhase : uint8_t
    {
        STANDBY = 0,   // 待命
        SELF_TEST,     // 系统自检
        ARMED,         // 系统已经武装，检测到起飞条件就进入飞行逻辑
        ASCENT,        // 上升段
        DESCENT,       // 下降段
        LANDED         // 落地
    };

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
    RocketCommand *m_uartCommand;
    LaunchPhase m_launchPhase = LaunchPhase::STANDBY;
    
private:
    bool m_isInitedCompleted = false;
    bool m_isLoggerInitCompleted = false;
    uint8_t m_commandRxBuffer[COMMAND_RX_BUFFER_SIZE];
    volatile uint16_t m_commandRxLength = 0;
    volatile bool m_commandRxPending = false;

public:
    Rocket(IMU *imu, GNSS *gnss, W25Q128 *flash, SX1268 *lora, BMP388 *barometer, ActiveBuzzer *buzzer, RocketLog::FlightLogger *logger, RocketCommand *uartCommand);
    virtual ~Rocket() = default;
    bool isInitCompleted() {return m_isInitedCompleted;}
    RocketError Init();
    void rocketTotalLoop();
    void imuLoop();
    bool selfTest();
    LaunchPhase getPhase(){return m_launchPhase;}
    bool setPhase(LaunchPhase launchPhase);
    void setUARTCommand(RocketCommand* command);
    
// 通信回调
    void handleUARTmessageForCommand(const uint8_t *pRxData, uint16_t rxDataLength);
private:
    RocketError initLogger();
    RocketError initLoRa();
    RocketError initFlash();
    RocketError initIMU();
               
};