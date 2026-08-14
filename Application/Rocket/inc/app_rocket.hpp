#pragma once
#include "RSL_common.h"
#include "alg_general.hpp"
#include "FreeRTOS.h"
#include "app_command.hpp"
#include "math_const.h"
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
#include "mid_logger.hpp"
#include "queue.h"
#include <cstdint>

class RocketCommand; 

class Rocket
{
public:
    static constexpr size_t COMMAND_RX_BUFFER_SIZE = 256;
    static constexpr uint32_t LOG_QUEUE_LENGTH = 64;
    static constexpr fp32 LAUNCH_ACCEL_CRITICAL_VALUE = 10.0f;
    static constexpr fp32 PARACHUTE_PITCH_CRITICAL_POINT = 120.0f/90.0f * MATH_PI;   // 以大地为坐标系，背地朝天为0 rad，背天朝地为PI rad。
    static constexpr fp32 PARACHUTE_MAX_WAITING_TIME = 10.0f; // 最晚开伞时间，单位秒
    static constexpr uint16_t PARACHUTE_PITCH_CONFIRM_TIMES = 10;
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

    enum class LogEventType : uint8_t
    {
        IMU,
        GNSS,
        AHRS,
        FlightEstimate,
        FlightState,
        Power,
        SystemHealth
    };

    struct LogEvent
    {
        LogEventType type;

        union
        {
            IMURawMessage imu;
            GNSSMessage gnss;
            AHRSMessage ahrs;
            FlightEstimateMessage flightEstimate;
            FlightStateMessage flightState;
            PowerMessage power;
            SystemHealthMessage systemHealth;
        } data;
    };


private:
    IMU *m_imu;
    GNSS *m_gnss;
    W25Q128 *m_flash;
    SX1268 *m_lora;
    BMP388 *m_barometer;
    ActiveBuzzer *m_buzzer;
    RocketLog::FlightLogger *m_logger;
    RocketLog::RocketLogger *m_loggerWriter;
    RocketCommand *m_uartCommand;
    LaunchPhase m_launchPhase = LaunchPhase::STANDBY;
    LaunchPhase m_lastLaunchPhase = LaunchPhase::STANDBY;

    StaticQueue_t m_logQueueControlBlock;
    uint8_t m_logQueueStorage[LOG_QUEUE_LENGTH * sizeof(LogEvent)];
    QueueHandle_t m_logQueue;

    
private:
    uint64_t m_launchTimeus = 0;
    uint64_t m_nowTimeus = 0;
    uint16_t m_pitchParachuteConfirmTimes = 0;
    IMURawMessage m_imuMessage; 
    RSLMath::Vector3f m_rawAccel;
    RSLMath::Vector3f m_eulerAngle;
    uint32_t m_imuSequence = 0;
    uint32_t m_logDroppedCount = 0;
    uint32_t m_loggerErrorCount = 0;
    bool m_isInitedCompleted = false;
    bool m_isParachuteIgnited = false;
    uint8_t m_commandRxBuffer[COMMAND_RX_BUFFER_SIZE];
    volatile uint16_t m_commandRxLength = 0;
    volatile bool m_commandRxPending = false;

public:
    Rocket(IMU *imu, GNSS *gnss, W25Q128 *flash, SX1268 *lora, BMP388 *barometer, ActiveBuzzer *buzzer, RocketLog::FlightLogger *logger, RocketLog::RocketLogger *loggerWriter, RocketCommand *uartCommand);
    virtual ~Rocket() = default;
    RocketError Init();
    bool isInitCompleted() {return m_isInitedCompleted;}
    bool isAccelLaunched();
    bool isPitchOurOfCritialPoint();
    void phaseSelect();
    void rocketTotalLoop();
    void imuLoop();
    void loggerLoop();
    LaunchPhase getPhase(){return m_launchPhase;}
    bool setPhaseBetweenSTANDBYandARMED(LaunchPhase launchPhase);
    void setUARTCommand(RocketCommand* command);
    RocketError initLogger();
    RocketError eraseAllChipForNewFlight();
    RocketError readAllFlashDataThroughUART();

// 通信回调
    void handlePendingUARTCommand();
    void receiveUARTCommandData(const uint8_t *pRxData, uint16_t rxDataLength);
    
private:
    void igniteParachute();
    void parachuteLoop();
    RocketError initLoRa();
    RocketError initFlash();
    RocketError initIMU();
    uint64_t getTimestampUs();
               
};