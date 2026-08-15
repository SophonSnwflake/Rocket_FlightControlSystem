#include "app_rocket.hpp"
#include "mid_logger.hpp"
#include "stm32f4xx_hal_gpio.h"
#include "math_const.h"

#include "stm32f4xx_hal_gpio.h"
#include <cstddef>
#include <cstdint>

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

Rocket::Rocket(IMU *imu, 
            GNSS *gnss, 
            W25Q128 *flash, 
            SX1268 *lora, 
            BMP388 *barometer, 
            ActiveBuzzer *buzzer, 
            RocketLog::FlightLogger *logger, 
            RocketLog::RocketLogger *loggerWriter, 
            RocketCommand *uartCommand,
            Communicator *communicator):
    m_imu(imu),
    m_gnss(gnss),
    m_flash(flash),
    m_lora(lora),
    m_barometer(barometer),
    m_buzzer(buzzer),
    m_logger(logger),
    m_loggerWriter(loggerWriter),
    m_isInitedCompleted(false),
    m_launchPhase(LaunchPhase::STANDBY),
    m_uartCommand(uartCommand),
    m_communicator(communicator)
    

{
    m_logQueue = xQueueCreateStatic(
    LOG_QUEUE_LENGTH,
    sizeof(LogEvent),
    m_logQueueStorage,
    &m_logQueueControlBlock
    );
}

//==============================================================================
// 初始化
//==============================================================================

Rocket::RocketError Rocket::Init(){
    if(m_isInitedCompleted) return RocketError::HasInited;
    DWT_Init();
    SPI_BusInit(&hspi1);
    SPI_BusInit(&hspi2);
    SPI_Init(&hspi1, nullptr);
    UART_Init(&huart1,uart1Callback,1024);
    UART_Init(&huart2,uart2Callback,1024);
    RocketError state;
    bool isDeviceInithasError = false;

    printf("Rocket Flight Control System is Online!\r\n");
    printf("Software Git Hash:\r\n");
    printf("After\r\n");
    printf("4f279cb27f4562ec366c66d3eea6c4bb83fce3f5\r\n");
    
    m_buzzer->handleChipping(true);
    osDelay(80U);
    m_buzzer->handleChipping(false);
    osDelay(80U);
    m_buzzer->handleChipping(true);
    osDelay(80U);
    m_buzzer->handleChipping(false);

    if (initIMU()   != RocketError::OK) return RocketError::DeviceError;
    if (initLoRa()  != RocketError::OK) return RocketError::DeviceError;
    if (initGNSS()  != RocketError::OK) return RocketError::DeviceError;
    if (initFlash() != RocketError::OK) return RocketError::DeviceError;
    
    printf("DeviceInitSuccess!\r\n");
    m_isInitedCompleted = true;
    return RocketError::OK;
}

Rocket::RocketError Rocket::initGNSS(){
    if(m_gnss == nullptr){
        printf("GNSSInitFailed!\r\n");
        return RocketError::DeviceError;
    }
    m_gnss->Init();
    printf("GNSSInitSuccess!\r\n");
    return RocketError::OK;
}

Rocket::RocketError Rocket::initLogger()
{
    if (m_logger == nullptr) {
        return RocketError::DeviceError;
    }

    const uint64_t timestampUs = getTimestampUs();

    RocketLog::FlightLogger::FlightLoggerError loggerstate;
    loggerstate = m_logger->start(timestampUs);

    if(loggerstate != RocketLog::FlightLogger::FlightLoggerError::OK){
        return RocketError::DeviceError;
        printf("LoggerInitFailed!\r\n");
    }

    printf("LoggerInitSuccess!\r\n");
    return RocketError::OK;
}

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

//==============================================================================
// 执行逻辑
//==============================================================================

void Rocket::rocketTotalLoop()
{
    handlePendingUARTCommand();
    phaseSelect();
    parachuteLoop();
    sendFlightTelemetryPayloadLoop();

    m_nowTimeus = getTimestampUs();
}

void Rocket::parachuteLoop(){
    switch(m_launchPhase){
        case LaunchPhase::STANDBY:{
            break;
        }
        case LaunchPhase::ARMED:{
            break;
        }
        case LaunchPhase::ASCENT:{
            break;
        }
        case LaunchPhase::DESCENT:{
            if(m_isParachuteIgnited){
                break;
            }else{
                igniteParachute();
                m_isParachuteIgnited = true;
                break;
            }
            break;
        }
        case LaunchPhase::LANDED:{
            break;
        }

    }
}

void Rocket::imuLoop()
{
    taskENTER_CRITICAL();
    m_eulerAngle = m_imu->solveAttitude();
    taskEXIT_CRITICAL();
    m_rawAccel = m_imu->getAccelRawData();

    switch (m_launchPhase)
    {  
        case LaunchPhase::STANDBY:
        {
            break;
        }

        case LaunchPhase::ARMED:
        case LaunchPhase::ASCENT:
        case LaunchPhase::DESCENT:
        {
            const RSLMath::Vector3f accel =
                m_imu->getAccelRawData();

            const RSLMath::Vector3f gyro =
                m_imu->getGyroRawData();

            LogEvent IMUevent{};

            IMUevent.type = LogEventType::IMU;

            IMUevent.data.imu.timestamp = getTimestampUs();
            IMUevent.data.imu.sequence  = ++m_imuSequence;

            IMUevent.data.imu.accel_raw[0] =
                static_cast<int16_t>(accel[0] * LOGGER_IMU_SCALE_FACTOR);

            IMUevent.data.imu.accel_raw[1] =
                static_cast<int16_t>(accel[1] * LOGGER_IMU_SCALE_FACTOR);

            IMUevent.data.imu.accel_raw[2] =
                static_cast<int16_t>(accel[2] * LOGGER_IMU_SCALE_FACTOR);

            IMUevent.data.imu.gyro_raw[0] =
                static_cast<int16_t>(gyro[0] * LOGGER_IMU_SCALE_FACTOR);

            IMUevent.data.imu.gyro_raw[1] =
                static_cast<int16_t>(gyro[1] * LOGGER_IMU_SCALE_FACTOR);

            IMUevent.data.imu.gyro_raw[2] =
                static_cast<int16_t>(gyro[2] * LOGGER_IMU_SCALE_FACTOR);

            if (xQueueSend( m_logQueue, &IMUevent, 0) != pdPASS)
            {
                ++m_logDroppedCount;
            }

            LogEvent AHRSevent{};
            AHRSevent.type = LogEventType::AHRS;
            AHRSevent.data.ahrs.timestamp_us = getTimestampUs();
            AHRSevent.data.ahrs.gyroBias[0] = m_imu->getGyroBias()[0] * LOGGER_GYRO_BIAS_SCALE_FACTOR;
            AHRSevent.data.ahrs.gyroBias[1] = m_imu->getGyroBias()[1] * LOGGER_GYRO_BIAS_SCALE_FACTOR;
            AHRSevent.data.ahrs.gyroBias[2] = m_imu->getGyroBias()[2] * LOGGER_GYRO_BIAS_SCALE_FACTOR;

            const fp32* quaternionFP32 = m_imu->m_ahrs->getQuaternion();
            for (int i = 0; i < 4; ++i){
                AHRSevent.data.ahrs.quaternion[i] = static_cast<int16_t>(quaternionFP32[i] * LOGGER_QUATERNION_SCALE_FACTOR);
            }
            if (xQueueSend( m_logQueue, &AHRSevent, 0) != pdPASS)
            {
                ++m_logDroppedCount;
            }

            break;
        }

        case LaunchPhase::LANDED:
        {
            break;
        }

        case LaunchPhase::SELF_TEST:
        {
            break;
        }
    }
}

void Rocket::sendFlightTelemetryPayloadLoop(){
    Telemetry::FlightTelemetryPayload payload;
    payload.timeStamp_ms = static_cast<uint32_t>(getTimestampUs() / 1000ULL);
    payload.flight_phase = translateLaunchFhaseIntoFlightPhase(m_launchPhase);

    payload.pitch_centidegree = static_cast<int16_t>((m_eulerAngle[1] * 180.0f/MATH_PI) * 100.0f);
    payload.roll_centidegree = static_cast<int16_t>((m_eulerAngle[0] * 180.0f/MATH_PI) * 100.0f);
    payload.yaw_centidegree = static_cast<int16_t>((m_eulerAngle[2] * 180.0f/MATH_PI) * 100.0f);

    payload.relative_altitude_mm = static_cast<uint32_t>(m_altitude_m * 1000);

    payload.vertical_velocity_mm_s = static_cast<uint32_t>(m_velocity_m_s * 1000);
    m_communicator->sendFlightTelemetryPayload(&payload);
}


void Rocket::loggerLoop()
{
    if (!m_logger->isStarted())
        return;

    LogEvent event{};

    if (xQueueReceive(m_logQueue, &event, pdMS_TO_TICKS(10)) != pdPASS) return;

    switch (event.type)
    {
        case LogEventType::IMU:
        {
            m_logger->writeIMU(&event.data.imu);
            break;
        }

        case LogEventType::GNSS:
        {
            m_logger->writeGNSS(&event.data.gnss);
            break;
        }

        case LogEventType::AHRS:
        {
            m_logger->writeAHRS(&event.data.ahrs);
            break;
        }

        case LogEventType::Power:
        {
            m_logger->writePower(&event.data.power);
            break;
        }

        default:
            break;
    }
}

void Rocket::GNSSLoop(){
    if(!m_gnss->isHasNewData()) return;
    m_gnss->handleGNSSMessageLoop();
    if(m_launchPhase == LaunchPhase::STANDBY || m_launchPhase == LaunchPhase::ARMED) return;
    LogEvent event{};

    event.type = LogEventType::GNSS;
    event.data.gnss.timestamp_us = getTimestampUs();
    event.data.gnss.iTOW_ms = m_gnss->getITOW();
    event.data.gnss.latitude_deg_e7 = m_gnss->getLatitude();
    event.data.gnss.longitude_deg_e7 = m_gnss->getLongitude();
    event.data.gnss.altitude_msl_mm = m_gnss->getAltitude();
    event.data.gnss.velocity_north_mm_s = m_gnss->getVelocityNorth();
    event.data.gnss.velocity_east_mm_s = m_gnss->getVelocityEast();
    event.data.gnss.velocity_down_mm_s = m_gnss->getVelocityDown();
    event.data.gnss.h_accuracy_mm = m_gnss->getHAccuracy();
    event.data.gnss.v_accuracy_mm = m_gnss->getVAccuracy();
    event.data.gnss.speed_accuracy_mm_s = m_gnss->getSpeedAccuracy();
    event.data.gnss.valid_flags = m_gnss->getValid();
    event.data.gnss.fix_type = m_gnss->getFixType();
    event.data.gnss.num_satellites = m_gnss->getNumSatellites();

    if (xQueueSend( m_logQueue, &event, 0) != pdPASS){
        ++m_logDroppedCount;
    }
}

void Rocket::communicationLoop(){
    if(!m_lora->isLoRaBegined()) return;
    m_communicator->CommunicatorLoop();
}



Rocket::RocketError Rocket::eraseAllChipForNewFlight(){
    RocketLog::RocketLogger::FlashLogError state;
    state = m_loggerWriter->prepareNewFlight();
    if(state != RocketLog::RocketLogger::FlashLogError::OK){
        return Rocket::RocketError::DeviceError;
    }
    return Rocket::RocketError::OK;
}

void Rocket::phaseSelect(){
    switch(m_launchPhase){
        case LaunchPhase::STANDBY :{
            getTimestampUs();
            break;
        }
        case LaunchPhase::ARMED :{
            if(isAccelLaunched()){
                m_launchTimeus = getTimestampUs();
                m_launchPhase = LaunchPhase::ASCENT;
                break;
            }
            break;
        }
        case LaunchPhase::ASCENT:{  
            m_nowTimeus = getTimestampUs();
            // 防止由于执行顺序导致的溢出
            if(m_nowTimeus <= m_launchTimeus){
                break;
            }
            if (m_nowTimeus - m_launchTimeus >= PARACHUTE_MAX_WAITING_TIME * 1000000ULL){
                m_launchPhase = LaunchPhase::DESCENT;
                break;
            }
            if (isPitchOurOfCritialPoint()){
                m_pitchParachuteConfirmTimes ++;
                if(m_pitchParachuteConfirmTimes >= PARACHUTE_PITCH_CONFIRM_TIMES){
                    m_launchPhase = LaunchPhase::DESCENT;
                }
                break;
            }else{
                m_pitchParachuteConfirmTimes = 0;
                break;
            }
        }
        case LaunchPhase::DESCENT:{
            break;
        }
        case LaunchPhase::LANDED:{
            break;
        }
    }
}

bool Rocket::setPhaseBetweenSTANDBYandARMED(LaunchPhase launchPhase){
    // 当系统处于飞行阶段时，拒绝切换请求
    if(m_launchPhase != LaunchPhase::ARMED && m_launchPhase != LaunchPhase::STANDBY) return false;
    // 重复操作退出
    if(launchPhase == m_launchPhase){
        return true;
    }
    m_buzzer->handleChipping(true);
    osDelay(80);
    m_buzzer->handleChipping(false);
    m_launchPhase = launchPhase;
    return true;
}

void Rocket::receiveUARTCommandData(const uint8_t* pRxData, uint16_t rxDataLength)
{
    if (pRxData == nullptr || rxDataLength == 0)
        return;

    if (rxDataLength > COMMAND_RX_BUFFER_SIZE)
        return;

    if (m_commandRxPending)
        return;

    memcpy(
        m_commandRxBuffer,
        pRxData,
        rxDataLength
    );

    m_commandRxLength = rxDataLength;

    m_commandRxPending = true;
}

void Rocket::handlePendingUARTCommand(){
    if (!m_commandRxPending)return;
    m_commandRxPending = false;
    m_uartCommand->feed(reinterpret_cast<const char*>(m_commandRxBuffer), static_cast<size_t>(m_commandRxLength));
    
}

void Rocket::setUARTCommand(RocketCommand* command)
{
    m_uartCommand = command;
}


bool Rocket::isAccelLaunched(){
    if(m_rawAccel[2] >= LAUNCH_ACCEL_CRITICAL_VALUE){
        return true;
    }
    return false;
}

bool Rocket::isPitchOurOfCritialPoint(){
    if (m_eulerAngle[0] >= PARACHUTE_PITCH_CRITICAL_POINT){
        return true;
    }
    return false;
}

Rocket::RocketError Rocket::readAllFlashDataThroughUART()
{
    RocketLog::RocketLogger::FlashLogError state;

    const uint32_t dataLength =
        m_loggerWriter->getChipBytesCounts();

    if (dataLength == 0)
    {
        return RocketError::OK;
    }

    constexpr uint32_t READ_CHUNK_SIZE = 256;

    uint8_t buffer[READ_CHUNK_SIZE];

    uint32_t offset = 0;

    while (offset < dataLength)
    {
        uint32_t readLength = READ_CHUNK_SIZE;

        if (dataLength - offset < READ_CHUNK_SIZE)
        {
            readLength = dataLength - offset;
        }

        state = m_loggerWriter->read(
            offset,
            buffer,
            readLength
        );

        if (state !=
            RocketLog::RocketLogger::FlashLogError::OK)
        {
            return RocketError::DeviceError;
        }

        if (HAL_UART_Transmit(
                &huart1,
                buffer,
                static_cast<uint16_t>(readLength),
                1000) != HAL_OK)
        {
            return RocketError::DeviceError;
        }

        offset += readLength;
    }

    return RocketError::OK;
}

void Rocket::igniteParachute(){
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
}

Telemetry::FlightPhase Rocket::translateLaunchFhaseIntoFlightPhase(LaunchPhase launchphase){
    return static_cast<Telemetry::FlightPhase>(launchphase);
}

void Rocket::receiveUARTGNSSData(uint8_t *pRxData, uint16_t rxDataLength){
    m_gnss->receiveGNSSMessageFromUART(pRxData, rxDataLength);
}

uint64_t Rocket::getTimestampUs()
{
    uint64_t totalCyclesSnapshot;

    taskENTER_CRITICAL();

    static uint32_t lastCycle = DWT->CYCCNT;
    static uint64_t totalCycles = 0;

    const uint32_t currentCycle = DWT->CYCCNT;

    const uint32_t elapsedCycles = currentCycle - lastCycle;

    totalCycles += elapsedCycles;
    lastCycle = currentCycle;

    totalCyclesSnapshot = totalCycles;

    taskEXIT_CRITICAL();

    return totalCyclesSnapshot /
           (SystemCoreClock / 1000000ULL);
}