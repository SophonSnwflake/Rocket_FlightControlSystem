#include "dvc_barometer.hpp"
#include "drv_spi.h"


BMP388::BMP388(BMP388_HandleTypeDef handleTypeDef, BMP388Config config) :
    m_initialized(false),
    m_calibrationValid(false),
    m_busTimeoutCount(0U),
    m_config(config)
{
    m_handleTypeDef = handleTypeDef;   
}

Barometer::BarometerError BMP388::init(){
    m_initialized = false;
    uint8_t chipId = 0;


    for (uint32_t i = 0; i < 1000U; i++)
    {
    uint8_t chipId = 0U;

    const auto result =
        readRegister(REG_CHIP_ID, &chipId);

    printf(
        "result=%u, chipId=0x%02X\r\n",
        static_cast<unsigned int>(result),
        static_cast<unsigned int>(chipId));

    HAL_Delay(10U);
    }
    // BARO_TRY(readRegister(REG_CHIP_ID, &chipId));

    if (chipId != CHIP_ID_BMP388) {
        return BarometerError::DEVICE_NOT_FOUND;
    }

    BARO_TRY(softReset());
    BARO_TRY(readCalibration());
    BARO_TRY(configure(m_config));
    BARO_TRY(setMode(PowerMode::Normal));

    m_initialized = true;
    return BarometerError::OK;
}

Barometer::BarometerError BMP388::softReset(){
   uint8_t commandErrorStatus = 0U;

    // 等待芯片准备接收命令。
    BARO_TRY(waitCommandReady());

    // 向 CMD 寄存器写入软件复位命令 0xB6。
    BARO_TRY(writeRegister(REG_CMD, SOFT_RESET));

    // Bosch 官方驱动在复位命令后等待 2 ms。
    if (osKernelGetState() == osKernelRunning){
            osDelay(2U);
    }
    else{
         HAL_Delay(2U);
    }
    

    // 检查芯片是否报告命令执行错误。
    BARO_TRY(readRegister(REG_ERR, &commandErrorStatus));

    if ((commandErrorStatus & ERR_CMD) != 0U)
    {
        return BarometerError::DEVICE_ERROR;
    }

    return BarometerError::OK;
}

/**
 * @brief 等待 BMP388 准备接收命令。
 * @return 就绪返回 OK，超时返回 DEVICE_NOT_READY。
 */
Barometer::BarometerError BMP388::waitCommandReady()
{
    constexpr uint32_t timeoutMs = 10U;
    const uint32_t startTick = HAL_GetTick();

    while ((HAL_GetTick() - startTick) < timeoutMs)
    {
        uint8_t status = 0;

        BARO_TRY(readRegister(REG_SENS_STATUS, &status));

        if ((status & CMD_RDY) != 0U)
        {
            return BarometerError::OK;
        }

        if (osKernelGetState() == osKernelRunning)
            {
                osDelay(1U);
            }
            else
            {
                HAL_Delay(1U);
            }
        }

    return BarometerError::DEVICE_NOT_READY;
}

Barometer::BarometerError BMP388::readCalibration(){
    uint8_t reg_addr = REG_CALIB_DATA;
    m_calibrationValid = false;
    uint8_t calib_data[LEN_CALIB_DATA] = { 0 };
    BARO_TRY(readRegisters(reg_addr, calib_data, LEN_CALIB_DATA));
    parseCalibrationData(calib_data);
    m_calibrationValid = true;
    return BarometerError::OK;
    
}

void BMP388::parseCalibrationData(const uint8_t* regData)
{
    /*
     * 这是private函数，而且只由readCalibration()传入
     * 固定大小的局部数组，因此这里不必重复检查nullptr。
     */

    const uint16_t rawT1 =
        combineBytes(regData[1], regData[0]);

    const uint16_t rawT2 =
        combineBytes(regData[3], regData[2]);

    const int8_t rawT3 =
        static_cast<int8_t>(regData[4]);

    const int16_t rawP1 =
        static_cast<int16_t>(
            combineBytes(regData[6], regData[5])
        );

    const int16_t rawP2 =
        static_cast<int16_t>(
            combineBytes(regData[8], regData[7])
        );

    const int8_t rawP3 =
        static_cast<int8_t>(regData[9]);

    const int8_t rawP4 =
        static_cast<int8_t>(regData[10]);

    const uint16_t rawP5 =
        combineBytes(regData[12], regData[11]);

    const uint16_t rawP6 =
        combineBytes(regData[14], regData[13]);

    const int8_t rawP7 =
        static_cast<int8_t>(regData[15]);

    const int8_t rawP8 =
        static_cast<int8_t>(regData[16]);

    const int16_t rawP9 =
        static_cast<int16_t>(
            combineBytes(regData[18], regData[17])
        );

    const int8_t rawP10 =
        static_cast<int8_t>(regData[19]);

    const int8_t rawP11 =
        static_cast<int8_t>(regData[20]);

    /*
     * 以下换算系数来自Bosch浮点补偿分支。
     * 不要随意修改，否则后续补偿公式将失效。
     */
    m_calibData.parT1 =
        static_cast<double>(rawT1) / 0.00390625;

    m_calibData.parT2 =
        static_cast<double>(rawT2) / 1073741824.0;

    m_calibData.parT3 =
        static_cast<double>(rawT3) / 281474976710656.0;

    m_calibData.parP1 =
        static_cast<double>(rawP1 - 16384) / 1048576.0;

    m_calibData.parP2 =
        static_cast<double>(rawP2 - 16384) / 536870912.0;

    m_calibData.parP3 =
        static_cast<double>(rawP3) / 4294967296.0;

    m_calibData.parP4 =
        static_cast<double>(rawP4) / 137438953472.0;

    m_calibData.parP5 =
        static_cast<double>(rawP5) / 0.125;

    m_calibData.parP6 =
        static_cast<double>(rawP6) / 64.0;

    m_calibData.parP7 =
        static_cast<double>(rawP7) / 256.0;

    m_calibData.parP8 =
        static_cast<double>(rawP8) / 32768.0;

    m_calibData.parP9 =
        static_cast<double>(rawP9) / 281474976710656.0;

    m_calibData.parP10 =
        static_cast<double>(rawP10) / 281474976710656.0;

    m_calibData.parP11 =
        static_cast<double>(rawP11) /
        36893488147419103232.0;

    m_calibData.tLin = 0.0;
}

/**
 * @brief 配置 BMP388 的过采样率、输出数据率、IIR 滤波器和传感器使能状态。
 * @param config BMP388 运行配置。
 * @return 配置成功返回 OK，否则返回具体错误。
 */
Barometer::BarometerError BMP388::configure(const BMP388Config& config){
    BARO_TRY(validateConfig(config));
    BARO_TRY(setOversampling(
        config.pressureOversampling,
        config.temperatureOversampling));

    BARO_TRY(setOutputDataRate(config.outputDataRate));
    BARO_TRY(setIIRFilter(config.iirFilter));
    BARO_TRY(enableSensors(true, true));

    m_config = config;
    return BarometerError::OK;
}


/**
 * @brief 检查 BMP388 配置字段及 OSR、ODR 组合是否合法。
 * @param config 待检查的 BMP388 配置。
 * @return 配置合法返回 OK，否则返回 BAD_PARAM。
 */
Barometer::BarometerError BMP388::validateConfig(
    const BMP388Config& config) const
{
    const uint8_t pressureOsr =
        static_cast<uint8_t>(config.pressureOversampling);

    const uint8_t temperatureOsr =
        static_cast<uint8_t>(config.temperatureOversampling);

    const uint8_t odr =
        static_cast<uint8_t>(config.outputDataRate);

    const uint8_t iir =
        static_cast<uint8_t>(config.iirFilter);

    // 先检查各枚举的寄存器编码是否合法。
    if ((pressureOsr > MAX_OVERSAMPLING_CODE) ||
        (temperatureOsr > MAX_OVERSAMPLING_CODE) ||
        (odr > MAX_ODR_CODE) ||
        (iir > MAX_IIR_FILTER_CODE))
    {
        return Barometer::BarometerError::BAD_PARAM;
    }

    // ODR 编码 0x00～0x11 对应的输出周期，单位为微秒。
    static constexpr uint32_t odrPeriodUs[18] = {
        5000U,       // 0x00：200 Hz
        10000U,      // 0x01：100 Hz
        20000U,      // 0x02：50 Hz
        40000U,      // 0x03：25 Hz
        80000U,      // 0x04：12.5 Hz
        160000U,     // 0x05：6.25 Hz
        320000U,     // 0x06：3.125 Hz
        640000U,     // 0x07：1.5625 Hz
        1280000U,    // 0x08：0.78125 Hz
        2560000U,    // 0x09：0.390625 Hz
        5120000U,    // 0x0A：0.1953125 Hz
        10240000U,   // 0x0B：0.09765625 Hz
        20480000U,   // 0x0C：0.048828125 Hz
        40960000U,   // 0x0D：0.0244140625 Hz
        81920000U,   // 0x0E：0.01220703125 Hz
        163840000U,  // 0x0F：0.006103515625 Hz
        327680000U,  // 0x10：0.0030517578125 Hz
        655360000U   // 0x11：0.00152587890625 Hz
    };

    /*
     * 过采样编码：
     * 0 -> ×1
     * 1 -> ×2
     * 2 -> ×4
     * 3 -> ×8
     * 4 -> ×16
     * 5 -> ×32
     */
    const uint32_t pressureSamples =
        static_cast<uint32_t>(1UL << pressureOsr);

    const uint32_t temperatureSamples =
        static_cast<uint32_t>(1UL << temperatureOsr);

    const uint32_t pressureMeasurementTimeUs =
        PRESS_SETTLE_TIME_US +
        pressureSamples * ADC_CONVERSION_TIME_US;

    const uint32_t temperatureMeasurementTimeUs =
        TEMP_SETTLE_TIME_US +
        temperatureSamples * ADC_CONVERSION_TIME_US;

    /*
     * configure() 固定调用 enableSensors(true, true)，
     * 因此压力与温度的测量时间都必须计入。
     */
    const uint32_t totalMeasurementTimeUs =
        MEAS_OVERHEAD_US +
        pressureMeasurementTimeUs +
        temperatureMeasurementTimeUs;

    // 必须在下一个 ODR 周期到来前完成当前测量。
    if (totalMeasurementTimeUs >= odrPeriodUs[odr])
    {
        return Barometer::BarometerError::BAD_PARAM;
    }

    return Barometer::BarometerError::OK;
}

Barometer::BarometerError BMP388::setMode(BMP388::PowerMode mode){
    const uint8_t modeValue = static_cast<uint8_t>(mode);
    const uint8_t sleepValue = static_cast<uint8_t>(BMP388::PowerMode::Sleep);
    const uint8_t forcedValue = static_cast<uint8_t>(BMP388::PowerMode::Forced);
    const uint8_t normalValue = static_cast<uint8_t>(BMP388::PowerMode::Normal);
    if ((modeValue != sleepValue) && (modeValue != forcedValue) && (modeValue != normalValue))
    {
        return Barometer::BarometerError::BAD_PARAM;
    }
    uint8_t regValue = 0;
    BARO_TRY(readRegister(REG_PWR_CTRL, &regValue));
    const uint8_t currentMode = static_cast<uint8_t>((regValue & OP_MODE_MASK) >> OP_MODE_POS);
    /*
     * Bosch 的流程要求：
     * 从 Forced 或 Normal 切换模式时，必须先进入 Sleep。
     */
    if (currentMode != sleepValue)
    {
        // 清除 bit 5:4，即设置为 Sleep 模式。
        regValue = static_cast<uint8_t>(regValue & static_cast<uint8_t>(~OP_MODE_MASK));

        BARO_TRY(writeRegister(REG_PWR_CTRL, regValue));

        // 等待芯片稳定进入 Sleep 模式。
        (void)osDelay(MODE_TRANSITION_DELAY_MS);
    }
     /*
     * 如果目标模式本身就是 Sleep，到这里已经完成。
     * 如果原来也是 Sleep，则不需要执行任何写操作。
     */
    if (mode == PowerMode::Sleep)
    {
        return Barometer::BarometerError::OK;
    }

    /*
     * Normal 模式下，OSR 与 ODR 必须能够匹配。
     * Forced 模式不依赖持续输出周期，因此不检查 ODR 组合。
     */
    if (mode == PowerMode::Normal)
    {
        BARO_TRY(validateConfig(m_config));
    }

    /*
     * 重新读取 PWR_CTRL。
     * 这样可以保留 press_en、temp_en 等其他字段的实际值。
     */
    BARO_TRY(readRegister(REG_PWR_CTRL, &regValue));

    // 清除旧的工作模式。
    regValue = static_cast<uint8_t>(
        regValue &
        static_cast<uint8_t>(~OP_MODE_MASK));

    // 将新模式编码写入 bit 5:4。
    regValue = static_cast<uint8_t>(
        regValue |
        ((modeValue << OP_MODE_POS) & OP_MODE_MASK));

    BARO_TRY(writeRegister(REG_PWR_CTRL, regValue));

    /*
     * Bosch 原驱动进入 Normal 后会检查 ERR 寄存器，
     * 确认芯片是否报告 OSR/ODR 配置错误。
     */
    if (mode == PowerMode::Normal)
    {
        uint8_t errorStatus = 0;

        BARO_TRY(readRegister(REG_ERR, &errorStatus));

        if ((errorStatus & ERR_CONF_MASK) != 0U)
        {
            return Barometer::BarometerError::DEVICE_ERROR;
        }
    }

    return Barometer::BarometerError::OK;
}

/**
 * @brief 设置压力和温度的过采样倍率。
 * @param pressureOversampling 压力过采样倍率。
 * @param temperatureOversampling 温度过采样倍率。
 * @return 设置成功返回 OK，否则返回具体错误。
 */
Barometer::BarometerError BMP388::setOversampling(
    Oversampling pressureOversampling,
    Oversampling temperatureOversampling)
{
    const uint8_t pressureValue =
        static_cast<uint8_t>(pressureOversampling);

    const uint8_t temperatureValue =
        static_cast<uint8_t>(temperatureOversampling);

    // BMP388 只支持 ×1、×2、×4、×8、×16、×32，
    // 对应寄存器编码 0x00～0x05。
    if ((pressureValue > 0x05U) ||
        (temperatureValue > 0x05U))
    {
        return Barometer::BarometerError::BAD_PARAM;
    }

    uint8_t regValue = 0;

    BARO_TRY(readRegister(REG_OSR, &regValue));

    // 清除原来的 osr_p 和 osr_t，同时保留 bit 7:6。
    regValue = static_cast<uint8_t>(
        regValue &
        static_cast<uint8_t>(~(PRESS_OSR_MASK | TEMP_OSR_MASK)));

    // 压力过采样写入 bit 2:0。
    regValue = static_cast<uint8_t>(
        regValue |
        (pressureValue & PRESS_OSR_MASK));

    // 温度过采样写入 bit 5:3。
    regValue = static_cast<uint8_t>(
        regValue |
        ((temperatureValue << TEMP_OSR_POS) & TEMP_OSR_MASK));

    return writeRegister(REG_OSR, regValue);
}


/**
 * @brief 设置 BMP388 正常模式下的输出数据率。
 * @param outputDataRate 输出数据率枚举值。
 * @return 设置成功返回 OK，否则返回具体错误。
 */
Barometer::BarometerError BMP388::setOutputDataRate(OutputDataRate outputDataRate){
    const uint8_t odrValue =
        static_cast<uint8_t>(outputDataRate);

    // BMP388 的合法 ODR 编码范围为 0x00～0x11。
    if (odrValue > 0x11U)
    {
        return Barometer::BarometerError::BAD_PARAM;
    }

    uint8_t regValue = 0;

    BARO_TRY(readRegister(REG_ODR, &regValue));

    // 清除 ODR 的 bit 4:0，保留 bit 7:5。
    regValue = static_cast<uint8_t>(
        regValue &
        static_cast<uint8_t>(~ODR_MASK));

    // 写入新的 ODR 编码。
    regValue = static_cast<uint8_t>(
        regValue |
        (odrValue & ODR_MASK));

    return writeRegister(REG_ODR, regValue);
}


/**
 * @brief 设置 BMP388 内部 IIR 滤波器的滤波系数。
 * @param iirFilter IIR 滤波器系数枚举值。
 * @return 设置成功返回 OK，否则返回具体错误。
 */
Barometer::BarometerError BMP388::setIIRFilter(IIRFilter iirFilter){
    const uint8_t filterValue =
        static_cast<uint8_t>(iirFilter);

    // IIR 有 8 种寄存器编码：0x00～0x07。
    if (filterValue > 0x07U)
    {
        return Barometer::BarometerError::BAD_PARAM;
    }

    uint8_t regValue = 0;

    BARO_TRY(readRegister(REG_CONFIG, &regValue));

    // 清除 CONFIG 寄存器 bit 3:1，保留其他位。
    regValue = static_cast<uint8_t>(
        regValue &
        static_cast<uint8_t>(~IIR_FILTER_MASK));

    // IIR 编码左移一位后放入 bit 3:1。
    regValue = static_cast<uint8_t>(
        regValue |
        ((filterValue << IIR_FILTER_POS) & IIR_FILTER_MASK));

    return writeRegister(REG_CONFIG, regValue);
}

/**
 * @brief 启用或关闭 BMP388 的压力和温度测量通道。
 * @param enablePressure true 表示启用压力测量。
 * @param enableTemperature true 表示启用温度测量。
 * @return 设置成功返回 OK，否则返回具体错误。
 */
Barometer::BarometerError BMP388::enableSensors(bool enablePressure, bool enableTemperature){

    uint8_t regValue = 0;

    BARO_TRY(readRegister(REG_PWR_CTRL, &regValue));

    // 只清除压力、温度使能位。
    // 不修改 bit 5:4 的工作模式。
    regValue = static_cast<uint8_t>(
        regValue &
        static_cast<uint8_t>(
            ~(PRESS_ENABLE_MASK | TEMP_ENABLE_MASK)));

    if (enablePressure)
    {
        regValue = static_cast<uint8_t>(
            regValue | PRESS_ENABLE_MASK);
    }

    if (enableTemperature)
    {
        regValue = static_cast<uint8_t>(
            regValue | TEMP_ENABLE_MASK);
    }

    return writeRegister(REG_PWR_CTRL, regValue);

}

Barometer::BarometerError BMP388::readRegister(uint8_t reg_addr, uint8_t* data){
    return readRegisters(reg_addr, data, 1U);
}

Barometer::BarometerError BMP388::readRegisters(uint8_t reg_addr, uint8_t* data, size_t length){
    if (data == nullptr || length == 0 || length > MAX_READ_LENGTH) return BarometerError::BAD_PARAM; 
    const uint8_t command = static_cast<uint8_t>(reg_addr | 0x80U);
    uint8_t tempBuffer[MAX_READ_LENGTH + SPI_DUMMY_BYTES] = {};
    const size_t transferDataLength = length + SPI_DUMMY_BYTES;
    BARO_TRY(SPITransferStream(&command, 1U, false, nullptr, tempBuffer, transferDataLength));
    memcpy(data, &tempBuffer[SPI_DUMMY_BYTES], length);
    return BarometerError::OK;
}


Barometer::BarometerError BMP388::writeRegister(uint8_t regAddr, uint8_t data)
{
    const uint8_t command = static_cast<uint8_t>(regAddr & 0x7FU);

    return SPITransferStream(
        &command,
        1U,
        true,
        &data,
        nullptr,
        1U
    );
}

Barometer::BarometerError BMP388::SPITransferStream(const uint8_t* cmd, uint8_t cmdLen, bool write, const uint8_t* pTxData, uint8_t* pRxData, size_t numBytes){
    BarometerError result;
    size_t buffLen = cmdLen + numBytes;
    static constexpr size_t SPI_BUF_MAX = 32;

    if (buffLen > SPI_BUF_MAX) return BarometerError::BAD_PARAM;

// 空指针检查
    if ((cmd == nullptr) || (cmdLen == 0U)) {
        return BarometerError::BAD_PARAM;
    }
    if ((static_cast<size_t>(cmdLen) > SPI_BUF_MAX) ||
        (numBytes > SPI_BUF_MAX - cmdLen)) {
        return BarometerError::BAD_PARAM;
    }
    if (write) {
        if ((numBytes > 0U) && (pTxData == nullptr)) {
            return BarometerError::BAD_PARAM;
        }
    } else {
        if ((numBytes > 0U) && (pRxData == nullptr)) {
            return BarometerError::BAD_PARAM;
        }
    }

    uint8_t buffOut[SPI_BUF_MAX];
    uint8_t* buffOutPtr = buffOut;

    memcpy(buffOutPtr, cmd, cmdLen);
    buffOutPtr += cmdLen;

    if (write) {
        if (numBytes > 0U){
            memcpy(buffOutPtr, pTxData, numBytes);
        }
    } else {
        memset(buffOutPtr, 0x00, numBytes);
    }

    uint8_t buffIn[SPI_BUF_MAX];

    result = spiSendReceiveBuffer(buffOut, buffLen, buffIn);
    if (result != BarometerError::OK) return result;

    if (!write && numBytes > 0 && pRxData != nullptr){
        memcpy(pRxData, &buffIn[cmdLen], numBytes);
    }

    return BarometerError::OK;
}

Barometer::BarometerError BMP388::spiSendReceiveBuffer(uint8_t* txBuf, size_t len, uint8_t* rxBuf){
    if ((txBuf == nullptr) ||
        (rxBuf == nullptr) ||
        (len == 0U) ||
        (len > UINT16_MAX)) {
        return BarometerError::BAD_PARAM;
    }
    SPIGuard guard(m_handleTypeDef, (uint32_t)200);
    if (!guard.ok()){
        m_busTimeoutCount++;
        return BarometerError::BUS_TIMEOUT;
    }
    HAL_StatusTypeDef status = SPI_TransmitReceive(m_handleTypeDef.spiHandle, txBuf, rxBuf,((uint16_t)len), 200);
    if (status != HAL_OK) return BarometerError::COMM_FAIL;
    return BarometerError::OK;
}

uint16_t BMP388::combineBytes(uint8_t msb, uint8_t lsb)
{
    return static_cast<uint16_t>((static_cast<uint16_t>(msb) << 8U) | static_cast<uint16_t>(lsb));
}