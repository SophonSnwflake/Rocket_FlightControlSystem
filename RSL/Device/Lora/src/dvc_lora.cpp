#include "cmsis_os2.h"
#include "drv_spi.h"
#include "stm32f411xe.h"
#include "dvc_lora.hpp"
#include "def_sx1268.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "drv_time.h"

LoRa::LoRa(){}

SX1268::SX1268(SX1268PinConfig &config) : 
        m_PinConfig(config),
        m_chipType(SX1268_CHIP_TYPE){
        
}

/**
 * @brief 初始化 SX1268，并配置为 LoRa 模式。
 *
 * @details 完成芯片检测、基础校准、同步字、数据包参数、
 * 调制参数、工作频率、PA 配置和发射功率设置。
 *
 * @param config LoRa 配置参数，包括频率、带宽、扩频因子、
 * 编码率、同步字、发射功率和前导码长度。
 *
 * @return LoraError::OK 初始化成功。
 * @return LoraError::ChipNotFound 未检测到 SX1268。
 * @return LoraError::BusyTimeout BUSY 或 SPI 总线等待超时。
 * @return LoraError::CommFail SPI 通信失败。
 * @return LoraError::DeviceError 芯片命令执行失败。
 * @return 其他错误 参数超出支持范围或当前模式不正确。
 *
 * @note 固定使用显式包头、CRC、标准 IQ 和软件控制 RXEN/TXEN。
 * @warning 初始化失败后芯片可能处于部分配置状态，建议重新复位。
 */
LoRa::LoraError SX1268::beginLoRa(const ConfigLoRa_t& config){
    m_spreadingFactor = config.spreadingFactor;
    m_bandwidthKhz = config.bandwidthKhz;
    m_ldrOptimize = 0x00;
    m_crcTypeLoRa = SX126X_LORA_CRC_ON;
    m_preambleLengthLoRa = config.preambleLength;
    m_headerType = SX126X_LORA_HEADER_EXPLICIT;
    m_implicitLen = 0xFF;
    m_codingRate = config.codingRate;
    m_syncWord = config.syncWord;
    m_currentLimit = 140.0f;
    m_power = config.power;
    m_frequency = config.frequency;
    m_tcxoVoltage = 2.2f;

    // modSetup ↓
    if(!findChip(m_chipType)){
        return LoraError::ChipNotFound;
    }
    LORA_TRY(setTCXO(m_tcxoVoltage, 5000));
    LORA_TRY(configLoRa());
    // modSetup ↑


    LORA_TRY(setSyncWord(m_syncWord));

    LORA_TRY(setPacketParams(m_preambleLengthLoRa, m_implicitLen));
    
    LORA_TRY(setCurrentLimit(m_currentLimit));

    LORA_TRY(disableDio2RfSwitch());

    LORA_TRY(soluteModlationParams(m_spreadingFactor, m_bandwidthKhz, m_codingRate));

    LORA_TRY(setFrequency(m_frequency));

    LORA_TRY(applyPaClampingWorkaround());

    LORA_TRY(setOutputPower(m_power));

    m_isLoRabegined = true;

    return LoraError::OK;

}

/**
 * @brief 配置 SX1268 的 LoRa 基础工作状态。
 *
 * @details 设置 TX/RX 缓冲区基地址、LoRa PacketType、
 * 收发结束后的回退模式、内部稳压方式，并清除已有 IRQ。
 * 随后执行芯片全项校准，等待 BUSY 引脚恢复低电平。
 *
 * @return LoraError::OK 配置成功。
 * @return LoraError::BusyTimeout 等待 BUSY 或 SPI 总线超时。
 * @return LoraError::CommFail SPI 通信失败。
 * @return LoraError::DeviceError 芯片命令处理或执行失败。
 * @return LoraError::BadParam 底层函数接收到非法参数。
 *
 * @note 当前使用 LoRa 模式、STBY_RC 回退状态和 DC-DC 稳压模式。
 * @warning 调用前应确保芯片已复位并处于可接收命令的状态。
 */
LoRa::LoraError SX1268::configLoRa(){
    LORA_TRY(setBufferBaseAddress(0,0));

    // 设置调制模式
    uint8_t data[7];
    data[0] = SX126X_PACKET_TYPE_LORA;
    LORA_TRY(SPIwriteStream(SX126X_CMD_SET_PACKET_TYPE, data, 1));

    // 设置自动安全状态
    data[0] = SX126X_RX_TX_FALLBACK_MODE_STDBY_RC;
    LORA_TRY(SPIwriteStream(SX126X_CMD_SET_RX_TX_FALLBACK_MODE, data, 1));

    clearIrqStatus();

    LORA_TRY(setRegulatorMode(SX126X_REGULATOR_DC_DC));
    LORA_TRY(setDioIrqParams(SX126X_IRQ_NONE, SX126X_IRQ_NONE, SX126X_IRQ_NONE, SX126X_IRQ_NONE));

    data[0] = SX126X_CALIBRATE_ALL;
    LORA_TRY(SPIwriteStream(SX126X_CMD_CALIBRATE, data, 1));

    osDelay(5);
    LORA_TRY(waitBusy());

    

    return LoraError::OK;
}

/**
 * @brief 复位并检测 SX1268 芯片是否存在。
 *
 * @details 最多尝试 10 次，每次先硬件复位芯片，再读取版本字符串寄存器，
 * 并比较前 6 个字符是否与预期芯片标识一致。读取失败或校验不匹配时延时重试。
 *
 * @param verStr 预期的芯片版本字符串，不可为 nullptr。
 *
 * @return true 成功读取到匹配的芯片版本字符串。
 * @return false 连续 10 次尝试后仍未检测到目标芯片。
 *
 * @note 本函数会多次复位芯片，调用成功后仍需重新执行完整初始化配置。
 */
bool SX1268::findChip(const char* verStr){
    uint8_t i = 0;
    bool flagFound = false;

    while((i <10) && !flagFound){
        reset(true);
        char version[16] = {0};
        if (SPIreadRegister(SX126X_REG_VERSION_STRING, 16, (uint8_t*)version) != LoraError::OK) {
            osDelay(10); i++; continue;   // 读失败,重试
        }
        
        if (strncmp(verStr, version, 6) == 0){
            flagFound = true;
        } else {
            osDelay(10);
            i ++;
        }
    }
    return flagFound;
}

/**
 * @brief 对 SX1268 执行硬件复位。
 *
 * @details 将 NRST 拉低 1 ms 后重新拉高。若 verify 为 true，
 * 则循环发送 SetStandby 命令验证芯片是否已恢复响应，最长等待 1000 ms。
 *
 * @param verify 是否在复位后通过进入 STBY_RC 验证芯片响应。
 *
 * @return LoraError::OK 复位成功，且芯片在需要时通过响应验证。
 * @return LoraError::BusyTimeout 等待 BUSY 或 SPI 总线超时。
 * @return LoraError::CommFail SPI 通信失败。
 * @return LoraError::DeviceError 芯片拒绝或无法执行 Standby 命令。
 *
 * @note verify 为 false 时，本函数不会等待或确认芯片已完成启动。
 */
LoRa::LoraError SX1268::reset(bool verify){
    HAL_GPIO_WritePin(m_PinConfig.rst.port, m_PinConfig.rst.pin, GPIO_PIN_RESET);
    osDelay(1);
    HAL_GPIO_WritePin(m_PinConfig.rst.port, m_PinConfig.rst.pin, GPIO_PIN_SET);

    if (!verify){
        return LoraError::OK;
    }

    uint32_t start = HAL_GetTick();
    while(true) {
        LoraError state = standby(RC);
        if (state == LoraError::OK) return LoraError::OK;

        if (HAL_GetTick() - start >= 1000){
            return state;
        }

        osDelay(10);
    }

}                                                                           


LoRa::LoraError SX1268::standby(StandbyMode mode) {
    setRfMode(Idle);
    uint8_t param;
    switch (mode) {
        case StandbyMode::RC:   param = SX126X_STANDBY_RC;   break;
        case StandbyMode::XOSC: param = SX126X_STANDBY_XOSC; break;
        default:                return LoraError::BadParam;   
    }
    const uint8_t data[] = { param };
    uint8_t opcode = SX126X_CMD_SET_STANDBY;
    return (SPITransferStream(&opcode, 1, true, data, NULL, 1, true));
}


//==============================================================================
// Hand函数
//==============================================================================


/**
 * @brief 阻塞发送一帧 LoRa 数据。
 * @details 启动发送后轮询 DIO1，直到 TX_DONE 或软件超时。
 *          超时时间取预计空中时间的 5 倍，并额外增加 5 ms。
 *          发送完成或超时后调用 finishTransmit() 清理 IRQ 和 RF 通路。
 * @param data 待发送数据的缓冲区，长度非零时不能为 nullptr。
 * @param len 待发送数据长度，最大为 SX126X_MAX_PACKET_LENGTH。
 * @param addr 数据写入 SX126x 内部 Buffer 的起始偏移地址。
 * @retval LoraError::OK 数据发送完成且清理成功。
 * @retval LoraError::PacketTooLong 数据长度超过芯片允许的最大包长。
 * @retval LoraError::TxTimeOut 在预计时间内未检测到发送完成 IRQ。
 * @retval 其他错误 待机、配置、SPI、BUSY 或发送清理失败。
 * @note 本函数为阻塞式接口，等待期间通过 osThreadYield() 让出 CPU。
 */

LoRa::LoraError SX1268::transmit(
    const uint8_t* data,
    size_t len,
    uint8_t addr)
{
    LORA_TRY(standby(RC));

    if (len > SX126X_MAX_PACKET_LENGTH) {
        return LoraError::PacketTooLong;
    }
    const uint32_t timeout = 3000U;

    LORA_TRY(startTransmit(data, len, addr));
    uint32_t start = HAL_GetTick();

    while(true)
    {
        volatile uint32_t pa11_mode =
        (GPIOA->MODER >> 22) & 0x03;
        const uint16_t irq = getIrqFlags();

        if((irq & SX126X_IRQ_TX_DONE) != 0U)
        {
            printf("TX_DONE by IRQ status\r\n");
            break;
        }

        if((irq & SX126X_IRQ_TIMEOUT) != 0U)
        {
            printf("HW TX timeout\r\n");
            finishTransmit();
            return LoraError::TxTimeOut;
        }

        if(HAL_GetTick() - start > timeout)
        {
            printf(
                "SW timeout: DIO1=%u IRQ=0x%04X timeout=%lu\r\n",
                isGetIrq(),
                getIrqFlags(),
                timeout
            );

            finishTransmit();
            return LoraError::TxTimeOut;
        }

        osDelay(1);
    
    }
    return finishTransmit();
}


/**
 * @brief 阻塞接收一帧 LoRa 数据。
 * @details 启动接收后轮询 DIO1，直到收到数据包或发生硬件/软件超时。
 *          timeout 为 0 时，根据预计空中时间自动计算等待时间。
 *          超时时间将转换为 SX126x 的 15.625 us 计数并传入 SetRx。
 *          接收结束后退出 RX、关闭外部接收通路并读取芯片缓冲区。
 * @param data 接收数据的目标缓冲区，不能为 nullptr。
 * @param len 最大读取长度；为 0 时读取完整数据包。
 * @param timeout 接收超时时间，单位为 ms；为 0 时自动计算。
 * @retval LoraError::OK 成功接收并读取数据。
 * @retval LoraError::RxTimeOut 发生硬件或软件接收超时。
 * @retval LoraError::BadParam 超时无效或超出硬件计数范围。
 * @retval LoraError::CrcMismatch 收到数据，但数据完整性校验失败。
 * @retval 其他错误 SPI、BUSY、IRQ 或缓冲区读取失败。
 * @note 本函数为阻塞式接口；len 为 0 时，data 应至少可容纳 255 字节。
 */
LoRa::LoraError SX1268::receive(uint8_t* data, size_t len, uint32_t timeout){
    LORA_TRY(standby(RC));
    uint32_t timeoutInternal = timeout;
    if (timeoutInternal == 0U){
        size_t maxLen = len;
        if(len == 0){maxLen = SX126X_MAX_PACKET_LENGTH;}
        timeoutInternal = (getTimeOnAir(maxLen) * 5) / 1000;
    }

    const uint64_t timeoutTicks64 = static_cast<uint64_t>(timeoutInternal) * 64ULL;
    if ((timeoutTicks64 == 0ULL) || (timeoutTicks64 >= 0xFFFFFFULL)) return LoraError::BadParam; 
    const uint32_t timeoutValue = static_cast<uint32_t>(timeoutTicks64);
    
    LORA_TRY(startReceive(len, timeoutValue));

    constexpr uint32_t RX_TIMEOUT_MARGIN_MS = 5U;
    bool softTimeout = false;
    uint32_t start = HAL_GetTick();
    while(!isGetIrq()){
        osThreadYield();
        if(HAL_GetTick() - start > timeoutInternal + RX_TIMEOUT_MARGIN_MS)
        {
            softTimeout = true;
            break;
        }
    }

    LORA_TRY(standby(RC));

    if(softTimeout || (getIrqFlags() & SX126X_IRQ_TIMEOUT)) {
        (void)finishReceive();
        return(LoraError::RxTimeOut);
    }
    setRfMode(RfMode::Idle);

    return(readData(data, len));
}

LoRa::LoraError SX1268::startTransmit(const uint8_t* data, size_t len, uint8_t addr){
    // check packet length
    if(len > SX126X_MAX_PACKET_LENGTH) return LoraError::PacketTooLong;

    LORA_TRY(setPacketParams(m_preambleLengthLoRa, len));

    LORA_TRY(setDioIrqParams(SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT, SX126X_IRQ_TX_DONE,  SX126X_IRQ_NONE, SX126X_IRQ_NONE));

    LORA_TRY(setBufferBaseAddress(0x00, 0x00));

    LORA_TRY(SPIwriteBuffer(data, static_cast<uint8_t>(len), 0x00));

    // LORA_TRY(SPIwriteBuffer(data, len));      

    LORA_TRY(clearIrqStatus());

    LORA_TRY(fixSensitivity());

    LORA_TRY(launchMode(Tx, 0));
                       
    return LoraError::OK;
}

LoRa::LoraError SX1268::startReceive(size_t len, uint32_t timeout){
    if (len > 255U)  return LoraError::PacketTooLong;
    LORA_TRY(standby(RC));
    const uint16_t enabledIrqs =
    SX126X_IRQ_RX_DONE       |
    SX126X_IRQ_TIMEOUT       |
    SX126X_IRQ_CRC_ERR       |
    SX126X_IRQ_HEADER_VALID  |
    SX126X_IRQ_HEADER_ERR;

    const uint16_t dio1Irqs =
    SX126X_IRQ_RX_DONE |
    SX126X_IRQ_TIMEOUT;


    LORA_TRY(setDioIrqParams(enabledIrqs, dio1Irqs, 0, 0));
    LORA_TRY(setBufferBaseAddress(0x00, 0x00));
    LORA_TRY(clearIrqStatus());
    LORA_TRY(setPacketParams(m_preambleLengthLoRa, len));
    return(launchMode(Rx, timeout));
}

// 配置DIO3为外部32MHzTCXO的电源控制，并设置TCXO启动等待时间
LoRa::LoraError SX1268::setTCXO(fp32 voltage, uint32_t delay){
    LORA_TRY(standby(RC));
    uint16_t errors = 0U;
    LORA_TRY(getDeviceErrors(&errors));
    if(errors & 0x0020U) {
        LORA_TRY(clearDeviceErrors());
    }
    if(fabsf(voltage - 0.0f) <= 0.001f) return(reset(true));
    uint8_t data[4];
    if(fabsf(voltage - 1.6f) <= 0.001f) {
        data[0] = SX126X_DIO3_OUTPUT_1_6;
    } else if(fabsf(voltage - 1.7f) <= 0.001f) {
        data[0] = SX126X_DIO3_OUTPUT_1_7;
    } else if(fabsf(voltage - 1.8f) <= 0.001f) {
        data[0] = SX126X_DIO3_OUTPUT_1_8;
    } else if(fabsf(voltage - 2.2f) <= 0.001f) {
        data[0] = SX126X_DIO3_OUTPUT_2_2;
    } else if(fabsf(voltage - 2.4f) <= 0.001f) {
        data[0] = SX126X_DIO3_OUTPUT_2_4;
    } else if(fabsf(voltage - 2.7f) <= 0.001f) {
        data[0] = SX126X_DIO3_OUTPUT_2_7;
    } else if(fabsf(voltage - 3.0f) <= 0.001f) {
        data[0] = SX126X_DIO3_OUTPUT_3_0;
    } else if(fabsf(voltage - 3.3f) <= 0.001f) {
        data[0] = SX126X_DIO3_OUTPUT_3_3;
    } else {
        return(LoraError::InvalidTXCOVoltage);
    }

    uint32_t delayValue = (delay * 8U) / 125U;
    data[1] = (uint8_t)((delayValue >> 16) & 0xFF);
    data[2] = (uint8_t)((delayValue >> 8) & 0xFF);
    data[3] = (uint8_t)(delayValue & 0xFF);
    m_tcxoDelay = delay;

    return(SPIwriteStream(SX126X_CMD_SET_DIO3_AS_TCXO_CTRL, data, 4));
}

//------------------------------------------------------------------------------
// Hand Helper
//------------------------------------------------------------------------------

LoRa::LoraError SX1268::finishTransmit()
{
    // printf("finishTx: enter\r\n");

    // printf("finishTx: before standby, BUSY=%u\r\n",
    //        static_cast<unsigned>(isBusy()));

    LoRa::LoraError state = standby(RC);

    if(state != LoraError::OK) {
        return state;
    }

    // printf("finishTx: before clearIrq\r\n");

    state = clearIrqStatus();

    // printf("finishTx: after clearIrq, state=%u BUSY=%u\r\n",
    //        static_cast<unsigned>(state),
    //        static_cast<unsigned>(isBusy()));

    return state;
}


LoRa::LoraError SX1268::finishReceive() {
    LORA_TRY(standby(RC));

    return(clearIrqStatus());
}

LoRa::LoraError SX1268::readData(uint8_t* data, size_t len)
{
    if (data == nullptr) return LoraError::BadParam;

    const uint16_t irq = getIrqFlags();

    if ((irq & SX126X_IRQ_TIMEOUT) != 0U) {
        (void)clearIrqStatus();
        setRfMode(RfMode::Idle);
        return LoraError::RxTimeOut;
    }

    // 确认确实完成了一次接收
    if ((irq & SX126X_IRQ_RX_DONE) == 0U) {
        (void)clearIrqStatus();
        setRfMode(RfMode::Idle);
        return LoraError::DeviceError;
    }

    // 记录数据完整性错误，但仍然读取原始数据
    LoraError crcState = LoraError::OK;

    if (
        ((irq & SX126X_IRQ_CRC_ERR) != 0U) ||
        (
            ((irq & SX126X_IRQ_HEADER_ERR) != 0U) &&
            ((irq & SX126X_IRQ_HEADER_VALID) == 0U)
        )
    ) {
        crcState = LoraError::CrcMismatch;
    }

    uint8_t payloadLength = 0U;
    uint8_t bufferOffset = 0U;

    LoraError state = getRxBufferStatus(
        payloadLength,
        bufferOffset,
        true
    );

    if (state != LoraError::OK) {
        (void)clearIrqStatus();
        setRfMode(RfMode::Idle);
        return state;
    }

    // len == 0 表示读取整个数据包
    if (
        (len != 0U) &&
        (len < static_cast<size_t>(payloadLength))
    ) {
        payloadLength = static_cast<uint8_t>(len);
    }

    state = SPIreadBuffer(
        data,
        payloadLength,
        bufferOffset
    );

    // 无论读取是否成功，都尽量完成清理
    const LoraError cleanupState = clearIrqStatus();
    setRfMode(RfMode::Idle);

    // 优先报告真正的数据读取/SPI错误
    if (state != LoraError::OK) {
        return state;
    }

    // 数据已经读出，但不可信
    if (crcState != LoraError::OK) {
        return crcState;
    }

    return cleanupState;
}

LoRa::LoraError SX1268::clearDeviceErrors(){
    const uint8_t data[2] = {SX126X_CMD_NOP, SX126X_CMD_NOP};
    return(SPIwriteStream(SX126X_CMD_CLEAR_DEVICE_ERRORS, data, 2));
}

/**
 * @brief 启动 SX1268 的指定射频工作模式。
 *
 * 根据 mode 将射频前端切换至接收或发送状态，并向 SX1268
 * 下发对应的 SetRx / SetTx 命令。
 *
 * Rx 模式下使用 64000 个 RTC tick，对应约 1000 ms 接收超时；
 * Tx 模式下关闭硬件发送超时，数据发送完成后由 TX_DONE IRQ 指示。
 * Tx 启动后会等待 BUSY 拉低，以确保芯片完成发送模式切换。
 *
 * @param mode 目标射频模式，仅支持 RfMode::Rx 和 RfMode::Tx。
 * @param timeout RX模式下的硬件超时tick，Tx模式传0即可
 * @return LoraError::OK 成功；否则返回底层操作错误或 Unsupported。
 */
LoRa::LoraError SX1268::launchMode(RfMode mode, uint32_t timeout) {
  switch(mode) {
    case(RfMode::Rx): {
      setRfMode(Rx);
      LORA_TRY(setRx(timeout));
    } break;
    
    case(RfMode::Tx): {
      setRfMode(Tx);

      // 无超时限制，一直发送到发送完
      LORA_TRY(setTx(SX126X_TX_TIMEOUT_NONE));
      LORA_TRY(waitBusy());
    } break;
    
    default:
      return LoraError::Unsupported;
    }

  return LoraError::OK;
}


//==============================================================================
// LoRa参数获取（Get）
//==============================================================================

LoRa::LoraError SX1268::getDeviceErrors(uint16_t* opError){
    uint8_t cmd[2];
    cmd[0] = SX126X_CMD_GET_DEVICE_ERRORS;   
    cmd[1] = 0x00;                            

    uint8_t data[2] = {0, 0};
    LORA_TRY(SPITransferStream(cmd, 2, false, NULL, data, 2, true));

    *opError = ((uint16_t)data[0] << 8) | (uint16_t)data[1];
    return LoraError::OK;
}

LoRa::LoraError SX1268::getPacketType(uint8_t* packetType) {   
    uint8_t cmd[2];
    cmd[0] = SX126X_CMD_GET_PACKET_TYPE;
    cmd[1] = 0x00;

    LORA_TRY(SPITransferStream(cmd, 2, false, NULL, packetType, 1, true));
    return LoraError::OK;
}

/**
 * @brief 计算 LoRa 数据包的空中时间（Time-on-Air），单位：微秒(us)。
 *
 * @details 纯软件计算，不发 SPI 命令——ToA 由 SF/BW/CR/包长唯一决定，是理论值。
 *   依据 SX1268 datasheet v1.1 §6.1.4 公式。为避开浮点误差，带宽与系数均以
 *   整数(×10 / ×4)参与运算，最后统一还原。用于给发送/接收设定超时余量。
 *
 *   固定假设（与 setPacketParams 写死的配置一致）：显式头(N_symbol_header=20)、
 *   CRC 开(bitsPerCrc=16)。SF5/6 系数特殊；LDRO 开时分母不同。
 *
 * @param[in] len  payload 字节数（不含 preamble/header/CRC）
 * @return 空中时间，单位微秒(us)。调用方若需毫秒需自行 /1000。
 * @note 依赖成员 m_spreadingFactor / m_bandwidthKhz / m_codingRate /
 *       m_ldrOptimize / m_preambleLengthLoRa，调用前须已配置。
 */
uint32_t SX1268::getTimeOnAir(size_t len){
    uint32_t bwKhz_x10 = (uint32_t)(m_bandwidthKhz * 10 + 0.5f);   // 浮点转整数(四舍五入)
    uint32_t symbolLength_us = ((uint32_t)(1000 * 10) << m_spreadingFactor) / bwKhz_x10;
    uint8_t sfCoeff1_x4 = 17; // (4.25 * 4)
    uint8_t sfCoeff2 = 8;
    if(m_spreadingFactor == 5 || m_spreadingFactor == 6) {
        sfCoeff1_x4 = 25; // 6.25 * 4
        sfCoeff2 = 0;
    }
    uint8_t sfDivisor = 4*m_spreadingFactor;
      if(m_ldrOptimize) {
        sfDivisor = 4*(m_spreadingFactor - 2);
    }
    const int8_t bitsPerCrc = 16;
    const int8_t N_symbol_header = 20;

    // numerator of equation in section 6.1.4 of SX1268 datasheet v1.1 (might not actually be bitcount, but it has len * 8)
    int16_t bitCount = (int16_t) 8 * len + bitsPerCrc - 4 * m_spreadingFactor  + sfCoeff2 + N_symbol_header;
    if(bitCount < 0) {
        bitCount = 0;
    }
    // add (sfDivisor) - 1 to the numerator to give integer CEIL(...)
    uint16_t nPreCodedSymbols = (bitCount + (sfDivisor - 1)) / (sfDivisor);

    // preamble can be 65k, therefore nSymbol_x4 needs to be 32 bit
    uint32_t nSymbol_x4 = (m_preambleLengthLoRa + 8) * 4 + sfCoeff1_x4 + nPreCodedSymbols * m_codingRate * 4;

    return((symbolLength_us * nSymbol_x4) / 4);
}

uint32_t SX1268::getIrqFlags() {
  uint8_t data[] = { 0x00, 0x00 };
  SPIreadStream(SX126X_CMD_GET_IRQ_STATUS, data, 2, true);
  return(((uint32_t)(data[0]) << 8) | data[1]);
}



LoRa::LoraError SX1268::getRxBufferStatus(uint8_t& payloadLength, uint8_t& bufferOffset, bool isUseDummy)
{
    uint8_t status[2] = {};
    LORA_TRY(SPIreadStream(SX126X_CMD_GET_RX_BUFFER_STATUS, status, sizeof(status), isUseDummy));
    payloadLength = status[0];
    bufferOffset = status[1];
    return LoraError::OK;
}


//==============================================================================
// LoRa参数设置（Set）
//==============================================================================

LoRa::LoraError SX1268::setBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress){
    const uint8_t data[2] = {txBaseAddress, rxBaseAddress};
    return (SPIwriteStream(SX126X_CMD_SET_BUFFER_BASE_ADDRESS, data, 2));
}

LoRa::LoraError SX1268::setSyncWord(uint8_t syncWord) {
    uint8_t type;
    LORA_TRY(getPacketType(&type));                        // 传指针,LORA_TRY 接返回值
    if(type != SX126X_PACKET_TYPE_LORA) return LoraError::WrongModem;
    const uint8_t data[2] = {
        (uint8_t)((syncWord & 0xF0) | ((SX126X_SYNC_WORD_CONTROL_BITS & 0xF0) >> 4)),   // data[0]
        (uint8_t)(((syncWord & 0x0F) << 4) | (SX126X_SYNC_WORD_CONTROL_BITS & 0x0F))    // data[1]
    };

    return (SPIwriteRegister(SX126X_REG_LORA_SYNC_WORD_MSB, 2, data));
}

LoRa::LoraError SX1268::setModulationParams(uint8_t spreadingFactor, uint8_t bandwidth, uint8_t codingRate) {
  const uint8_t data[4] = {spreadingFactor, bandwidth, codingRate, m_ldrOptimize};
  return(SPIwriteStream(SX126X_CMD_SET_MODULATION_PARAMS, data, 4));
}

LoRa::LoraError SX1268::setPacketParams(uint16_t preambleLen, uint8_t payloadLen) {   // 只留这俩可变
    uint8_t data[6];
    data[0] = (uint8_t)(preambleLen >> 8);
    data[1] = (uint8_t)(preambleLen & 0xFF);
    data[2] = SX126X_LORA_HEADER_EXPLICIT;   // 固定显式,写死
    data[3] = payloadLen;
    data[4] = SX126X_LORA_CRC_ON;            // 固定 CRC 开,写死
    data[5] = SX126X_LORA_IQ_STANDARD;       // 固定不反转,写死
    return SPIwriteStream(SX126X_CMD_SET_PACKET_PARAMS, data, 6);
}

LoRa::LoraError SX1268::disableDio2RfSwitch() {           
    uint8_t data = SX126X_DIO2_AS_IRQ;              
    return SPIwriteStream(SX126X_CMD_SET_DIO2_AS_RF_SWITCH_CTRL, &data, 1);
}


// 设置电源整流方案
LoRa::LoraError SX1268::setRegulatorMode(uint8_t mode){
    const uint8_t data[1] = {mode};
    return SPIwriteStream(SX126X_CMD_SET_REGULATOR_MODE, data, 1);
}

LoRa::LoraError SX1268::setCurrentLimit(fp32 currentLimit) {
  if(!((currentLimit >= 0) && (currentLimit <= 140))) return LoraError::InvalidCurrentLimit;

  uint8_t rawLimit = (uint8_t)(currentLimit / 2.5f);

  // update register
  return(SPIwriteRegister(SX126X_REG_OCP_CONFIGURATION, 1 ,&rawLimit));
}

LoRa::LoraError SX1268::applyPaClampingWorkaround(){
    uint8_t clampConfig = 0;
    LORA_TRY(SPIreadRegister(SX126X_REG_TX_CLAMP_CONFIG,1U,&clampConfig));
    clampConfig |= 0x1EU;
    return(SPIwriteRegister(SX126X_REG_TX_CLAMP_CONFIG, 1, &clampConfig));
}

LoRa::LoraError SX1268::setOutputPower(int8_t power) {
    uint8_t ocp = 0;

    uint8_t tempaDutyCycle = 0x04U;
    uint8_t temhpMax = 0x07U;
    uint8_t temdeviceSel =0x00;

    LORA_TRY(SPIreadRegister(SX126X_REG_OCP_CONFIGURATION, 1, &ocp));

    LORA_TRY(setPaConfig(tempaDutyCycle, temhpMax, temdeviceSel,   SX126X_PA_CONFIG_PA_LUT));

    LORA_TRY(setTxParams(power, SX126X_PA_RAMP_200U));

    return(SPIwriteRegister(SX126X_REG_OCP_CONFIGURATION, 1, &ocp));
}

// 设置两个参数：发射功率和功率爬升时间
// 待审查
LoRa::LoraError SX1268::setTxParams(int8_t powerDbm, uint8_t rampTime){
    if ((powerDbm < -9) || (powerDbm > 22)) return LoraError::InvalidOutputPower;

    const uint8_t data[2] = {
        static_cast<uint8_t>(powerDbm),
        rampTime
    };
    return SPIwriteStream(SX126X_CMD_SET_TX_PARAMS, data, 2);
}

LoRa::LoraError SX1268::setPaConfig(uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut){
    const uint8_t data[] = { paDutyCycle, hpMax, deviceSel, paLut };
    return SPIwriteStream(SX126X_CMD_SET_PA_CONFIG, data, 4);
}

LoRa::LoraError SX1268::setFrequency(fp32 frequency){
    if (frequency < 410.0f || frequency > 810.0f) return LoraError::InvalidFrequency;
    LORA_TRY(calibrateImage(frequency));

    // frf = freq_MHz × 2^25 / F_XTAL_MHz(单位都用 MHz)
    uint32_t frf = (uint32_t)((frequency * (float)(uint32_t(1) << SX126X_DIV_EXPONENT)) / SX126X_CRYSTAL_FREQ_MHZ);
    //                        ↑ 强制浮点运算,避免整数中间溢出
    const uint8_t data[4] = {
        (uint8_t)((frf >> 24) & 0xFF),
        (uint8_t)((frf >> 16) & 0xFF),
        (uint8_t)((frf >> 8) & 0xFF),
        (uint8_t)(frf & 0xFF)
    };
    return SPIwriteStream(SX126X_CMD_SET_RF_FREQUENCY, data, 4);
}

LoRa::LoraError SX1268::setDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask){
    const uint8_t data[8] = {(uint8_t)((irqMask >> 8) & 0xFF), (uint8_t)(irqMask & 0xFF),
                     (uint8_t)((dio1Mask >> 8) & 0xFF), (uint8_t)(dio1Mask & 0xFF),
                     (uint8_t)((dio2Mask >> 8) & 0xFF), (uint8_t)(dio2Mask & 0xFF),
                     (uint8_t)((dio3Mask >> 8) & 0xFF), (uint8_t)(dio3Mask & 0xFF)};
    return(SPIwriteStream(SX126X_CMD_SET_DIO_IRQ_PARAMS, data, 8));
}

void SX1268::setRfMode(RfMode mode){

    // 拉低全部控制线，防止出现同时拉高的烧芯片状态
    HAL_GPIO_WritePin(m_PinConfig.rxen.port, m_PinConfig.rxen.pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(m_PinConfig.txen.port, m_PinConfig.txen.pin, GPIO_PIN_RESET);
    switch(mode){
        case RfMode::Idle:
            HAL_GPIO_WritePin(m_PinConfig.rxen.port, m_PinConfig.rxen.pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(m_PinConfig.txen.port, m_PinConfig.txen.pin, GPIO_PIN_RESET);
            break;
        case RfMode::Tx:
            HAL_GPIO_WritePin(m_PinConfig.rxen.port, m_PinConfig.rxen.pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(m_PinConfig.txen.port, m_PinConfig.txen.pin, GPIO_PIN_SET);
            break;
        case RfMode::Rx:
            HAL_GPIO_WritePin(m_PinConfig.txen.port, m_PinConfig.txen.pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(m_PinConfig.rxen.port, m_PinConfig.rxen.pin, GPIO_PIN_SET); 
            break;
    }   
}

LoRa::LoraError SX1268::setRx(uint32_t timeout) {
  const uint8_t data[] = { (uint8_t)((timeout >> 16) & 0xFF), (uint8_t)((timeout >> 8) & 0xFF), (uint8_t)(timeout & 0xFF) };
  return(SPIwriteStream(SX126X_CMD_SET_RX, data, 3));
}

LoRa::LoraError SX1268::setTx(uint32_t timeout) {
  const uint8_t data[] = { (uint8_t)((timeout >> 16) & 0xFF), (uint8_t)((timeout >> 8) & 0xFF), (uint8_t)(timeout & 0xFF)} ;
  return(SPIwriteStream(SX126X_CMD_SET_TX, data, 3));
}

//------------------------------------------------------------------------------
// Set Helper
//------------------------------------------------------------------------------

LoRa::LoraError SX1268::clearIrqStatus(){
    uint8_t data[2];
    data[0] =  (uint8_t)(SX126X_IRQ_ALL >> 8U );
    data[1] =  (uint8_t)(SX126X_IRQ_ALL & 0xFF);
    LORA_TRY(SPIwriteStream(SX126X_CMD_CLEAR_IRQ_STATUS, data, 2));
    return LoraError::OK;
}

LoRa::LoraError SX1268::calibrateImage(fp32 freq){
    uint8_t data[2] = { 0, 0 };
    int freqBand = (int)freq;
    if((freqBand >= 902) && (freqBand <= 928)) {
    data[0] = SX126X_CAL_IMG_902_MHZ_1;
    data[1] = SX126X_CAL_IMG_902_MHZ_2;
  } else if((freqBand >= 863) && (freqBand <= 870)) {
    data[0] = SX126X_CAL_IMG_863_MHZ_1;
    data[1] = SX126X_CAL_IMG_863_MHZ_2;
  } else if((freqBand >= 779) && (freqBand <= 787)) {
    data[0] = SX126X_CAL_IMG_779_MHZ_1;
    data[1] = SX126X_CAL_IMG_779_MHZ_2;
  } else if((freqBand >= 470) && (freqBand <= 510)) {
    data[0] = SX126X_CAL_IMG_470_MHZ_1;
    data[1] = SX126X_CAL_IMG_470_MHZ_2;
  } else if((freqBand >= 430) && (freqBand <= 440)) {
    data[0] = SX126X_CAL_IMG_430_MHZ_1;
    data[1] = SX126X_CAL_IMG_430_MHZ_2;
  }
  else {return LoraError::InvalidFrequency;}

  LORA_TRY(SPIwriteStream(SX126X_CMD_CALIBRATE_IMAGE, data, 2));

  return LoraError::OK;

}

LoRa::LoraError SX1268::fixSensitivity(){
    uint8_t sensitivityConfig = 0;
    LORA_TRY(SPIreadRegister(SX126X_REG_SENSITIVITY_CONFIG, 1, &sensitivityConfig));

    uint8_t packetType;

    LORA_TRY(getPacketType(&packetType));
    if((packetType == SX126X_PACKET_TYPE_LORA) && (fabsf(m_bandwidthKhz - 500.0f) <= 0.001f)) {
    sensitivityConfig &= 0xFB;
    } else {
    sensitivityConfig |= 0x04;
    }
    return(SPIwriteRegister(SX126X_REG_SENSITIVITY_CONFIG, 1, &sensitivityConfig));
}

LoRa::LoraError SX1268::soluteModlationParams(uint8_t spreadingFactor, fp32 bandwidthKhz, uint8_t codingRate) {
    uint8_t type;
    LORA_TRY(getPacketType(&type)); 
    if(type != SX126X_PACKET_TYPE_LORA){
        return LoraError::WrongModem;
    }

    // SF处理开始
    if (spreadingFactor < 5 || spreadingFactor > 12) return LoraError::InvalidSpreadingFactor;
    const uint8_t temSpreadingFactor = spreadingFactor;
    // SF处理结束

    // BW处理开始
    if (bandwidthKhz < 0.0f || bandwidthKhz > 510.0f) return LoraError::InvalidBandWidth;
    uint8_t bw_div2 = bandwidthKhz / 2 + 0.01f;
    uint8_t temBandWidth;
    switch (bw_div2)  {
    case 3: // 7.8:
      temBandWidth = SX126X_LORA_BW_7_8;
      break;
    case 5: // 10.4:
      temBandWidth = SX126X_LORA_BW_10_4;
      break;
    case 7: // 15.6:
      temBandWidth = SX126X_LORA_BW_15_6;
      break;
    case 10: // 20.8:
      temBandWidth = SX126X_LORA_BW_20_8;
      break;
    case 15: // 31.25:
      temBandWidth = SX126X_LORA_BW_31_25;
      break;
    case 20: // 41.7:
      temBandWidth = SX126X_LORA_BW_41_7;
      break;
    case 31: // 62.5:
      temBandWidth = SX126X_LORA_BW_62_5;
      break;
    case 62: // 125.0:
      temBandWidth = SX126X_LORA_BW_125_0;
      break;
    case 125: // 250.0
      temBandWidth = SX126X_LORA_BW_250_0;
      break;
    case 250: // 500.0
      temBandWidth = SX126X_LORA_BW_500_0;
      break;
    default:
      return LoraError::InvalidBandWidth;
    }
    fp32 symbolLength = (fp32)(uint32_t(1) << spreadingFactor) / (float)bandwidthKhz;
    if(symbolLength >= 16.0f) {
      m_ldrOptimize = SX126X_LORA_LOW_DATA_RATE_OPTIMIZE_ON;
    } else {
      m_ldrOptimize = SX126X_LORA_LOW_DATA_RATE_OPTIMIZE_OFF;
    }

    // BW处理结束

    // CR处理开始
    if (codingRate < 4 || codingRate > 8) return LoraError::InvalidCodingRate; 
    const uint8_t temCodingRate = codingRate - 4;
    // CR处理结束
    
  return(setModulationParams(temSpreadingFactor, temBandWidth, temCodingRate));
}


//==============================================================================
// 次级SPI抽象
//==============================================================================


/**
 * @brief 读取 SX1268 指定寄存器中的数据
 *
 * 该函数通过 SPI 向 SX1268 发送 ReadRegister 命令，并从指定的
 * 16 位寄存器地址开始连续读取若干字节。
 *
 * SPI 命令格式：
 * @code
 * Byte 0：ReadRegister 命令码
 * Byte 1：寄存器地址高字节
 * Byte 2：寄存器地址低字节
 * Byte 3：NOP，占位字节
 * @endcode
 *
 * @param regsite  要读取的寄存器起始地址
 * @param numBytes 要读取的数据字节数
 * @param rxBuff   接收缓冲区指针，用于保存读取到的数据。
 *                 缓冲区容量不得小于 numBytes 字节
 *
 * @return LoraError::OK           读取成功
 * @return LoraError::BadParam     参数无效或缓冲区长度超出限制
 * @return LoraError::BusyTimeout  等待 BUSY 引脚或获取 SPI 总线锁超时
 * @return LoraError::CommFail     SPI 通信失败
 * @return LoraError::DeviceError  SX1268 报告命令执行错误
 */
LoRa::LoraError SX1268::SPIreadRegister(uint16_t regsite, size_t numBytes, uint8_t* rxBuff) {
    uint8_t cmd[4];
    cmd[0] = SX126X_CMD_READ_REGISTER;
    cmd[1] = (uint8_t)(regsite >> 8);
    cmd[2] = (uint8_t)(regsite & 0xFF);
    cmd[3] = 0x00;
    LoraError state;
    state = SPITransferStream(cmd, 4, false, NULL, rxBuff, numBytes, true);
    return state;
}

LoRa::LoraError SX1268::SPIwriteRegister(uint16_t regsite, size_t numBytes, const uint8_t* txData) {
    uint8_t cmd[3];
    cmd[0] = SX126X_CMD_WRITE_REGISTER;
    cmd[1] = (uint8_t)(regsite >> 8);
    cmd[2] = (uint8_t)(regsite & 0xFF);

    LORA_TRY(SPITransferStream(cmd, 3, true, txData, NULL, numBytes, true));
    return LoraError::OK; 
}

LoRa::LoraError SX1268::SPIwriteStream(uint16_t cmd, const uint8_t* data, size_t numBytes){
    uint8_t cmdBuf[1];
    cmdBuf[0] = static_cast<uint8_t>(cmd & 0xFFU);

    LoraError state;
    state = SPITransferStream(cmdBuf, 1, true, data, NULL, numBytes, true);

    return state;
}


// 对底层SPI引擎的构式小包装，在不确认dummy位置的情况下慎用
LoRa::LoraError SX1268::SPIreadStream(uint8_t opcode, uint8_t* data, size_t numBytes, bool isUseDummy){
    if(isUseDummy){
        uint8_t cmdBuf[2];
        cmdBuf[0] = opcode;
        cmdBuf[1] = 0x00;
        LORA_TRY(SPITransferStream(cmdBuf, 2, false, NULL, data, numBytes, true));
        return LoraError::OK;
    }else{
        uint8_t cmdBuf[1];
        cmdBuf[0] = opcode;
        LORA_TRY(SPITransferStream(cmdBuf, 1, false, NULL, data, numBytes, true));
        return LoraError::OK;
    }
}

LoRa::LoraError SX1268::SPIwriteBuffer(
    const uint8_t* data,
    uint8_t numBytes,
    uint8_t offset
)
{
    if ((numBytes > 0U) && (data == nullptr)) {
        return LoraError::BadParam;
    }

    if ((static_cast<uint16_t>(offset) + numBytes) > 256U) {
        return LoraError::BadParam;
    }

    const uint8_t cmd[] = {
        SX126X_CMD_WRITE_BUFFER,
        offset
    };

    return SPITransferStream(
        cmd,
        sizeof(cmd),
        true,
        data,
        nullptr,
        numBytes,
        true
    );
}

LoRa::LoraError SX1268::SPIreadBuffer(uint8_t* data, uint8_t numBytes, uint8_t offset) {
    const uint8_t cmd[] = {SX126X_CMD_READ_BUFFER, offset, 0x00};
    return(SPITransferStream(cmd, 3, false, NULL, data, numBytes, true));
}


//==============================================================================
// 底层SPI通信
//==============================================================================


/**
 * @brief  SX1268 SPI 底层事务引擎：拼命令、收发、等 BUSY、解析状态一条龙。
 *
 * @details
 *   这是所有 SX1268 命令的唯一 SPI 出口——上层的 standby、setModulationParams
 *   等命令函数都通过它把 opcode + 参数发给芯片。一次调用完成一条完整的 SPI 事务。
 *
 *   【SX126x 命令格式】一次事务的字节流为 [opcode][参数...]：
 *     - cmd/cmdLen  是命令段（opcode，通常 1 字节；读写寄存器时是 opcode+地址）
 *     - pTxData/numBytes 是参数/数据段
 *   两段在这里被拼进同一个发送缓冲 buffOut 一起发出。
 *
 *   【全双工与 status】SPI 收发同步：发多少必然收多少。SX126x 每次事务的
 *   第一个字节回来的是芯片实时 status（不是命令回复，是状态广播）。因此：
 *     - 读操作(!write)时收长度要多算一个 STATUS_LEN_BYTES（打头的 status 字节）
 *     - 该 status 字节顺手拿来做命令结果诊断（parseStatus），几乎零成本
 *
 *   【读操作的 dummy】读时没有真实数据要发，但全双工要求“收几个就得发几个”来
 *   产生时钟，所以发送段用 0x00(NOP) 填满，纯粹为把 status+数据挤回来。
 *
 *             注意！这里的cmd和cmdLen指的是opcode+地址
 * @param[in]  cmd           命令段缓冲（opcode 起始），不可为 nullpt。如果读取寄存器，结构为opcode + 地址
 * @param[in]  cmdLen        命令段字节数，不可为 0
 * @param[in]  write         true=写命令(发参数)，false=读命令(收数据)
 * @param[in]  pTxData       写操作要发送的参数/数据；write 且 numBytes>0 时不可为 nullptr
 * @param[out] pRxData       读操作接收数据的缓冲；!write 且 numBytes>0 时不可为 nullptr
 * @param[in]  numBytes      参数段/数据段字节数（不含 opcode，不含 status）
 * @param[in]  isWaitForGPIO 是否在收发前后等待 BUSY 拉低；常规命令传 true，
 *                           不能等 BUSY 的特例（如 sleep 唤醒）才传 false
 *
 * @return LoraError
 *   - OK           事务成功
 *   - BadParam     参数非法：空指针 / cmdLen==0 / 总长度超过 SPI_BUF_MAX
 *   - BusyTimeout  等 BUSY 超时，或抢 SPI 总线锁超时（见 spiSendReceiveBuffer）
 *   - CommFail     HAL SPI 传输本身失败
 *   - DeviceError  芯片 status 报命令执行错误（超时/非法命令/执行失败）
 *
 * @note 缓冲区用固定大小栈数组 SPI_BUF_MAX(260) 而非堆/VLA——飞控禁止堆分配
 *       (碎片、时序不确定)，也禁止 VLA(栈用量不可静态预测、可能栈溢出)。数组
 *       大小固定，实际收发长度由 buffLen 决定，不会发出多余字节。
 * @note 260 = 命令头 + 最大 255 payload + status，覆盖最坏情况。
 *
 * @warning 收发缓冲长度必须一致(SPI 全双工)，二者都用 buffLen。
 * @warning 上层调用本函数时务必检查返回值(LORA_TRY)，勿丢弃——错误需逐级上抛。
 *
 * @todo 待确认：drv_spi 内部 rx 缓冲仅 256 字节，读满 255 payload 时
 *       实际收长度 = cmdLen + 255 + status 可能溢出，需核对接收路径。
 */
LoRa::LoraError SX1268::SPITransferStream(const uint8_t* cmd, uint8_t cmdLen, bool write, const uint8_t* pTxData, uint8_t* pRxData, size_t numBytes, bool isWaitForGPIO){
    LoraError result;
    size_t buffLen = cmdLen + numBytes;
    static constexpr size_t SPI_BUF_MAX = 260;

    if (buffLen > SPI_BUF_MAX) return LoraError::BadParam;



// 空指针检查
    if ((cmd == nullptr) || (cmdLen == 0U)) {
        return LoraError::BadParam;
    }
    if (write) {
        if ((numBytes > 0U) && (pTxData == nullptr)) {
            return LoraError::BadParam;
        }
    } else {
        if ((numBytes > 0U) && (pRxData == nullptr)) {
            return LoraError::BadParam;
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

    if (isWaitForGPIO) {
        result = waitBusy();
        if (result != LoraError::OK) return result;
    }

    uint8_t buffIn[SPI_BUF_MAX];

    result = spiSendReceiveBuffer(buffOut, buffLen, buffIn);
    if (result != LoraError::OK) return result;

    if (isWaitForGPIO){
        Delay_us(1);
        result = waitBusy();
        if (result != LoraError::OK) return result;
    }

    // if (numBytes > 0) {
    // uint8_t statusPos = write ? 0 : 1;   // 写 status 在 byte0,读在 byte1
    // result = parseStatus(buffIn[statusPos]);
    // if (result != LoraError::OK) return result;
    // }

    if (!write && numBytes > 0 && pRxData != nullptr){
        memcpy(pRxData, &buffIn[cmdLen], numBytes);
    }

    return LoraError::OK;
}

LoRa::LoraError SX1268::spiSendReceiveBuffer(uint8_t* txBuf, size_t len, uint8_t* rxBuf){
    SPIGuard guard(m_PinConfig, (uint32_t)200);
    if (!guard.ok()){
        m_busTimeoutCount++;
        return LoraError::BusyTimeout;
    }
    HAL_StatusTypeDef status = SPI_TransmitReceive(m_PinConfig.hspi, txBuf, rxBuf,((uint16_t)len), 200);
    if (status != HAL_OK) return LoraError::CommFail;
    return LoraError::OK;
}

//------------------------------------------------------------------------------
// 底层SPI Helper
//------------------------------------------------------------------------------

LoRa::LoraError SX1268::parseStatus(uint8_t status){
    int16_t commandStatus = (status >> 1U) & 0x07;
    switch (commandStatus)
    {
        case 0x03U:  // SPI command timeout
        case 0x04U:  // Invalid SPI command
        case 0x05U:  // Command execution failed
            return LoraError::DeviceError;

        default:
            // 0、1、7：Reserved
            // 2：Data available
            // 6：TX done
            return LoraError::OK;
    }
}

bool SX1268::isBusy(){
    return HAL_GPIO_ReadPin(m_PinConfig.busy.port, m_PinConfig.busy.pin) == GPIO_PIN_SET;
}

bool SX1268::isGetIrq(){
    return HAL_GPIO_ReadPin(m_PinConfig.dio1.port, m_PinConfig.dio1.pin) == GPIO_PIN_SET;
}

LoRa::LoraError SX1268::waitBusy() {
    uint32_t start = HAL_GetTick();
    while (isBusy()) {
        osThreadYield();
        if (HAL_GetTick() - start >= LORA_TIMEOUT) {
            return LoraError::BusyTimeout;
        }
    }
    return LoraError::OK;
}

// 抓虫临时函数
LoRa::LoraError SX1268::getStatusRaw(uint8_t& status)
{
    uint8_t tx[2] = {
        0xC0U,
        0x00U
    };

    uint8_t rx[2] = {};

    LORA_TRY(waitBusy());
    LORA_TRY(spiSendReceiveBuffer(tx, sizeof(tx), rx));

    status = rx[1];

    return LoraError::OK;
}