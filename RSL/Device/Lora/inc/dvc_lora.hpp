#pragma once

#include "RSL_common.h"
#include "alg_general.hpp"
#include "cmsis_os.h"
#include "drv_spi.h"

#define LORA_SYNC_WORD_PRIVATE (0x12UL << 0)

class LoRa {
public:
    typedef struct{
        fp32 frequency = 434.0f;
        fp32 bandwidthKhz = 125.0f;
        uint8_t spreadingFactor = 9;
        uint8_t codingRate = 7;
        uint8_t syncWord = LORA_SYNC_WORD_PRIVATE;
        int8_t power = 10;
        uint16_t preambleLength = 8;

    } ConfigLoRa_t;

    typedef enum  {
        MODEM_FSK = 0,
        MODEM_LORA,
        MODEM_LRFHSS,
        MODEM_NONE,  
    } ModemType_t;

    enum class LoraError : uint8_t {
    OK = 0,          
    CommFail,        
    BadParam,        
    Unsupported,     
    NotInitialized,  
    BusyTimeout,
    DeviceError,
    ChipNotFound,
    WrongModem,
    InvalidCodingRate,
    InvalidCurrentLimit,
    InvalidSpreadingFactor,
    InvalidBandWidth,
    InvalidOutputPower,
    InvalidFrequency,
    PacketTooLong, // 15
    TxTimeOut, // 16
    RxTimeOut,
    CrcMismatch,
    InvalidTXCOVoltage,
    };
protected:
    

public:
    LoRa();
    virtual ~LoRa() = default;

    virtual LoraError beginLoRa(const ConfigLoRa_t& config) = 0;
    virtual LoraError setFrequency(fp32 freq) = 0;
    virtual LoraError setOutputPower(int8_t power) = 0;
    virtual LoraError transmit(const uint8_t* data, size_t len) = 0;
    virtual LoraError receive(uint8_t* data, size_t len, uint32_t timeout) = 0;

    bool isLoRaBegined() {return m_isLoRabegined;}

protected:
    fp32 m_bandwidthKhz = 0;
    uint8_t m_spreadingFactor = 0;
    uint8_t m_codingRate = 0;
    uint8_t m_ldrOptimize = 0;
    uint8_t m_crcTypeLoRa = 0;
    uint32_t m_preambleLengthLoRa = 0;
    uint8_t m_headerType = 0;
    size_t m_implicitLen = 0;
    uint8_t m_syncWord = 0;
    fp32 m_currentLimit = 0;
    int8_t m_power = 0;
    fp32 m_frequency = 0;
    bool m_isLoRabegined = false;

};

#define LORA_TRY(expr)                          \
    do {                                        \
        LoRa::LoraError _e = (expr);            \
        if (_e != LoRa::LoraError::OK) {        \
            return _e;                          \
        }                                       \
    } while (0)



class SX1268: public LoRa{
public:
    typedef struct {
        GPIO_TypeDef *port;
        uint16_t      pin;
    } GpioPin;

    enum class RadioModeType_t {
    RADIO_MODE_NONE = 0,
    RADIO_MODE_STANDBY,
    RADIO_MODE_RX,
    RADIO_MODE_TX,
    RADIO_MODE_SCAN,
    RADIO_MODE_SLEEP,
    };

    typedef struct {
        SPI_HandleTypeDef *hspi;

        GpioPin cs;      // 片选(输出)
        GpioPin busy;    // 忙标志(输入)—— 事务前等它拉低
        GpioPin rst;     // 复位 NRST(输出,低有效)
        GpioPin dio1;    // 中断(输入)—— TxDone/RxDone
        GpioPin rxen;    // 射频开关(输出)—— E22 特有
        GpioPin txen;    // 射频开关(输出)—— E22 特有

    } SX1268PinConfig;

    // 射频状态
    typedef enum {
        Idle,
        Tx,
        Rx
    } RfMode;

    // 振荡器模式
    typedef enum { 
        RC,
        XOSC 
    }StandbyMode; 

private:
    SX1268PinConfig m_PinConfig;
    RadioModeType_t m_radioType;
    

private:
    uint32_t m_busTimeoutCount;
    const char* m_chipType = NULL;
    uint32_t        m_tcxoDelay = 0;
    fp32 m_tcxoVoltage = 0;
public:
    SX1268(SX1268PinConfig &config);
    LoraError beginLoRa(const ConfigLoRa_t& config) override;
    LoraError setFrequency(fp32 freq) override;
    LoraError setOutputPower(int8_t power) override;
    LoraError transmit(const uint8_t* data, size_t len) override;
    LoraError receive(uint8_t* data, size_t len, uint32_t timeout) override;
    // 总线锁
    class SPIGuard
    {
    public:
        SPIGuard(const SX1268PinConfig& cfg, uint32_t timeoutMs)
            : m_cfg(cfg),   
              m_locked(SPI_BusLock(cfg.hspi, timeoutMs) == HAL_OK)
        {
            if (m_locked)
                HAL_GPIO_WritePin(m_cfg.cs.port, m_cfg.cs.pin, GPIO_PIN_RESET);
        }

        ~SPIGuard()
        {
            if (m_locked) {
                HAL_GPIO_WritePin(m_cfg.cs.port, m_cfg.cs.pin, GPIO_PIN_SET);
                SPI_BusUnlock(m_cfg.hspi);
            }
        }

        bool ok() const { return m_locked; }

        SPIGuard(const SPIGuard&)            = delete;
        SPIGuard& operator=(const SPIGuard&) = delete;

    private:
        const SX1268PinConfig& m_cfg;
        bool                   m_locked;
    };

    // 抓虫临时函数
    LoraError getStatusRaw(uint8_t& status);

private:
    bool findChip(const char* verStr);
    LoraError reset(bool verify);
    LoraError standby(StandbyMode mode);
    void setRfMode(RfMode mode);
    bool isBusy();
    LoraError waitBusy();
    LoraError parseStatus(uint8_t status);
    LoraError configLoRa();
    LoraError calibrateImage(fp32 freq);
    bool isGetIrq();

// Hand函数
    
    LoraError startTransmit(const uint8_t* data, size_t len);
    LoraError startReceive(size_t len, uint32_t timeout);
    LoraError finishTransmit();
    LoraError launchMode(RfMode mode, uint32_t timeout);
    LoraError finishReceive(); 
    LoraError readData(uint8_t* data, size_t len);
    
// Get相关
    LoraError getDeviceErrors(uint16_t* opError);
    LoraError getPacketType(uint8_t* packetType); 
    uint32_t getTimeOnAir(size_t len);
    uint32_t getIrqFlags();    
    LoraError getRxBufferStatus(uint8_t& payloadLength, uint8_t& bufferOffset, bool isUseDummy);                                                                  

// Set相关
    LoraError soluteModlationParams(uint8_t spreadingFactor, fp32 bandwidthKhz, uint8_t codingRate);
    LoraError setRegulatorMode(uint8_t mode);
    LoraError setBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress);
    LoraError setModulationParams(uint8_t spreadingFactor, uint8_t bandwidth, uint8_t codingRate);
    LoraError setSyncWord(uint8_t syncWord);
    LoraError setPacketParams(uint16_t preambleLen, uint8_t payloadLen);
    LoraError setCurrentLimit(fp32 currentLimit);
    LoraError disableDio2RfSwitch();
    LoraError setSpreadingFactor(uint8_t sf);
    LoraError applyPaClampingWorkaround();
    LoraError setTxParams(int8_t powerDbm, uint8_t rampTime);
    LoraError setPaConfig(uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut);
    LoraError clearIrqStatus();
    LoraError setDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask);
    LoraError fixSensitivity();
    LoraError setRx(uint32_t timeout);
    LoraError setTx(uint32_t timeout);
    LoraError setTCXO(fp32 voltage, uint32_t delay);
    LoraError clearDeviceErrors();

// 通信相关
    // 次级抽象
    LoraError SPIreadRegister(uint16_t regsite, size_t numBytes, uint8_t* rxBuff);
    LoraError SPIwriteStream(uint16_t cmd, const uint8_t* data, size_t numBytes);
    LoraError SPIwriteRegister(uint16_t regsite, size_t numBytes, const uint8_t* txData);
    LoraError SPIwriteBuffer(const uint8_t* data, uint8_t numBytes, uint8_t offset);
    // LoraError SPIwriteBuffer(const uint8_t* data, uint8_t numBytes);
    LoraError SPIreadStream(uint8_t opcode, uint8_t* data, size_t numBytes, bool isUseDummy = true);
    LoraError SPIreadBuffer(uint8_t* data, uint8_t numBytes, uint8_t offset);

    // 底层
    LoraError SPITransferStream(const uint8_t* cmd, uint8_t cmdLen, bool write, const uint8_t* pTxData, uint8_t* pRxData, size_t numBytes, bool isWaitForGPIO);
    LoraError spiSendReceiveBuffer(uint8_t* txBuf, size_t len, uint8_t* rxBuf);
};