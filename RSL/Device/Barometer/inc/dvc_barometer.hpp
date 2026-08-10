#pragma once

#include "RSL_common.h"
#include "alg_general.hpp"
#include "cmsis_os.h"
#include "stm32f411xe.h"
#include "cmsis_os2.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "drv_time.h"
#include "def_bmp388.h"
#include "drv_spi.h"

#define BARO_TRY(expr)                                    \
    do {                                                  \
        Barometer::BarometerError _e = (expr);            \
        if (_e != Barometer::BarometerError::OK) {        \
            return _e;                                    \
        }                                                 \
    } while (0)

class Barometer
{
public:
    typedef enum : uint8_t {
        OK,
        COMM_FAIL,
        BAD_PARAM,
        NOT_INITIALIZED,
        DEVICE_NOT_FOUND,
        BUS_TIMEOUT,
        DEVICE_NOT_READY,
        DEVICE_ERROR,
    } BarometerError;


public:
    Barometer() = default;
    virtual ~Barometer() = default;
    virtual BarometerError init() = 0;


};

class BMP388 final : public Barometer
{
public:

    enum class Oversampling : uint8_t
    {
        X1  = 0x00,
        X2  = 0x01,
        X4  = 0x02,
        X8  = 0x03,
        X16 = 0x04,
        X32 = 0x05
    };
    enum class OutputDataRate : uint8_t
    {
        Hz200   = 0x00,
        Hz100   = 0x01,
        Hz50    = 0x02,
        Hz25    = 0x03,
        Hz12_5  = 0x04,
        Hz6_25  = 0x05,
        Hz3_125 = 0x06,
        Hz1_563 = 0x07,
        Hz0_781 = 0x08,
        Hz0_391 = 0x09,
        Hz0_195 = 0x0A
    };
    enum class IIRFilter : uint8_t
    {
        Off  = 0x00,
        C2   = 0x01,
        C4   = 0x02,
        C8   = 0x03,
        C16  = 0x04,
        C32  = 0x05,
        C64  = 0x06,
        C128 = 0x07
    };
    typedef struct {
        double parT1 = 0.0;
        double parT2 = 0.0;
        double parT3 = 0.0;

        double parP1 = 0.0;
        double parP2 = 0.0;
        double parP3 = 0.0;
        double parP4 = 0.0;
        double parP5 = 0.0;
        double parP6 = 0.0;
        double parP7 = 0.0;
        double parP8 = 0.0;
        double parP9 = 0.0;
        double parP10 = 0.0;
        double parP11 = 0.0;

        // 温度补偿时计算，压力补偿时使用
        double tLin = 0.0;
    }CalibrationData;

    enum class PowerMode : uint8_t
    {
        Sleep  = 0x00,
        Forced = 0x01,
        Normal = 0x03
    };

    typedef struct {
        SPI_HandleTypeDef *spiHandle;
        GPIO_TypeDef *cs_port;
        uint16_t cs_pin;
        uint8_t m_chipID;
        uint8_t dummy_byte;
        bool initialized;
    } BMP388_HandleTypeDef;

    typedef struct 
    {
        Oversampling pressureOversampling{
            Oversampling::X8
        };

        Oversampling temperatureOversampling{
            Oversampling::X2
        };

        OutputDataRate outputDataRate{
            OutputDataRate::Hz50
        };

        IIRFilter iirFilter{
            IIRFilter::C4
        };
    } BMP388Config;

    // 总线锁
    class SPIGuard
    {
    public:
        SPIGuard(const BMP388_HandleTypeDef& cfg, uint32_t timeoutMs)
            : m_cfg(cfg),   
              m_locked(SPI_BusLock(cfg.spiHandle, timeoutMs) == HAL_OK)
        {
            if (m_locked)
                HAL_GPIO_WritePin(m_cfg.cs_port, m_cfg.cs_pin, GPIO_PIN_RESET);
        }

        ~SPIGuard()
        {
            if (m_locked) {
                HAL_GPIO_WritePin(m_cfg.cs_port, m_cfg.cs_pin, GPIO_PIN_SET);
                SPI_BusUnlock(m_cfg.spiHandle);
            }
        }

        bool ok() const { return m_locked; }

        SPIGuard(const SPIGuard&)            = delete;
        SPIGuard& operator=(const SPIGuard&) = delete;

    private:
        const BMP388_HandleTypeDef& m_cfg;
        bool                   m_locked;
    };

private:
    BMP388_HandleTypeDef m_handleTypeDef;
    CalibrationData m_calibData;
    BMP388Config m_config;

private:
    bool m_initialized = false;
    uint32_t m_busTimeoutCount = 0;
    bool m_calibrationValid = false;

public:
    explicit BMP388(BMP388_HandleTypeDef handleTypeDef, BMP388Config config);

    // 对外主要接口
    BarometerError init() override;
    BarometerError read();

    // 运行状态控制
    BarometerError setMode(PowerMode mode);

    // 安全地重新配置传感器
    BarometerError reconfigure(const BMP388Config& config);

private:
    // ==================== 初始化与配置 ====================

    BarometerError softReset();
    BarometerError waitCommandReady();

    BarometerError configure(const BMP388Config& config);
    BarometerError validateConfig(const BMP388Config& config) const;

    BarometerError setOversampling(Oversampling pressureOversampling, Oversampling temperatureOversampling);
    BarometerError setOutputDataRate(OutputDataRate outputDataRate);
    BarometerError setIIRFilter(IIRFilter iirFilter);
    BarometerError enableSensors(bool enablePressure, bool enableTemperature);

    // ==================== 校准参数处理 ====================

    BarometerError readCalibration();
    void parseCalibrationData(const uint8_t* regData);

    // ==================== 原始数据与补偿 ====================

    BarometerError parseRawData();
    BarometerError compensateTemperature();
    BarometerError compensatePressure();

    // ==================== 寄存器访问 ====================

    BarometerError readRegister(uint8_t regAddr, uint8_t* data);
    BarometerError readRegisters(uint8_t regAddr, uint8_t* data, size_t length);
    BarometerError writeRegister(uint8_t regAddr, uint8_t data);

    // ==================== SPI 底层事务 ====================

    BarometerError SPITransferStream(const uint8_t* cmd, uint8_t cmdLen, bool write, const uint8_t* txData, uint8_t* rxData, size_t numBytes);
    BarometerError spiSendReceiveBuffer(uint8_t* txBuffer, size_t length, uint8_t* rxBuffer);

    // ==================== 工具函数 ====================

    static uint16_t combineBytes(uint8_t msb, uint8_t lsb);
};