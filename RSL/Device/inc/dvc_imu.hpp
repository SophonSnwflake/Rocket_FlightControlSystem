#pragma once    

#include "RSL_common.h"
#include "alg_general.hpp"
#include "cmsis_os.h"
#include "stm32f411xe.h"
#include "alg_ahrs.hpp"
#include "def_bmi088.h"
#include "drv_spi.h"

#define SpiLockTimeoutMs 5
class IMU
{
// protected:
//仅供调试
public:
    using Vector3f = RSLMath::Vector3f;
    using Matrix33f = RSLMath::Matrix33f;   
    AHRS *m_ahrs; // 指向AHRS对象的指针
    Vector3f m_gyroRawData; // 原始陀螺仪数据
    Vector3f m_accelRawData; // 原始加速度计数据
    Vector3f m_magnetRawData; // 原始磁力计数据
    Vector3f m_gyroData;      // 陀螺仪数据(校准后传入AHRS)
    Vector3f m_accelData;     // 加速度计数据(校准后传入AHRS)
    Vector3f m_magnetData;    // 磁力计数据(校准后传入AHRS)
    // 常量定义
    static constexpr fp32 ACCEL_SEN = BMI088_ACCEL_3G_SEN;
    static constexpr fp32 GYRO_SEN  = BMI088_GYRO_2000_SEN;

    bool m_Inited = false;

public:
    virtual ~IMU() = default;
    virtual bool init() = 0;
    Vector3f solveAttitude();
    bool isInited() {return m_Inited;}
    Vector3f getGyroRawData(){return m_gyroRawData;}
    Vector3f getAccelRawData(){return m_accelRawData;}
    Vector3f getMagnetRawData(){return m_magnetRawData;}

protected:

    IMU(AHRS *ahrs);
    virtual bool readRawData() = 0;
    virtual void dataCalibration() = 0;
};



class BMI088 : public IMU
{
public:
    enum ErrorCode : uint8_t
    {
        BMI088_NO_ERROR                     = 0x00,
        BMI088_ACC_PWR_CTRL_ERROR           = 0x01,
        BMI088_ACC_PWR_CONF_ERROR           = 0x02,
        BMI088_ACC_CONF_ERROR               = 0x03,
        BMI088_ACC_SELF_TEST_ERROR          = 0x04,
        BMI088_ACC_RANGE_ERROR              = 0x05,
        BMI088_INT1_IO_CTRL_ERROR           = 0x06,
        BMI088_INT_MAP_DATA_ERROR           = 0x07,
        BMI088_GYRO_RANGE_ERROR             = 0x08,
        BMI088_GYRO_BANDWIDTH_ERROR         = 0x09,
        BMI088_GYRO_LPM1_ERROR              = 0x0A,
        BMI088_GYRO_CTRL_ERROR              = 0x0B,
        BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
        BMI088_GYRO_INT3_INT4_IO_MAP_ERROR  = 0x0D,
        BMI088_SELF_TEST_ACCEL_ERROR        = 0x80,
        BMI088_SELF_TEST_GYRO_ERROR         = 0x40,
        BMI088_NO_SENSOR                    = 0xFF
    };

    typedef void (*ErrorCallback)(ErrorCode errorCode);


    // struct TemperatureCtrlConfig {
    //     TIM_HandleTypeDef *htim; // 定时器句柄
    //     uint32_t channel;        // 定时器通道
    //     Controller *controller;  // 温度控制器接口
    // };
    struct SPIConfig{
        SPI_HandleTypeDef *hspi;
        GPIO_TypeDef *csGPIOPort;
        uint16_t csPin;
    };

    struct CalibrationInfo{
        Vector3f gyroOffset;
        Vector3f accelOffset;
        Vector3f magnetOffset;
        Matrix33f installSpinMatrix;
    };

private:
    SPIConfig m_accelSPIConfig;
    SPIConfig m_gyroSPIConfig;
    const CalibrationInfo m_calibrationInfo;
    ErrorCallback m_errorCallback;
    ErrorCode m_errorCode;
private:
    fp32 m_temperature;                      
    uint32_t m_busTimeoutCount;
    uint16_t m_tempDivider;
    int16_t m_accelCounts[3];
    int16_t m_gyroCounts[3];

    class SPIGuard
    {
    public:
        SPIGuard(const SPIConfig& cfg, uint32_t timeoutMs)
            : m_cfg(cfg),   
              m_locked(SPI_BusLock(cfg.hspi, timeoutMs) == HAL_OK)
        {
            if (m_locked)
                HAL_GPIO_WritePin(m_cfg.csGPIOPort, m_cfg.csPin, GPIO_PIN_RESET);
        }

        ~SPIGuard()
        {
            if (m_locked) {
                HAL_GPIO_WritePin(m_cfg.csGPIOPort, m_cfg.csPin, GPIO_PIN_SET);
                SPI_BusUnlock(m_cfg.hspi);
            }
        }

        bool ok() const { return m_locked; }

        SPIGuard(const SPIGuard&)            = delete;
        SPIGuard& operator=(const SPIGuard&) = delete;

    private:
        const SPIConfig& m_cfg;
        bool             m_locked;
    };


public:
    BMI088(AHRS *ahrs, SPIConfig accelSPIconfig,SPIConfig gyroSPIconfig,CalibrationInfo calibrationInfo, ErrorCallback errorCallback);
    bool init() override;
    uint32_t getBusTimeoutCount() {return m_busTimeoutCount;}

protected:


    bool readRawData() override;
    void dataCalibration() override;
    bool selfTestAccel();
    bool selfTestGyro();
    bool initAccel();
    bool initGyro();
    inline bool readSingleReg(const SPIConfig &SPIconfig, uint8_t reg, uint8_t &prxData);
    bool readMutipleReg(const SPIConfig &SPIconfig, uint8_t reg, uint8_t *prxData, uint8_t length);
    bool writeSingleReg(const SPIConfig &SPIconfig, uint8_t reg,uint8_t txData);
    void handleError(ErrorCode errorcode);



};






         