#pragma once    

#include "RSL_common.h"
#include "alg_general.hpp"

class IMU
{
protected:
    using Vector3f = RSLMath::Vector3f;
    using Vector33f = RSLMath::Vector33f;   
    AHRS *m_ahrs; // 指向AHRS对象的指针
    Vector3f m_gyroRawData; // 原始陀螺仪数据
    Vector3f m_accelRawData; // 原始加速度计数据
    Vector3f m_magnetRawData; // 原始磁力计数据
    Vector3f m_gyroData;      // 陀螺仪数据(校准后传入AHRS)
    Vector3f m_accelData;     // 加速度计数据(校准后传入AHRS)
    Vector3f m_magnetData;    // 磁力计数据(校准后传入AHRS)

public:
    virtual ~IMU() = default;
    virtual bool init() = 0; // 初始化IMU
    const Vector3f &solveAttitude();
    const Vector3f &getGyroData() const;
    const Vector3f &getAccelData() const;
    const Vector3f &getMotionAccelBodyFrame() const;
    const Vector3f &getMotionAccelEarthFrame() const;
    const fp32 *getQuaternion() const;
    const Vector3f &getEulerAngle() const;

protected:
    IMU(AHRS *ahrs);
    virtual void readRawData()     = 0;
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

    struct SPIConfig
    {
        SPI_HandleTypeDef *hspi;
        GPIO_TypeDef *csGPIOPort;
        uint16_t csPin;
    };

protected:
    SPIConfig m_spiConfig;
    

public:
    BMI088(AHRS *ahrs);
    bool init() override;

protected:
    void readRawData() override;
    void dataCalibration() override;
private:
    bool initAccel();
    bool selfTestAccel();
    bool initGyro();
    bool selfTestGyro();

};