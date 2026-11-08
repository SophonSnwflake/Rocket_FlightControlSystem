/**
 ******************************************************************************
 * @file           : dvc_imu.cpp
 * @brief          : IMU解析库
 ******************************************************************************
 * @attention
 *
 * Copied from Gmaster's GSRL library, with modifications for the Rocket Flight Control project.
 * Copyright (c) 2026 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */
#pragma once    

#include "Eigen/Core"
#include "RSL_common.h"
#include "cmsis_os.h"
#include "stm32f411xe.h"
#include "alg_ahrs.hpp"
#include "def_bmi088.h"
#include "drv_spi.h"

#define SpiLockTimeoutMs 5
class IMU
{
public:
    // 标准IMU帧
    struct IMUSample{
        uint32_t timestampCycles = 0U; // DWT周期计数
        uint32_t sequence = 0U;

        // 传感器坐标系，已换算物理单位，但未进行零偏和安装旋转补偿
        Eigen::Vector3f gyroRaw = Eigen::Vector3f::Zero(); // rad/s
        Eigen::Vector3f accelRaw = Eigen::Vector3f::Zero(); // m/s^2

        // 机体坐标系，已经过标定和安装旋转
        Eigen::Vector3f angularRateBody = Eigen::Vector3f::Zero(); // rad/s
        Eigen::Vector3f specificForceBody = Eigen::Vector3f::Zero(); // m/s^2
    };
protected:
    using Vector3f = Eigen::Vector3f;
    using Matrix33f = Eigen::Matrix3f;   
    Vector3f m_gyroRawData = Vector3f::Zero(); // 原始陀螺仪数据    // 顺序为:X-Y-Z，使用右手定则，拇指朝向箭头，四指指向方向为正
    Vector3f m_accelRawData = Vector3f::Zero(); // 原始加速度计数据 // 顺序为:X-Y-Z, 朝箭头正方向为正。单位m/s2。包含重力加速度
    Vector3f m_magnetRawData = Vector3f::Zero(); // 原始磁力计数据
    Vector3f m_gyroData = Vector3f::Zero();      // 陀螺仪数据(校准后传入AHRS)
    Vector3f m_accelData = Vector3f::Zero();     // 加速度计数据(校准后传入AHRS)
    Vector3f m_magnetData = Vector3f::Zero();    // 磁力计数据(校准后传入AHRS)
    // 常量定义
    static constexpr fp32 ACCEL_SEN = BMI088_ACCEL_12G_SEN;
    static constexpr fp32 GYRO_SEN  = BMI088_GYRO_2000_SEN;

    bool m_Inited = false;
    uint32_t m_sequence = 0U;

public:
    virtual ~IMU() = default;
    virtual bool init() = 0;
    bool readSample(IMUSample &sample);
    bool isInited() {return m_Inited;}
    Vector3f getGyroRawData(){return m_gyroRawData;}
    Vector3f getAccelRawData(){return m_accelRawData;}
    Vector3f getMagnetRawData(){return m_magnetRawData;}
    virtual bool readRawData() = 0;

protected:
    IMU() = default;
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
        Vector3f gyroOffset = Vector3f::Zero();
        Vector3f accelOffset = Vector3f::Zero();
        Vector3f magnetOffset = Vector3f::Zero();
        Matrix33f installSpinMatrix = Matrix33f::Zero();
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
    BMI088(SPIConfig accelSPIconfig,SPIConfig gyroSPIconfig,CalibrationInfo calibrationInfo, ErrorCallback errorCallback);
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






         
