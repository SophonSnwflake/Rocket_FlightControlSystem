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
    virtual void init() = 0; // 初始化IMU
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