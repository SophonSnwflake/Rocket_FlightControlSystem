#pragma once

#include "RSL_common.h"
#include "alg_general.hpp"
#include "cmsis_os.h"
#include "stm32f411xe.h"

class AHRS
{
protected:
    using Vector3f = RSLMath::Vector3f;
    using Matrix33f = RSLMath::Matrix33f;
    Vector3f m_gyroData;      
    Vector3f m_accelData;     
    Vector3f m_magnetData;    

public:
    virtual ~AHRS() = default;
    virtual void reset();
    virtual void init() = 0;
    const void update(Vector3f m_gyroData, Vector3f m_accelData, Vector3f m_magnetData);
    const Vector3f getEulerAngle();

protected:
    AHRS();
    virtual void dataProcess() = 0;
    void convertQuaternionToEulerAngle();
    void initQuaternion();
    void calculateMotionAccel();   
    
};

class QuaterionEKF : public AHRS{
private:
    
public:

private:
};