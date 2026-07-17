#pragma once
#include "RSL_common.h"
#include "alg_general.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "dvc_imu.hpp"
#include "alg_ahrs.hpp"
#include "drv_uart.h"
#include "dvc_gnss.hpp"
#include "tsk_isr.hpp"
#include "drv_time.h"

class Rocket
{
protected:
    IMU *m_imu;
    GNSS *m_gnss;

public:
    Rocket(IMU *imu, GNSS *gnss);
    virtual ~Rocket() = default;
    void Init();
    void rocketLoop();
};