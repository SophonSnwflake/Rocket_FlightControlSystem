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
#include "dvc_flash.hpp"
#include "dvc_flash.hpp"

class Rocket
{
public:
    typedef enum {
        SELF_TEST       = 0,
        READY_TO_LAUNCH = 1,
        ERROR           = 2,
        FLYING_UP       = 3,
        LANDING         = 4
    }LaunchPhase;


private:
    IMU *m_imu;
    GNSS *m_gnss;
    W25Q128 *m_flash;

private:
    LaunchPhase m_launchPhase;

public:
    Rocket(IMU *imu, GNSS *gnss, W25Q128 *flash);
    virtual ~Rocket() = default;
    void Init();
    void rocketTotalLoop();
    bool selfTest();
               
};