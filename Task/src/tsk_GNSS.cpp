#include "FreeRTOS.h"
#include "task.h"
#include "app_rocket.hpp"
#include "dvc_imu.hpp"
#include "alg_ahrs.hpp"
#include "drv_uart.h"


extern NEOM9N_UART gnss;


extern "C" void GNSS_task(void *argument){
    TickType_t last_wake_time = xTaskGetTickCount();
    gnss.Init();
    while (true)
    {
        gnss.handleGNSSMessageLoop();

        vTaskDelayUntil(&last_wake_time, 100);
    }
}