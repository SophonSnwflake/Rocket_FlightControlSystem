#include "FreeRTOS.h"
#include "task.h"
#include "app_rocket.hpp"
#include "dvc_imu.hpp"
#include "alg_ahrs.hpp"
#include "drv_uart.h"


extern Rocket rocket;


extern "C" void GNSS_task(void *argument){
    TickType_t last_wake_time = xTaskGetTickCount();
    while (true)
    {
        while(rocket.isInitCompleted()){
        rocket.GNSSLoop();
        }
        vTaskDelayUntil(&last_wake_time, 100);
    }
}