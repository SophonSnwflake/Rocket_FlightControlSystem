#include "FreeRTOS.h"
#include "task.h"
#include "mid_logger.hpp"
#include "crt_rocket.hpp"

extern Rocket rocket;

namespace RocketLog
{
extern "C" void log_task(void *argument)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    while (true)
    {
        rocket.rocketTotalLoop();
        vTaskDelayUntil(&last_wake_time, 1);
    }
}
}