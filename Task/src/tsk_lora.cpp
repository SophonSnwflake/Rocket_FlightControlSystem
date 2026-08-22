#include "FreeRTOS.h"
#include "task.h"
#include "app_rocket.hpp"

extern Rocket rocket;

namespace RocketLog
{
extern "C" void lora_task(void *argument)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    while (true)
    {
        rocket.communicationLoop();
        vTaskDelayUntil(&last_wake_time, 1);
    }
}
}