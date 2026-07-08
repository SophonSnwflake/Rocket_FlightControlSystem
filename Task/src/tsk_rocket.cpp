#include "FreeRTOS.h"
#include "task.h"
#include "crt_rocket.h"

extern "C" void rocket_task(void *argument)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    while (true)
    {
        vTaskDelayUntil(&last_wake_time, 1);
    }
}