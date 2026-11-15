#include "FreeRTOS.h"
#include "task.h"
#include "app_logger.hpp"
#include "app_rocket.hpp"
#include "app_systemview.h"

extern Rocket rocket;

namespace RocketLog
{
extern "C" void log_task(void *argument)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    TickType_t last_stack_sample_time = last_wake_time;
    const TickType_t stack_sample_period = pdMS_TO_TICKS(1000U);
    while (true)
    {
        // rocket.loggerWriteLoop();
        const TickType_t now = xTaskGetTickCount();
        if ((now - last_stack_sample_time) >= stack_sample_period)
        {
            last_stack_sample_time = now;
            AppSystemView_SampleTaskStacks();
        }
        vTaskDelayUntil(&last_wake_time, 1);
    }
}
}
