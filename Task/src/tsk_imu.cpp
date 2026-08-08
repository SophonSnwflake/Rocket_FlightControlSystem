#include "FreeRTOS.h"
#include "task.h"
#include "crt_rocket.hpp"
#include "dvc_imu.hpp"
#include "alg_ahrs.hpp"


extern Rocket rocket;
extern BMI088 imu; 

extern "C" void imu_task(void *argument){
    while (!rocket.isInitCompleted()) {
        vTaskDelay(1);
    }
    TickType_t taskLastWakeTime = xTaskGetTickCount(); // 初始化完成后再取基准
    while (1) {
        rocket.imuLoop();
        vTaskDelayUntil(&taskLastWakeTime, 1);
    }
}