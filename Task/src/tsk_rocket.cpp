#include "FreeRTOS.h"
#include "task.h"
#include "crt_rocket.h"
#include "dvc_imu.hpp"

// 初始化IMU
BMI088::CalibrationInfo cali = {
    {0.0f, 0.0f, 0.0f}, // gyroOffset
    {0.0f, 0.0f, 0.0f}, // accelOffset
    {0.0f, 0.0f, 0.0f}, // magnetOffset
    {RSLMath::Matrix33f::ROTATION, MATH_PI / 2.0f, {0.0f, 0.0f, 1.0f}}}; // 纠正C板绕Z轴逆时针安装90°误差



BMI088 imu(nullptr,
           {&hspi1, GPIOB, GPIO_PIN_12},
           {&hspi1, GPIOB, GPIO_PIN_10},
           cali,
           nullptr 
           ); 

using Vector3f = RSLMath::Vector3f;
using Matrix33f = RSLMath::Matrix33f; 
Vector3f m_gyroRawData; // 原始陀螺仪数据
Vector3f m_accelRawData; // 原始加速度计数据

void imu_test(){
    imu.readRawData();
}

extern "C" void rocket_task(void *argument)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    SPI_Init(&hspi1, nullptr);
    if (imu.init()){
    while (true)
    {
        imu_test();

        vTaskDelayUntil(&last_wake_time, 1);
    }}
}