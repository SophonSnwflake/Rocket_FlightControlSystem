#include "FreeRTOS.h"
#include "task.h"
#include "crt_rocket.hpp"
#include "dvc_imu.hpp"
#include "alg_ahrs.hpp"
#include "drv_uart.h"
#include "dvc_lora.hpp"
#include "dvc_barometer.hpp"

using Vector3f = RSLMath::Vector3f;
using Matrix33f = RSLMath::Matrix33f; 
// 初始化IMU
BMI088::CalibrationInfo cali = {
    {0.0f, 0.0f, 0.0f}, // gyroOffset
    {0.0f, 0.0f, 0.0f}, // accelOffset
    {0.0f, 0.0f, 0.0f}, // magnetOffset
    {RSLMath::Matrix33f::ROTATION, MATH_PI / 2.0f, {0.0f, 0.0f, 1.0f}}}; // 纠正C板绕Z轴逆时针安装90°误差

QuaternionEKF myEKF(0.0f, 10.0f,0.001f, 1e6f, 1.0f, 0.0f, true, 1e-8f);

BMI088 imu(&myEKF,
           {&hspi1, GPIOB, GPIO_PIN_12},
           {&hspi1, GPIOB, GPIO_PIN_10},
           cali,
           nullptr 
           ); 

NEOM9N_UART gnss;

W25Q128 flash(hspi1, GPIOA, GPIO_PIN_4);

SX1268::SX1268PinConfig loraConfig{
    &hspi2,
    {GPIOA, GPIO_PIN_4},   // cs / NSS
    {GPIOA, GPIO_PIN_8},   // busy
    {GPIOB, GPIO_PIN_8},   // rst
    {GPIOA, GPIO_PIN_11},  // dio1
    {GPIOA, GPIO_PIN_12},  // rxen
    {GPIOA, GPIO_PIN_15}   // txen
};

SX1268 lora(loraConfig);

BMP388::BMP388_HandleTypeDef barometerHandle{
    &hspi1,       // spiHandle
    GPIOB,        // cs_port
    GPIO_PIN_1,  // cs_pin
    0U,           // m_chipID：初始化前未知
    1U,           // dummy_byte：BMP388 使用 SPI 读取时需要 1 个 dummy byte
    false         // initialized：尚未初始化
};

BMP388::BMP388Config barometerConfig{
    BMP388::Oversampling::X4,
    BMP388::Oversampling::X4,
    BMP388::OutputDataRate::Hz0_195,
    BMP388::IIRFilter::C4
};

ActiveBuzzer buzzer(GPIOB, GPIO_PIN_2, false);

BMP388 barometer(barometerHandle, barometerConfig);
Rocket rocket(&imu, &gnss, nullptr, &lora, &barometer, &buzzer);


extern uint16_t g_last_size;
extern uint16_t g_callback_count;

extern "C" void rocket_task(void *argument)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    rocket.Init();
    while (true)
    {
        rocket.rocketTotalLoop();
        vTaskDelayUntil(&last_wake_time, 100);
    }
}