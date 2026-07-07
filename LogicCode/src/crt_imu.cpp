#include "crt_imu.h"
#include "common.h"
#include "para_imu.h"
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "drv_uart.h"
#include "tsk_isr.h"
#include "cmsis_compiler.h"
#include <cstdio>
#include <cmath>
#include <cstring>
#include "dvc_gnss.hpp"


void IMU::imuInit()
{
    bmi08_interface_init(&m_bmi08, BMI08_I2C_INTF);
    init_bmi08(&m_bmi08);
    UART_Init(&huart1, GNSSCallBack, 500);
}

void IMU::init_bmi08(struct bmi08_dev *bmi08dev)
{
    int8_t rslt;

    /* ---- Gyroscope ---- */
    int8_t gyro_rslt = bmi08g_init(bmi08dev);
    if (gyro_rslt == BMI08_OK)
    {
        bmi08dev->gyro_cfg.power = BMI08_GYRO_PM_NORMAL;
        rslt = bmi08g_set_power_mode(bmi08dev);
    }
    if (gyro_rslt == BMI08_OK && rslt == BMI08_OK)
    {
        bmi08dev->gyro_cfg.odr   = BMI08_GYRO_BW_32_ODR_100_HZ;
        bmi08dev->gyro_cfg.range = BMI08_GYRO_RANGE_1000_DPS;
        bmi08dev->gyro_cfg.bw    = BMI08_GYRO_BW_32_ODR_100_HZ;
        bmi08dev->gyro_cfg.power = BMI08_GYRO_PM_NORMAL;
        rslt = bmi08g_set_meas_conf(bmi08dev);
    }
    printf("Gyro init %s (chip=0x%x)\r\n",
           (gyro_rslt == BMI08_OK) ? "OK" : "FAIL", bmi08dev->gyro_chip_id);

    /* ---- Accelerometer ---- */
    int8_t accel_rslt = bmi08a_init(bmi08dev);
    if (accel_rslt == BMI08_OK)
    {
        bmi08dev->accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;
        accel_rslt = bmi08a_set_power_mode(bmi08dev);
    }
    if (accel_rslt == BMI08_OK)
    {
        bmi08dev->accel_cfg.odr   = BMI08_ACCEL_ODR_100_HZ;
        bmi08dev->accel_cfg.range = BMI088_MM_ACCEL_RANGE_6G;
        bmi08dev->accel_cfg.bw    = BMI08_ACCEL_BW_NORMAL;
        accel_rslt = bmi08a_set_meas_conf(bmi08dev);
    }
    printf("Accel init %s (chip=0x%x)\r\n",
           (accel_rslt == BMI08_OK) ? "OK" : "FAIL", bmi08dev->accel_chip_id);
}

float IMU::lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width)
{
    double power = 2;
    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));
    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

float IMU::lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    double power = 2;
    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));
    return (dps / (half_scale)) * (val);
}

void IMU::imuLoop()
{
    uint32_t timestamp1 = __HAL_TIM_GET_COUNTER(&htim2);

    /* Gyroscope */
    m_rslt = bmi08g_get_data(&m_gyroData, &m_bmi08);
    if (m_rslt == BMI08_OK)
    {
        m_gyrox = lsb_to_dps(m_gyroData.x, 1000.0f, 16);
        m_gyroy = lsb_to_dps(m_gyroData.y, 1000.0f, 16);
        m_gyroz = lsb_to_dps(m_gyroData.z, 1000.0f, 16);
    }

    /* Accelerometer */
    int8_t a_rslt = bmi08a_get_data(&m_accelData, &m_bmi08);
    if (a_rslt == BMI08_OK)
    {
        m_accelx = lsb_to_mps2(m_accelData.x, 6.0f, 16);
        m_accely = lsb_to_mps2(m_accelData.y, 6.0f, 16);
        m_accelz = lsb_to_mps2(m_accelData.z, 6.0f, 16);
    }
    uint32_t timestamp2 = __HAL_TIM_GET_COUNTER(&htim2);

    // printf("$IMU,%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\r\n",
    //        timestamp1 + (timestamp2 - timestamp1) / 2,
    //        m_gyrox, m_gyroy, m_gyroz,
    //        m_accelx, m_accely, m_accelz);
}

void IMU::receiveGNSSDataFromISR(const uint8_t *rxData, uint16_t length)
{
    if (length > GNSS_COPY_BUFFER_SIZE)
    {
        length = GNSS_COPY_BUFFER_SIZE;
    }
    memcpy(m_gnssRxCopy, rxData, length);
    m_gnssRxLen = length;
}

void IMU::gnssLoop()
{
    if (m_gnssRxLen == 0)
    {
        return;
    }

    static uint8_t local[GNSS_COPY_BUFFER_SIZE];
    uint16_t n;

    __disable_irq();
    n = m_gnssRxLen;
    if (n > GNSS_COPY_BUFFER_SIZE)
    {
        n = GNSS_COPY_BUFFER_SIZE;
    }
    memcpy(local, m_gnssRxCopy, n);
    m_gnssRxLen = 0;
    __enable_irq();

    if (n > 0)
    {
        printf("%.*s", (int)n, (const char *)local);
    }
}

