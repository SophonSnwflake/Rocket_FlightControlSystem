#include "tsk_imu.h"
#include "crt_imu.h"
#include "cmsis_os2.h"   // osDelay
#include "spi.h"         // CubeMX 生成的 hspi1
#include "i2c.h"         // CubeMX 生成的 hi2c1
#include "dvc_gnss.hpp"

IMU imu;

// 必须用 extern "C"，否则 main.c 找不到这个函数
extern "C" void StartIMUTask(void *argument)
{
    // 初始化（只执行一次）
    // e.g. 配置 IMU 寄存器，发几条 SPI/I2C 命令
    imu.imuInit();
    
    while(1)
    {
        // 周期性逻辑
        // e.g. 读 IMU 数据、更新姿态角
        imu.imuLoop();
        imu.gnssLoop();
        osDelay(1);  // 释放 CPU，单位 ms
    }
}
