#include "tsk_isr.h"
#include "crt_imu.h"


extern IMU imu;

void GNSSCallBack(uint8_t *Buffer, uint16_t Length)
{
    imu.receiveGNSSDataFromISR(Buffer, Length);
}
