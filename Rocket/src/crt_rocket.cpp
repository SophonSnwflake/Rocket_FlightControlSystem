#include "crt_rocket.hpp"

extern volatile uint16_t g_last_size;
extern volatile uint32_t g_callback_count;

Rocket::Rocket(IMU *imu, GNSS *gnss):
    m_imu(imu),
    m_gnss(gnss)

{}

void Rocket::Init(){
    DWT_Init();
    SPI_Init(&hspi1, nullptr);
    UART_Init(&huart1,nullptr,100);
    UART_Init(&huart2,uart2Callback,1024);
    m_imu->init();

}

void Rocket::rocketLoop(){
    // m_imu->solveAttitude();
    if(m_gnss->getValid().latitude || m_gnss->getValid().latitude){
    printf("lat: %f, lon: %f\r\n", m_gnss->getLatitude(), m_gnss->getLongitude());
    }
    else{
        // TODO: 临时诊断代码，确认UART2 ISR是否还在触发后删除
        // printf("未收到经纬度信息 callback_count=%u last_size=%u\r\n", g_callback_count, g_last_size);
    }
}

