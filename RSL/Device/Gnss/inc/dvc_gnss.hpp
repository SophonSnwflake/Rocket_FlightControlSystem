#pragma once

#include "RSL_common.h"
#include "alg_general.hpp"
#include "cmsis_os.h"
#include "stm32f411xe.h"
#include "drv_uart.h"

class GNSS
{
public:
    typedef struct {
    uint8_t time : 1;       // 时间有效
    uint8_t latitude : 1;   // 纬度有效
    uint8_t longitude : 1;  // 经度有效
    uint8_t satellite : 1;  // 卫星数有效
    uint8_t precision : 1;  // 精度有效
    uint8_t altitude : 1;   // 高度有效
    uint8_t tracking : 1;   // 航向有效
    uint8_t velocity : 1;    // 速度有效
    } valid;

    typedef enum {
    FIX_NONE = 0,        // 无定位
    FIX_GPS = 1,         // GPS单点定位
    FIX_DGPS = 2,        // 差分定位
    FIX_PPS = 3,         // PPS定位
    FIX_RTK_FIXED = 4,   // RTK固定解
    FIX_RTK_FLOAT = 5,   // RTK浮动解
    } GNSS_FixType;

protected:
    //位置
    int32_t m_latitude; //纬度（度 * e7）
    int32_t m_longitude; //经度（度 * e7）
    int32_t m_altitude; //海拔（毫米）

    //速度
    int32_t m_velocity_north; //北向速度 (mm/s)
    int32_t m_velocity_east; //东向速度
    int32_t m_velocity_down; //地向速度
    // fp32 m_ground_speed; //地速
    // fp32 m_heading; //航向

    //精度/质量
    uint32_t m_h_accuracy; //水平精度 (mm)
    uint32_t m_v_accuracy; //垂直精度 (mm)
    uint32_t m_speed_accuracy; //速度精度 (mm/s)
    uint8_t m_fix_type; //定位类型
    uint8_t m_num_satellites; //卫星数量

    //时间
    uint32_t m_iTOW;     // 周内时间
    uint16_t m_time_y;   // 年
    uint8_t m_time_mon;  // 月
    uint8_t m_time_d;    // 月
    uint8_t m_time_h;    // 时
    uint8_t m_time_m;    // 分
    uint8_t m_time_s;    // 秒
    uint8_t m_time_ss;   // 秒的小数

    valid m_valid; // 有效判定
    GNSS_FixType m_fixType; //定位方式

public:
    GNSS();
    virtual ~GNSS() = default;
    virtual void Init() = 0;
    virtual void handleGNSSMessageLoop() = 0;
    virtual void receiveGNSSMessageFromUART(uint8_t *pRxData, uint16_t rxDataLength) = 0;
    virtual bool isHasNewData() = 0;
    // virtual void 
    uint8_t nmea_checksum(const char *sentence);
    int32_t getLatitude(){return m_latitude;}
    int32_t getLongitude(){return m_longitude;}
    int32_t getAltitude(){return m_altitude;}

    int32_t getVelocityNorth(){return m_velocity_north;}
    int32_t getVelocityEast(){return m_velocity_east;}
    int32_t getVelocityDown(){return m_velocity_down;}

    uint32_t getHAccuracy(){return m_h_accuracy;}
    uint32_t getVAccuracy(){return m_v_accuracy;}
    uint32_t getSpeedAccuracy(){return m_speed_accuracy;}
    uint8_t getFixType(){return m_fix_type;}
    uint8_t getNumSatellites(){return m_num_satellites;}

    uint32_t getITOW(){return m_iTOW;}
    uint16_t getTimeY(){return m_time_y;}
    uint8_t getTimeMon(){return m_time_mon;}
    uint8_t getTimeD(){return m_time_d;}
    uint8_t getTimeH(){return m_time_h;}
    uint8_t getTimeM(){return m_time_m;}
    uint8_t getTimeS(){return m_time_s;}
    uint8_t getTimeSs(){return m_time_ss;}

    uint8_t getValid() const
    {
        return
            (static_cast<uint8_t>(m_valid.time)      << 0) |
            (static_cast<uint8_t>(m_valid.latitude)  << 1) |
            (static_cast<uint8_t>(m_valid.longitude) << 2) |
            (static_cast<uint8_t>(m_valid.satellite) << 3) |
            (static_cast<uint8_t>(m_valid.precision) << 4) |
            (static_cast<uint8_t>(m_valid.altitude)  << 5) |
            (static_cast<uint8_t>(m_valid.tracking)  << 6) |
            (static_cast<uint8_t>(m_valid.velocity)  << 7);
    }
    GNSS_FixType getFixTypeEnum(){return m_fixType;}
};

#define USE_GNSS_UART (&huart1)
#define USE_GNSS_MESSAGE_LENGTH 2048

class NEOM9N_UART : public GNSS{
public:
    typedef enum {
    FREE   = 0, // 数据待被填入
    LOADED = 1, // 数据已转运
    BUSY   = 2, // 被占用
    } MemoryState;
protected:  
    uint8_t m_rxBuffer[2048];
    uint16_t m_para_size;
    bool m_data_ready;
    MemoryState m_memoryState;

public:
    NEOM9N_UART();
    void Init() override;
    void receiveGNSSMessageFromUART(uint8_t *pRxData, uint16_t rxDataLength) override;//与UART回调绑定，自动调用
    bool isHasNewData() override;
    void handleGNSSMessageLoop();
    void parseUBXdata(uint8_t *pData);
    void parseUBXMessage(uint8_t *msg, uint16_t msg_size);
    bool verifyChecksum(uint8_t *msg, uint16_t msg_size);
    void parseNavPvt(uint8_t *payload, uint16_t length);
};

// 读4字节小端无符号整数
static inline uint32_t readU4(uint8_t *data) {
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

// 读4字节小端有符号整数
static inline int32_t readI4(uint8_t *data) {
    return (int32_t)readU4(data);
}

// 读2字节小端无符号整数
static inline uint16_t readU2(uint8_t *data) {
    return data[0] | (data[1] << 8);
}