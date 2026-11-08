#include "dvc_gnss.hpp"
#include "drv_time.h"
#include "FreeRTOS.h"
#include "task.h"
#include <cstring>
#include <stdio.h>



const char *nmea_delimiter[] = {

	"GGA", 	// 	Global Positioning System Fix Data
	"VTG", 	//	Recommended minimum specific GPS/Transit data
	"HDT", 	// 	Heading - Heading True
	"HDM",	// 	Heading - Heading Magnetic
	"DPT",  //	Depth
	"MTW",  //	Water temperature
    };

#define		NMEA_GGA		nmea_delimiter[0]
#define		NMEA_VTG		nmea_delimiter[1]
#define		NMEA_HDT		nmea_delimiter[2]
#define		NMEA_HDM		nmea_delimiter[3]
#define		NMEA_DPT		nmea_delimiter[4]
#define		NMEA_MTW		nmea_delimiter[5]

GNSS::GNSS(){
    memset(&m_valid, 0, sizeof(m_valid));
    m_fixType = FIX_NONE;
}

bool GNSS::readSample(GNSSSample &sample){
    bool hasSample = false;

    taskENTER_CRITICAL();
    if (m_hasUnreadSample) {
        sample = m_latestSample;
        m_hasUnreadSample = false;
        hasSample = true;
    }
    taskEXIT_CRITICAL();

    return hasSample;
}

NEOM9N_UART::NEOM9N_UART(){
    m_para_size = 0;
}

uint8_t GNSS::nmea_checksum(const char *sentence)
{
	const char *n = sentence + 1;
  uint8_t chk = 0;
  while ('*' != *n && '\r' != *n)
	{
		if ('\0' == *n || n - sentence > 128)
      return 0;
    chk ^= (uint8_t) *n;
    n++;
  }
  return chk;
}


void NEOM9N_UART::parseUBXdata(uint8_t *pData){
    uint16_t i = 0;
    while (i + 1 < m_para_size){
        if (pData[i] == 0xB5 && pData[i+1] == 0x62){
            if((i+6)>m_para_size)break;
            uint16_t length = pData[i+4] | (pData[i+5] << 8);
            uint16_t msg_size = 8 + length;
            if(i + msg_size > m_para_size) break;
            parseUBXMessage(&pData[i], msg_size);
            i += msg_size;
        }
        else{
            i ++;
        }
    }
    memset(pData,0,2048);
}

void NEOM9N_UART::parseUBXMessage(uint8_t *message, uint16_t message_size){
    uint8_t message_class = message[2];
    uint8_t message_id = message[3];
    uint16_t length = message[4] | (message[5] << 8);
    uint8_t *payload = &message[6];
    if (!verifyChecksum(message, message_size)) {
        return;  // 校验失败，丢弃
    }
    if (message_class == 0x01 && message_id == 0x07) {
        parseNavPvt(payload, length);  // NAV-PVT
    }

}

void NEOM9N_UART::parseNavPvt(uint8_t *payload, uint16_t length){
    if (length != 92) return;
    static constexpr fp64 DEGREE_E7_TO_RADIAN =
        3.14159265358979323846 / (180.0 * 1.0e7);
    static constexpr fp64 MILLIMETER_TO_METER = 0.001;

    m_iTOW = readU4(&payload[0]);
    m_time_y = payload[4] | (payload[5] << 8);
    m_time_mon = payload[6];
    m_time_d   = payload[7];
    m_time_h   = payload[8];
    m_time_m   = payload[9];
    m_time_s   = payload[10];
    m_fixType  = (GNSS_FixType)payload[20];
    m_num_satellites = payload[23];
    // TODO: 临时诊断代码，确认实际定位状态后删除
    printf("NAV-PVT fixType=%u numSV=%u\r\n", payload[20], payload[23]);
    int32_t lon_raw = readI4(&payload[24]);
    m_longitude = lon_raw;
    int32_t lat_raw = readI4(&payload[28]);
    m_latitude = lat_raw;
    int32_t height_ellipsoid_raw = readI4(&payload[32]);
    int32_t hmsl_raw = readI4(&payload[36]);
    m_altitude = hmsl_raw;
    uint32_t h_acc_raw = readU4(&payload[40]);
    m_h_accuracy = h_acc_raw;
    uint32_t v_acc_raw = readU4(&payload[44]);
    m_v_accuracy = v_acc_raw;
    int32_t velN_raw = readI4(&payload[48]);
    int32_t velE_raw = readI4(&payload[52]);
    int32_t velD_raw = readI4(&payload[56]);
    m_velocity_north = velN_raw;  
    m_velocity_east  = velE_raw;
    m_velocity_down  = velD_raw;
    uint32_t s_acc_raw = readU4(&payload[68]);
    m_speed_accuracy = s_acc_raw;

    if (m_fixType >= FIX_GPS) {
    m_valid.time = 1;
    m_valid.latitude = 1;
    m_valid.longitude = 1;
    m_valid.satellite = 1;
    m_valid.altitude = 1;
    m_valid.velocity =1;
}

    GNSSSample nextSample{};
    nextSample.timestampCycles = m_rxTimestampCycles;
    nextSample.sequence = ++m_sampleSequence;
    nextSample.timeOfWeekMs = m_iTOW;

    nextSample.positionBLH <<
        static_cast<fp64>(lat_raw) * DEGREE_E7_TO_RADIAN,
        static_cast<fp64>(lon_raw) * DEGREE_E7_TO_RADIAN,
        static_cast<fp64>(height_ellipsoid_raw) * MILLIMETER_TO_METER;

    nextSample.velocityNED <<
        static_cast<fp32>(velN_raw) * 0.001f,
        static_cast<fp32>(velE_raw) * 0.001f,
        static_cast<fp32>(velD_raw) * 0.001f;

    const fp32 horizontalPositionStd = static_cast<fp32>(h_acc_raw) * 0.001f;
    const fp32 verticalPositionStd = static_cast<fp32>(v_acc_raw) * 0.001f;
    nextSample.positionStdNED <<
        horizontalPositionStd,
        horizontalPositionStd,
        verticalPositionStd;
    nextSample.speedStd = static_cast<fp32>(s_acc_raw) * 0.001f;

    const uint8_t navigationFixType = payload[20];
    if (navigationFixType <= static_cast<uint8_t>(NavigationFixType::TimeOnly)) {
        nextSample.fixType = static_cast<NavigationFixType>(navigationFixType);
    }
    nextSample.satelliteCount = m_num_satellites;

    const uint8_t timeValidityFlags = payload[11];
    const uint8_t navigationFlags = payload[21];
    const bool isGNSSFixValid = (navigationFlags & 0x01U) != 0U;
    const bool has3DPosition =
        navigationFixType == static_cast<uint8_t>(NavigationFixType::Fix3D) ||
        navigationFixType == static_cast<uint8_t>(NavigationFixType::GNSSAndDeadReckoning);

    nextSample.isTimestampValid = (timeValidityFlags & 0x06U) == 0x06U;
    nextSample.isPositionValid = isGNSSFixValid && has3DPosition;
    nextSample.isVelocityValid = isGNSSFixValid && has3DPosition;
    nextSample.isAccuracyValid = isGNSSFixValid && has3DPosition;

    taskENTER_CRITICAL();
    m_latestSample = nextSample;
    m_hasUnreadSample = true;
    taskEXIT_CRITICAL();
}

bool NEOM9N_UART::verifyChecksum(uint8_t *msg, uint16_t msg_size) {
    uint8_t ck_a = 0, ck_b = 0;
    
    // 从Class到Payload末尾（不含同步头和校验和）
    // 即从msg[2]到msg[msg_size-3]（校验和是msg[msg_size-2]和msg[msg_size-1]）
    for (uint16_t i = 2; i < msg_size - 2; i++) {
        ck_a = ck_a + msg[i];
        ck_b = ck_b + ck_a;
    }
    
    // 对比消息末尾的校验和
    uint8_t received_ck_a = msg[msg_size - 2];
    uint8_t received_ck_b = msg[msg_size - 1];
    
    return (ck_a == received_ck_a) && (ck_b == received_ck_b);
}

void NEOM9N_UART::Init(){
    m_memoryState = FREE;
}

void NEOM9N_UART::handleGNSSMessageLoop(){
    if(m_memoryState == LOADED){
        m_memoryState = BUSY;
        parseUBXdata(m_rxBuffer);
    }
    m_memoryState = FREE;
}

void NEOM9N_UART::receiveGNSSMessageFromUART(uint8_t *pRxData, uint16_t rxDataLength){
    if(m_memoryState!= BUSY){
    if (rxDataLength > sizeof(m_rxBuffer)) return;
    m_rxTimestampCycles = DWT_GetTimestamp();
    memcpy(m_rxBuffer, pRxData, rxDataLength);
    m_para_size = rxDataLength;
    m_memoryState = LOADED;
    }
    else{
        return;
    }
}
bool NEOM9N_UART::isHasNewData(){
    if(m_memoryState == LOADED){
        return true;
    }
    return false;
}


