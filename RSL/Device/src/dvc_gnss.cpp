#include "dvc_gnss.hpp"
#include "drv_time.h"
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

NEOM9N_UART::NEOM9N_UART(){
    m_para_size = 0;
    m_data_ready = false;
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
    m_longitude = lon_raw * 1e-7;
    int32_t lat_raw = readI4(&payload[28]);
    m_latitude = lat_raw * 1e-7;
    int32_t hmsl_raw = readI4(&payload[36]);
    m_altitude = hmsl_raw / 1000.0f;
    uint32_t h_acc_raw = readU4(&payload[40]);
    m_h_accuracy = h_acc_raw / 1000.0f;
    uint32_t v_acc_raw = readU4(&payload[44]);
    m_v_accuracy = v_acc_raw / 1000.0f;
    int32_t velN_raw = readI4(&payload[48]);
    int32_t velE_raw = readI4(&payload[52]);
    int32_t velD_raw = readI4(&payload[56]);
    m_velocity_north = velN_raw / 1000.0f;  // mm/s转m/s
    m_velocity_east  = velE_raw / 1000.0f;
    m_velocity_down  = velD_raw / 1000.0f;
    uint32_t s_acc_raw = readU4(&payload[68]);
    m_speed_accuracy = s_acc_raw / 1000.0f;

    if (m_fixType >= FIX_GPS) {
    m_valid.time = 1;
    m_valid.latitude = 1;
    m_valid.longitude = 1;
    m_valid.satellite = 1;
    m_valid.altitude = 1;
    m_valid.velocity =1;
}
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
        // TODO: 临时诊断代码，确认GNSS模块实际发送的协议(NMEA/UBX)后删除
        printf("GNSS raw len=%u: ", m_para_size);
        UART_SendData(&huart1,m_rxBuffer,m_para_size);
        printf("\r\n");
        parseUBXdata(m_rxBuffer);
    }
    m_memoryState = FREE;
}

void NEOM9N_UART::receiveGNSSMessageFromUART(uint8_t *pRxData, uint16_t rxDataLength){
    if(m_memoryState!= BUSY){
    if (rxDataLength > sizeof(m_rxBuffer)) return;
    memcpy(m_rxBuffer, pRxData, rxDataLength);
    m_para_size = rxDataLength;
    m_memoryState = LOADED;
    }
    else{
        return;
    }
}
























// void GNSS::parseNMEAdata(uint8_t *pData){
//     char line[512];
//     char *found = NULL;
// 	char *checksum = NULL;
// 	char checksum_read[2];
// 	bool end = false;
//     fp32 latitude_temp;
//     fp32 longitude_temp;

//     do{
//         end = false;
//         found = strstr((char*)pData, NMEA_GGA);
//         if (found != NULL){
//             found -= 3;
//             if (found[0] != '$')
//             break;
//             checksum = strstr(found, "*");
//             if (checksum == NULL)
//             break;
//             checksum_read[0] = *(checksum + 1);
//             checksum_read[1] = *(checksum + 2);
//             memset(line, 0, sizeof(line));
//             int cpy = checksum - found + 3;
//             if (cpy > sizeof(line))
//             break;
//             strncpy(line, found, cpy);
//             uint8_t c = nmea_checksum(line);
//             int cc = 0;
//             sscanf(checksum_read, "%X", &cc);
//             if(cc == c){
//                 char *str = strtok(line, ",*");
//                 uint8_t index = 0;
//                 while(str != NULL){
//                     str = strtok(NULL,",*");
//                 switch (index){
//                     case 0:
//                         if (str[0] < '0' || str[0] > '9')
//                         break;
//                         m_time_h = ((str[0] - 48) * 10) + (str[1] - 48);
//                         m_time_m = ((str[2] - 48) * 10) + (str[3] - 48);
//                         m_time_s = ((str[4] - 48) * 10) + (str[5] - 48);
//                         m_time_ss = ((str[7] - 48) * 10) + (str[8] - 48);
//                         m_valid.time = true;
//                     break;
//                     case 1:
//                         if (str[0] < '0' || str[0] > '9')
//                             break;
//                         latitude_temp = (fp32)atof(str);
//                     break;
//                     case 2:
//                         if (strcmp(str, "S") == 0)
//                         {
//                             m_latitude = -nmea_convert(latitude_temp);
//                             m_valid.latitude = 1;
//                         }
//                         else if (strcmp(str, "N") == 0)
//                         {
//                             m_latitude = nmea_convert(latitude_temp );
//                             m_valid.latitude = 1;
//                         }
//                     break;
//                     case 3:
//                         if (str[0] < '0' || str[0] > '9')
//                             break;
//                         longitude_temp = (float)atof(str);
//                     break;
//                     case 4:
//                         if (strcmp(str, "W") == 0)
//                         {
//                             m_longitude = -nmea_convert(longitude_temp);
//                             m_valid.longitude = 1;
//                         }
//                         else if (strcmp(str, "E") == 0)
//                         {
//                             m_longitude = nmea_convert(longitude_temp);
//                             m_valid.longitude = 1;
//                         }
//                     break;
//                     case 5:
//                         if (str[0] < '0' || str[0] > '9')
//                             break;
//                         m_fixType = (GNSS_FixType)atoi(str); 
//                     break;
//                     case 6:
//                         if (str[0] < '0' || str[0] > '9')
//                             break;
//                         m_num_satellites = atoi(str);
//                         m_valid.satellite = 1;
//                     break;
//                     case 7:
//                         if (str[0] < '0' || str[0] > '9')
//                             break;
//                         m_hdop = (float)atof(str);
//                         m_valid.precision = 1;
//                     break;
//                     case 8:
//                         if (str[0] < '0' || str[0] > '9')
//                             break;
//                         m_altitude = (float)atof(str);
//                         m_valid.altitude = 1;
//                     break;
//                     default:
//                         end = true;
//                     break;
//                 }
//                 index ++;
//                 if(end)break;
//                 }
//             }
//         }
//     }
//     while(0);

// do{
//     end = false;
//     found = strstr((char*)pData, NMEA_VTG);
//     if (found != NULL){
//         found -= 3;
//         if (found[0] != '$')
//             break;

//         checksum = strstr(found, "*");
//         if (checksum == NULL)
//             break;

//         checksum_read[0] = *(checksum + 1);
//         checksum_read[1] = *(checksum + 2);

//         memset(line, 0, sizeof(line));

//         int cpy = checksum - found + 3;
//         if (cpy > sizeof(line))
//             break;

//         strncpy(line, found, cpy);

//         uint8_t c = nmea_checksum(line);
//         int cc = 0;
//         sscanf(checksum_read, "%X", &cc);

//         if(cc == c){
//             char *str = strtok(line, ",*");
//             uint8_t index = 0;

//             while(str != NULL){
//                 str = strtok(NULL,",*");

//                 switch (index){

//                 case 0:    
//                     if (str[0] < '0' || str[0] > '9'){
//                         m_valid.tracking = 0;
//                         break;}
//                     m_heading = (float)atof(str);
//                     m_valid.tracking = 1;
//                     break;

//                 case 6:    
//                     if (str[0] < '0' || str[0] > '9'){
//                         m_valid.velocity = 0;
//                         break;}
//                     m_ground_speed = (float)atof(str);
//                     m_ground_speed = m_ground_speed * 3.6f;
//                     m_valid.velocity = 1;
//                     break;

//                 default:
//                     end = true;
//                     break;
//                 }

//                 index++;

//                 if(end)
//                     break;
//             }
//         }
//     }
// }
// while(0);
// if(m_valid.velocity && m_valid.tracking){
    
// }

// }