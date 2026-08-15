#pragma once

#include <cstdint>

namespace Telemetry{

enum class FlightPhase : uint8_t
{
        STANDBY = 0,   // 待命
        SELF_TEST,     // 系统自检
        ARMED,         // 系统已经武装，检测到起飞条件就进入飞行逻辑
        ASCENT,        // 上升段
        DESCENT,       // 下降段
        LANDED         // 落地
};

/**
 * @brief 协议包头魔法数字
 *
 * 以小端排序的魔法数字
 *
 * 52 4B -> ASCII "RK"
 */
constexpr uint16_t PACKET_MAGIC = 0x4B52;

/**
 * @brief 当前遥测协议版本
 */
constexpr uint8_t PROTOCOL_VERSION = 1;

/**
 * @brief 包头长度（字节）
 */
constexpr size_t PACKET_HEADER_SIZE = 8;

/* ==================== 包类型定义 ==================== */

enum class PacketType : uint8_t {
    FlightTelemetry = 0x01,
    GNSSTelemetry = 0x02,
    SystemTelemetry = 0x03,

    Event = 0x10,

    Command = 0x21,
    CommandResponse = 0x22

};

/* ==================== 包头定义 ==================== */

struct PacketHeader{
    uint16_t magic; // 双方约定的魔法数字
    uint8_t version; // 协议版本
    PacketType type; // 包类型
    uint16_t sequence; // 序列
    uint16_t payloadLength; // 载荷长度
};

/* ==================== 遥测载荷 ==================== */

struct FlightTelemetryPayload{
    uint32_t timeStamp_ms;

    // 单位：0.01度
    // 范围：-180 ~ +180deg
    int16_t roll_centidegree;
    int16_t yaw_centidegree;
    int16_t pitch_centidegree;

    // 融合后的高度数据
    // 单位：毫米
    int32_t relative_altitude_mm;

    // 融合后的垂直速度
    // 单位：mm/s
    // 方向：向上为正
    int32_t vertical_velocity_mm_s;

    FlightPhase flight_phase;
};

struct GNSSTelemetryPayload
{
    uint32_t timestamp_ms;

    // 单位：1e-7
    int32_t latitude_deg_e7;
    int32_t longitude_deg_e7;

    // 单位: mm
    int32_t altitude_msl_mm;

    uint8_t fix_type;
    uint8_t num_satellites;
    uint8_t valid_flags;
};

struct SystemTelemetryPayload
{
    uint32_t timestamp_ms;

    // 单位: mV
    uint16_t battery_mv;

    uint32_t log_dropped_count;

    uint8_t system_status;
};

// /* ==================== Protocol Error ==================== */

// enum class ProtocolError : uint8_t
// {
//     OK = 0,

//     InvalidArgument,
//     BufferTooSmall,
//     InvalidMagic,
//     UnsupportedVersion,
//     InvalidLength,
//     UnknownPacketType
// };

}