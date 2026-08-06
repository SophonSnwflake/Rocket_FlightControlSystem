#pragma once
#include <stdint.h>
#include "RSL_common.h"

enum class ULogMessageId : uint16_t
{
    ImuRaw         = 1U,
    Gnss           = 2U,
    Ahrs           = 3U,
    FlightEstimate = 4U,
    FlightState    = 5U,
    Power          = 6U,
    SystemHealth   = 7U
};


struct IMURawMessage
{
    uint64_t timestamp;
    uint32_t sequence;
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
};

inline constexpr char IMU_RAW_MESSAGE_NAME[] =
    "rocket_imu";

inline constexpr char IMU_RAW_MESSAGE_FORMAT[] =
    "rocket_imu:"
    "uint64_t timestamp;"
    "uint32_t sequence;"
    "int16_t[3] accel_raw;"
    "int16_t[3] gyro_raw;";

enum class GnssValidFlag : uint8_t
{
    Time      = 1U << 0,
    Latitude  = 1U << 1,
    Longitude = 1U << 2,
    Satellite = 1U << 3,
    Precision = 1U << 4,
    Altitude  = 1U << 5,
    Tracking  = 1U << 6,
    Velocity  = 1U << 7
};
    
struct GNSSMessage
{
    // 飞控本地时间，单位：us
    uint64_t timestamp_us;

    // GNSS 周内时间，单位：ms
    uint32_t iTOW_ms;

    // 位置
    int32_t latitude_deg_e7;
    int32_t longitude_deg_e7;
    int32_t altitude_msl_mm;

    // NED 速度，单位：mm/s
    int32_t velocity_north_mm_s;
    int32_t velocity_east_mm_s;
    int32_t velocity_down_mm_s;

    // 精度
    uint32_t h_accuracy_mm;
    uint32_t v_accuracy_mm;
    uint32_t speed_accuracy_mm_s;

    // 状态
    uint8_t valid_flags;
    uint8_t fix_type;
    uint8_t num_satellites;
};

inline constexpr char GNSS_MESSAGE_NAME[] =
    "rocket_gnss";

inline constexpr char GNSS_MESSAGE_FORMAT[] =
    "rocket_gnss:"
    "uint64_t timestamp_us;"
    "uint32_t iTOW_ms;"
    "int32_t latitude_deg_e7;"
    "int32_t longitude_deg_e7;"
    "int32_t altitude_msl_mm;"
    "int32_t velocity_north_mm_s;"
    "int32_t velocity_east_mm_s;"
    "int32_t velocity_down_mm_s;"
    "uint32_t h_accuracy_mm;"
    "uint32_t v_accuracy_mm;"
    "uint32_t speed_accuracy_mm_s;"
    "uint8_t valid_flags;"
    "uint8_t fix_type;"
    "uint8_t num_satellites;";

struct AHRSMessage{
    // 飞控本地时间，单位：us
    uint64_t timestamp_us;

    int32_t quaternion[4];
    int32_t gyroBias[3];
};

inline constexpr char AHRS_MESSAGE_NAME[] =
    "rocket_ahrs";

inline constexpr char AHRS_MESSAGE_FORMAT[] =
    "rocket_ahrs:"
    "uint64_t timestamp_us;"
    "int32_t[4] quaternion;"
    "int32_t[3] gyroBias;";

struct FlightEstimateMessage{
    // 飞控本地时间，单位：us
    uint64_t timestamp_us;
    fp32 relative_altitude_m;
    fp32 vertical_velocity_m_s;
    fp32 vertical_acceleration_m_s2;
    fp32 predicted_apogee_m;
    uint8_t valid_flags;
};

inline constexpr char FLIGHT_ESTIMATE_MESSAGE_NAME[] =
    "rocket_flight_estimate";

inline constexpr char FLIGHT_ESTIMATE_MESSAGE_FORMAT[] =
    "rocket_flight_estimate:"
    "uint64_t timestamp_us;"
    "float relative_altitude_m;"
    "float vertical_velocity_m_s;"
    "float vertical_acceleration_m_s2;"
    "float predicted_apogee_m;"
    "uint8_t valid_flags;";

struct FlightStateMessage{
    // 飞控本地时间，单位：us
    uint64_t timestamp_us;
    uint8_t previous_state;
    uint8_t current_state;
    uint16_t transition_reason;
};

inline constexpr char FLIGHT_STATE_MESSAGE_NAME[] =
    "rocket_flight_state";

inline constexpr char FLIGHT_STATE_MESSAGE_FORMAT[] =
    "rocket_flight_state:"
    "uint64_t timestamp_us;"
    "uint8_t previous_state;"
    "uint8_t current_state;"
    "uint16_t transition_reason;";

struct PowerMessage{
    // 飞控本地时间，单位：us
    uint64_t timestamp_us;
    uint16_t battery_voltage_mv;
};

inline constexpr char POWER_MESSAGE_NAME[] =
    "rocket_power";

inline constexpr char POWER_MESSAGE_FORMAT[] =
    "rocket_power:"
    "uint64_t timestamp_us;"
    "uint16_t battery_voltage_mv;";

struct SystemHealthMessage{
    uint64_t timestamp_us;
    uint32_t error_flags;
    uint16_t imu_error_count;
    uint16_t baro_error_count;
    uint16_t gnss_error_count;
    uint16_t flash_error_count;
    uint32_t imu_dropped_samples;
    uint32_t logger_queue_overflows;
    uint16_t logger_buffer_usage;
};

inline constexpr char SYSTEM_HEALTH_MESSAGE_NAME[] =
    "rocket_system_health";

inline constexpr char SYSTEM_HEALTH_MESSAGE_FORMAT[] =
    "rocket_system_health:"
    "uint64_t timestamp_us;"
    "uint32_t error_flags;"
    "uint16_t imu_error_count;"
    "uint16_t baro_error_count;"
    "uint16_t gnss_error_count;"
    "uint16_t flash_error_count;"
    "uint32_t imu_dropped_samples;"
    "uint32_t logger_queue_overflows;"
    "uint16_t logger_buffer_usage;";