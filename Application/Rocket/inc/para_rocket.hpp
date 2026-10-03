#pragma once

#include "math_const.h"

#define GRAVITY_ACCELERATION_M_S2                           9.797f

#define UART_COMMAND_RX_BUFFER_SIZE                         256 // UART命令接收缓冲区大小
#define LORA_COMMAND_RX_BUFFER_SIZE                         255
#define LORA_PRINTF_BUFFER_SIZE                             128U
#define LOG_QUEUE_LENGTH                                    256
#define LAUNCH_ACCEL_CRITICAL_VALUE                         10.0f
#define PARACHUTE_PITCH_CRITICAL_POINT_DEG                  60.0f // 相对于天顶向下的旋转角度。如120度代表地平线向下30度
#define PARACHUTE_MAX_WAITING_TIME                          11.0f // 降落伞展开最晚时间
#define PARACHUTE_PITCH_CONFIRM_TIMES                       10
#define LAUNCH_CONFIRM_TIMES                                10
#define LANDED_CONFRIM_TIMES                                1000 // 判断是否着陆次数
#define ALTITUDE_BARO_LANDED_STANDARD_M                     10 // 使用气压计高度判断是否着陆标准(单位：米)
#define PARACHUTE_IGNITE_TIME_MS                            1500  // 降落伞点火持续时间(单位：毫秒)
#define LANDED_WAITING_MIN_TIME_PARACHUTE                   10000 // 降落伞打开后，判定着陆的最小时间
#define LANDED_WAITING_MAX_TIME_PARACHUTE                   30000 // 降落伞打开后，强制判定着陆的最大时间
#define BUZZER_ALARM_PERIOD_MS                              1000U
#define BARO_SAMPLE_TIMES                                   50  // 标准气压温度采样次数

#define LOGGER_QUATERNION_SCALE_FACTOR                      10000.0f 
#define LOGGER_GYRO_BIAS_SCALE_FACTOR                       100000.0f
#define LOGGER_IMU_SCALE_FACTOR                             100.0f

#define TELEMETRY_FLIGHT_PERIOD_MS                          500
#define TELEMETRY_GNSS_PERIOD_MS                            2000
#define TELEMETRY_SYSTEM_PERIOD_MS                          4000
#define TELEMETRY_FLIGHT_PERIOD_STANDBY_MS                  3000

#define LOGGER_IMU_PERIOD_MS                                2
#define LOGGER_IMU_ARMED_PERIOD_MS                          10
#define LOGGER_AHRS_ARMED_PERIOD_MS                         50
#define LOGGER_AHRS_PERIOD_MS                               10
#define LOGGER_FLIGHT_ESTIMATE_PERIOD_MS                    10
#define LOGGER_POWER_MESSAGE_PERIOD_MS                      100
#define LOGGER_SYSTEM_HEALTH_PERIOD_MS                      1000
#define LOGGER_SYNC_PERIOD_MS                               500
#define LOGGER_GNSS_STANDBY_PERIOD_MS                       1000
#define LOGGER_GNSS_ARMED_PERIOD_MS                         500
#define LOGGER_GNSS_LANDED_PERIOD_MS                        5000
