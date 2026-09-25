#pragma once

#include "math_const.h"

#define GRAVITY_ACCELERATION_M_S2                           9.797f

#define UART_COMMAND_RX_BUFFER_SIZE                         256
#define LORA_COMMAND_RX_BUFFER_SIZE                         255
#define LORA_PRINTF_BUFFER_SIZE                             128U
#define LOG_QUEUE_LENGTH                                    256
#define LAUNCH_ACCEL_CRITICAL_VALUE                         10.0f
#define PARACHUTE_PITCH_CRITICAL_POINT_DEG                  120.0f // 相对于天顶向下的旋转角度。如120度代表地平线向下30度
#define PARACHUTE_MAX_WAITING_TIME                          10.0f
#define PARACHUTE_PITCH_CONFIRM_TIMES                       10
#define LAUNCH_CONFIRM_TIMES                                10
#define PARACHUTE_IGNITE_TIME_MS                            1000
#define BUZZER_ALARM_PERIOD_MS                              1000U
#define BARO_SAMPLE_TIMES                                   50  // 标准气压温度采样次数

#define LOGGER_QUATERNION_SCALE_FACTOR                      10000.0f
#define LOGGER_GYRO_BIAS_SCALE_FACTOR                       100000.0f
#define LOGGER_IMU_SCALE_FACTOR                             100.0f

#define FLIGHT_TELEMETRY_PERIOD_MS                          500
#define FLIGHT_TELEMETRY_PERIOD_STANDBY_MS                  3000
#define GNSS_TELEMETRY_PERIOD_MS                            2000
#define SYSTEM_TELEMETRY_PERIOD_MS                          4000
#define VOLTAGE_PROBE_PERIOD_MS                             1000