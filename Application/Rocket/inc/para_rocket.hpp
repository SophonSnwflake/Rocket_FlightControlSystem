#pragma once

#include "math_const.h"

#define GRAVITY_ACCELERATION_M_S2                           9.797f

#define UART_COMMAND_RX_BUFFER_SIZE                         256
#define LORA_COMMAND_RX_BUFFER_SIZE                         255
#define LORA_PRINTF_BUFFER_SIZE                             128U
#define LOG_QUEUE_LENGTH                                    64
#define LAUNCH_ACCEL_CRITICAL_VALUE                         10.0f
#define PARACHUTE_PITCH_CRITICAL_POINT                      120.0f/90.0f * MATH_PI
#define PARACHUTE_MAX_WAITING_TIME                          10.0f
#define PARACHUTE_PITCH_CONFIRM_TIMES                       10
#define PARACHUTE_IGNITE_TIME_MS                            1000

#define LOGGER_QUATERNION_SCALE_FACTOR                      10000.0f
#define LOGGER_GYRO_BIAS_SCALE_FACTOR                       100000.0f
#define LOGGER_IMU_SCALE_FACTOR                             100.0f

#define FLIGHT_TELEMETRY_PERIOD_MS                          350
#define FLIGHT_TELEMETRY_PERIOD_STANDBY_MS                  1000
#define VOLTAGE_PROBE_PERIOD_MS                             1000