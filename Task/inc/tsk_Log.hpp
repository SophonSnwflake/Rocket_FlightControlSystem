#pragma once

#include "alg_logger_message.hpp"

namespace RocketLog
{

// 初始化 LoggerTask 所需要的所有 Queue
bool initLoggerTask();

// 各生产任务向日志系统提交消息
bool submitIMU(const IMURawMessage& message);

bool submitGNSS(const GNSSMessage& message);

bool submitAHRS(const AHRSMessage& message);

bool submitFlightEstimate(
    const FlightEstimateMessage& message);

bool submitFlightState(
    const FlightStateMessage& message);

bool submitPower(const PowerMessage& message);

bool submitSystemHealth(
    const SystemHealthMessage& message);

// FreeRTOS Logger Task 入口
void LoggerTask(void* argument);

}