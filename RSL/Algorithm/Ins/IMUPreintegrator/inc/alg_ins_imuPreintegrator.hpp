/**
 ******************************************************************************
 * @file           : alg_ins_imuPreintegrator.hpp
 * @brief          : INS IMU预积分器接口
 * @brief          : INS IMU preintegrator interface
 ******************************************************************************
 * @attention
 * Copyright (c) Jihao Wang XJTLU.
 * All rights reserved.
 *
 ******************************************************************************
 */

#pragma once

#include "RSL_common.h"
#include "alg_ins_types.hpp"
#include "dvc_imu.hpp"

namespace INS {

// pushSample的处理结果；只有IncrementReady表示输出increment已更新
enum class IMUPreintegrationResult : uint8_t {
    Accumulating = 0,
    IncrementReady,
    NotInitialized,
    InvalidSample,
    InvalidTimeInterval,
    SampleDiscontinuity,
    CalculationFailed
};

struct IMUPreintegratorConfig {
    uint32_t timestampFrequencyHz = 0U; // DWT时间戳频率，通常传入SystemCoreClock
    uint16_t intervalsPerIncrement = 5U; // 每个输出增量包含的IMU采样区间数
    fp64 minimumSampleInterval = 0.0001; // s，允许的最小相邻采样间隔
    fp64 maximumSampleInterval = 0.01; // s，允许的最大相邻采样间隔
    bool validateSequence = false; // 是否要求IMU样本序号严格连续
};

class IMUPreintegrator {
public:
    IMUPreintegrator() = default;
    ~IMUPreintegrator() = default;

    /**
     * @brief 初始化固定区间数IMU预积分器。
     * @note 失败时保留原有配置和运行状态，成功时清空此前累计结果。
     */
    bool init(const IMUPreintegratorConfig &config);

    /**
     * @brief 输入一个按时间排序的IMU样本，并在累计满配置区间数后输出增量。
     * @note 采用梯形积分累计角增量和速度增量，不在此处重复执行机械编排中的
     *       二子样圆锥与划桨补偿。仅返回IncrementReady时才会修改increment。
     */
    IMUPreintegrationResult pushSample(const IMU::IMUSample &sample, IMUIncrement &increment);

    // 清空参考样本和当前累计结果，但保留初始化配置
    void reset();

    bool isInitialized() const { return m_isInitialized; }
    bool hasReferenceSample() const { return m_hasPreviousSample; }
    uint16_t getAccumulatedIntervalCount() const { return m_accumulatedIncrement.intervalCount; }
    const IMUPreintegratorConfig &getConfig() const { return m_config; }

private:
    IMUPreintegratorConfig m_config{};
    bool m_isInitialized = false;
    bool m_hasPreviousSample = false;

private:
    // 预积分状态，与配置变量分开保存
    IMU::IMUSample m_previousSample{};
    IMUIncrement m_accumulatedIncrement{};

private:
    bool isSampleValid(const IMU::IMUSample &sample) const;
    bool isSequenceContinuous(const IMU::IMUSample &sample) const;
    void clearAccumulatedIncrement();
    void restartFromSample(const IMU::IMUSample &sample);
};

}
