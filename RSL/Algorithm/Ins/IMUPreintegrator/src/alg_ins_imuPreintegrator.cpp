#include "alg_ins_imuPreintegrator.hpp"

#include <cmath>

namespace INS {

bool IMUPreintegrator::init(const IMUPreintegratorConfig &config) {
    if (config.timestampFrequencyHz == 0U) return false;
    if (config.intervalsPerIncrement == 0U) return false;
    if (!std::isfinite(config.minimumSampleInterval)) return false;
    if (!std::isfinite(config.maximumSampleInterval)) return false;
    if (config.minimumSampleInterval <= 0.0) return false;
    if (config.maximumSampleInterval <= config.minimumSampleInterval) return false;

    m_config = config;
    reset();
    m_isInitialized = true;

    return true;
}

IMUPreintegrationResult IMUPreintegrator::pushSample(const IMU::IMUSample &sample, IMUIncrement &increment) {
    if (!m_isInitialized) return IMUPreintegrationResult::NotInitialized;

    if (!isSampleValid(sample)) {
        reset();
        return IMUPreintegrationResult::InvalidSample;
    }

    if (!m_hasPreviousSample) {
        restartFromSample(sample);
        return IMUPreintegrationResult::Accumulating;
    }

    if (!isSequenceContinuous(sample)) {
        restartFromSample(sample);
        return IMUPreintegrationResult::SampleDiscontinuity;
    }

    const uint32_t deltaCycles = sample.timestampCycles - m_previousSample.timestampCycles;
    const fp64 deltaTime =
        static_cast<fp64>(deltaCycles) / static_cast<fp64>(m_config.timestampFrequencyHz);

    if (!std::isfinite(deltaTime) ||
        deltaTime < m_config.minimumSampleInterval ||
        deltaTime > m_config.maximumSampleInterval) {
        restartFromSample(sample);
        return IMUPreintegrationResult::InvalidTimeInterval;
    }

    const Eigen::Vector3d previousAngularRate = m_previousSample.angularRateBody.cast<fp64>();
    const Eigen::Vector3d currentAngularRate = sample.angularRateBody.cast<fp64>();
    const Eigen::Vector3d previousSpecificForce = m_previousSample.specificForceBody.cast<fp64>();
    const Eigen::Vector3d currentSpecificForce = sample.specificForceBody.cast<fp64>();

    // 对每个相邻原始采样区间进行梯形积分，避免遗漏中间的1ms IMU数据
    const Eigen::Vector3d intervalAngleIncrement =
        (previousAngularRate + currentAngularRate) * (0.5 * deltaTime);
    const Eigen::Vector3d intervalVelocityIncrement =
        (previousSpecificForce + currentSpecificForce) * (0.5 * deltaTime);

    if (!intervalAngleIncrement.allFinite() || !intervalVelocityIncrement.allFinite()) {
        restartFromSample(sample);
        return IMUPreintegrationResult::CalculationFailed;
    }

    if (m_accumulatedIncrement.intervalCount == 0U) {
        m_accumulatedIncrement.startTimestampCycles = m_previousSample.timestampCycles;
        m_accumulatedIncrement.startSequence = m_previousSample.sequence;
    }

    m_accumulatedIncrement.dt += deltaTime;
    m_accumulatedIncrement.dtheta += intervalAngleIncrement;
    m_accumulatedIncrement.dvelocity += intervalVelocityIncrement;
    ++m_accumulatedIncrement.intervalCount;

    m_accumulatedIncrement.endTimestampCycles = sample.timestampCycles;
    m_accumulatedIncrement.endSequence = sample.sequence;
    m_previousSample = sample;

    if (!std::isfinite(m_accumulatedIncrement.dt) ||
        !m_accumulatedIncrement.dtheta.allFinite() ||
        !m_accumulatedIncrement.dvelocity.allFinite()) {
        restartFromSample(sample);
        return IMUPreintegrationResult::CalculationFailed;
    }

    if (m_accumulatedIncrement.intervalCount < m_config.intervalsPerIncrement) {
        return IMUPreintegrationResult::Accumulating;
    }

    if (m_accumulatedIncrement.intervalCount != m_config.intervalsPerIncrement) {
        restartFromSample(sample);
        return IMUPreintegrationResult::CalculationFailed;
    }

    increment = m_accumulatedIncrement;
    clearAccumulatedIncrement();

    return IMUPreintegrationResult::IncrementReady;
}

void IMUPreintegrator::reset() {
    m_previousSample = IMU::IMUSample{};
    m_hasPreviousSample = false;
    clearAccumulatedIncrement();
}

bool IMUPreintegrator::isSampleValid(const IMU::IMUSample &sample) const {
    if (!sample.angularRateBody.allFinite()) return false;
    if (!sample.specificForceBody.allFinite()) return false;

    return true;
}

bool IMUPreintegrator::isSequenceContinuous(const IMU::IMUSample &sample) const {
    if (!m_config.validateSequence) return true;

    const uint32_t expectedSequence = m_previousSample.sequence + 1U;
    return sample.sequence == expectedSequence;
}

void IMUPreintegrator::clearAccumulatedIncrement() {
    m_accumulatedIncrement = IMUIncrement{};
}

void IMUPreintegrator::restartFromSample(const IMU::IMUSample &sample) {
    clearAccumulatedIncrement();
    m_previousSample = sample;
    m_hasPreviousSample = true;
}

}
