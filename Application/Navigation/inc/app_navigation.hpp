/**
 ******************************************************************************
 * @file           : app_navigation.hpp
 * @brief          : 导航应用层接口
 * @brief          : Navigation application interface
 ******************************************************************************
 * @attention
 * Copyright (c) Jihao Wang XJTLU.
 * All rights reserved.
 *
 ******************************************************************************
 */

#pragma once

#include <cstdint>

#include "FreeRTOS.h"
#include "mid_ins.hpp"
#include "queue.h"

namespace Application {

class Navigation final {
public:
    enum class State : uint8_t {
        Uninitialized = 0,
        WaitingForInitialAttitude,
        Running,
        Error
    };

    enum class NavigationError : uint8_t {
        OK = 0,
        AlreadyInitialized,
        INSNotAvailable,
        INSInitializationFailed,
        QueueCreationFailed
    };

private:
    static constexpr UBaseType_t IMU_INCREMENT_QUEUE_LENGTH = 4U;
    static constexpr UBaseType_t GNSS_SAMPLE_QUEUE_LENGTH = 1U;

    INS::ESKF_INS *m_ins;

    StaticQueue_t m_imuIncrementQueueControlBlock{};
    uint8_t m_imuIncrementQueueStorage[IMU_INCREMENT_QUEUE_LENGTH * sizeof(IMUIncrement)]{};
    QueueHandle_t m_imuIncrementQueue = nullptr;

    StaticQueue_t m_gnssSampleQueueControlBlock{};
    uint8_t m_gnssSampleQueueStorage[GNSS_SAMPLE_QUEUE_LENGTH * sizeof(GNSS::GNSSSample)]{};
    QueueHandle_t m_gnssSampleQueue = nullptr;

    GNSS::GNSSSample m_pendingGNSSSample{};
    Eigen::Vector3d m_latestSpecificForceBody = Eigen::Vector3d::Zero();

    State m_state = State::Uninitialized;
    bool m_isInitialized = false;
    bool m_hasReferenceIMUIncrement = false;
    bool m_hasPendingGNSSSample = false;
    bool m_hasLatestSpecificForceBody = false;

public:
    explicit Navigation(INS::ESKF_INS *ins);
    ~Navigation() = default;

    NavigationError init();
    void reset();

    bool publishIMUIncrement(const IMUIncrement &imuIncrement);
    bool publishGNSSSample(const GNSS::GNSSSample &gnssSample);
    INS::INS_Error update();

    bool isInitialized() const;
    State getState() const { return m_state; }
    const NavState *getCurrentNavState() const;
    const Eigen::Vector3d *getLatestSpecificForceBody() const;
};

}
