#include "app_navigation.hpp"

namespace Application {

Navigation::Navigation(INS::ESKF_INS *ins) :
    m_ins(ins)
{
    m_imuIncrementQueue = xQueueCreateStatic(
        IMU_INCREMENT_QUEUE_LENGTH,
        sizeof(IMUIncrement),
        m_imuIncrementQueueStorage,
        &m_imuIncrementQueueControlBlock
    );

    m_gnssSampleQueue = xQueueCreateStatic(
        GNSS_SAMPLE_QUEUE_LENGTH,
        sizeof(GNSS::GNSSSample),
        m_gnssSampleQueueStorage,
        &m_gnssSampleQueueControlBlock
    );
}

Navigation::NavigationError Navigation::init() {
    if (m_isInitialized) return NavigationError::AlreadyInitialized;
    if (m_ins == nullptr) return NavigationError::INSNotAvailable;
    if (m_imuIncrementQueue == nullptr) return NavigationError::QueueCreationFailed;
    if (m_gnssSampleQueue == nullptr) return NavigationError::QueueCreationFailed;

    const INS::INS_Error result = m_ins->init();
    if (result != INS::INS_Error::OK) {
        m_state = State::Error;
        return NavigationError::INSInitializationFailed;
    }

    xQueueReset(m_imuIncrementQueue);
    xQueueReset(m_gnssSampleQueue);
    m_pendingGNSSSample = GNSS::GNSSSample{};
    m_latestSpecificForceBody.setZero();
    m_hasReferenceIMUIncrement = false;
    m_hasPendingGNSSSample = false;
    m_hasLatestSpecificForceBody = false;

    m_state = State::Running;
    m_isInitialized = true;

    return NavigationError::OK;
}

void Navigation::reset() {
    if (m_imuIncrementQueue != nullptr) {
        xQueueReset(m_imuIncrementQueue);
    }
    if (m_gnssSampleQueue != nullptr) {
        xQueueReset(m_gnssSampleQueue);
    }

    m_pendingGNSSSample = GNSS::GNSSSample{};
    m_latestSpecificForceBody.setZero();
    m_hasReferenceIMUIncrement = false;
    m_hasPendingGNSSSample = false;
    m_hasLatestSpecificForceBody = false;
    m_state = State::Uninitialized;
    m_isInitialized = false;
}

bool Navigation::publishIMUIncrement(const IMUIncrement &imuIncrement) {
    if (!isInitialized()) return false;
    if (m_imuIncrementQueue == nullptr) return false;

    return xQueueSend(m_imuIncrementQueue, &imuIncrement, 0U) == pdPASS;
}

bool Navigation::publishGNSSSample(const GNSS::GNSSSample &gnssSample) {
    if (!isInitialized()) return false;
    if (m_gnssSampleQueue == nullptr) return false;

    return xQueueOverwrite(m_gnssSampleQueue, &gnssSample) == pdPASS;
}

INS::INS_Error Navigation::update() {
    if (m_ins == nullptr) return INS::INS_Error::ESKFNotAvailable;
    if (!isInitialized()) return INS::INS_Error::INSNotInited;
    if (m_imuIncrementQueue == nullptr) return INS::INS_Error::INSNotInited;
    if (m_gnssSampleQueue == nullptr) return INS::INS_Error::INSNotInited;

    GNSS::GNSSSample latestGNSSSample{};
    if (xQueueReceive(m_gnssSampleQueue, &latestGNSSSample, 0U) == pdPASS) {
        m_pendingGNSSSample = latestGNSSSample;
        m_hasPendingGNSSSample = true;
    }

    IMUIncrement imuIncrement{};
    while (xQueueReceive(m_imuIncrementQueue, &imuIncrement, 0U) == pdPASS) {
        GNSS::GNSSSample *gnssSample = nullptr;
        if (m_hasReferenceIMUIncrement && m_hasPendingGNSSSample) {
            const int32_t imuTimeAfterGNSS = static_cast<int32_t>(
                imuIncrement.endTimestampCycles - m_pendingGNSSSample.timestampCycles
            );
            if (m_pendingGNSSSample.timestampCycles == 0U ||
                imuTimeAfterGNSS >= 0) {
                gnssSample = &m_pendingGNSSSample;
            }
        }

        const INS::INS_Error result = m_ins->update(imuIncrement, gnssSample);
        if (result != INS::INS_Error::OK) {
            if (result == INS::INS_Error::DeltaTimeTooSmall ||
                result == INS::INS_Error::DeltaTimeTooLarge ||
                result == INS::INS_Error::IMUNotInited) {
                m_hasReferenceIMUIncrement = false;
            }
            return result;
        }

        if (!m_hasReferenceIMUIncrement) {
            m_hasReferenceIMUIncrement = true;
        } else if (gnssSample != nullptr) {
            m_hasPendingGNSSSample = false;
        }

        if (imuIncrement.dt > 0.0 &&
            imuIncrement.dvelocity.allFinite()) {
            const Eigen::Vector3d specificForceBody =
                imuIncrement.dvelocity / imuIncrement.dt;
            if (specificForceBody.allFinite()) {
                m_latestSpecificForceBody = specificForceBody;
                m_hasLatestSpecificForceBody = true;
            }
        }
    }

    return INS::INS_Error::OK;
}

bool Navigation::isInitialized() const {
    if (!m_isInitialized) return false;
    if (m_ins == nullptr) return false;

    return m_ins->isINSInited();
}

const NavState *Navigation::getCurrentNavState() const {
    if (!isInitialized()) return nullptr;

    return &m_ins->getCurrentNavState();
}

const Eigen::Vector3d *Navigation::getLatestSpecificForceBody() const {
    if (!m_hasLatestSpecificForceBody) return nullptr;

    return &m_latestSpecificForceBody;
}

}
