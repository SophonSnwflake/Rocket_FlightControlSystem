#include "mid_ins.hpp"
#include "mid_ins_para.hpp"
#include "dvc_imu.hpp"
#include "dvc_gnss.hpp"
#include "drv_time.h"
#include "earth.h"
#include "rotation.h"

#include <cmath>

namespace INS{

ESKF_INS::ESKF_INS(INSMechanization *mech, INSESKF *eskf, GNSSMeasurementModel *gnssModel) :
    m_mech(mech),
    m_eskf(eskf),
    m_gnssModel(gnssModel)
{}

INS_Error ESKF_INS::init() {
    isINSInitialized = false;

    if (m_mech == nullptr) return INS_Error::MechanizationNotAvailable;
    if (m_eskf == nullptr) return INS_Error::ESKFNotAvailable;
    if (!m_eskf->isInitialized()) return INS_Error::ESKFNotInitialized;

    m_currentNavState.position[0] = LAUNCH_PLACE_LATITUDE_RAD;
    m_currentNavState.position[1] = LAUNCH_PLACE_LONGITUDE_RAD;
    m_currentNavState.position[2] = LAUNCH_PLACE_HEIGHT_M;
    m_currentNavState.velocity.setZero();

    // 仅当机体系确实与NED重合时才正确
    m_currentNavState.attitude = Eigen::Quaterniond::Identity();

    m_previousNavState = m_currentNavState;

    m_previousIMUData = IMUData{};
    m_currentIMUData = IMUData{};
    isCurrentIMUDataValid = false;

    m_lastGNSSSequence = 0U;

    isINSInitialized = true;
    return INS_Error::OK;
}

INS_Error ESKF_INS::update(IMU::IMUSample &previousSample, IMU::IMUSample &currentSample, GNSS::GNSSSample *gnssSample){
    if(!isINSInited())return INS_Error::INSNotInited;
    if (m_eskf == nullptr) {
        return INS_Error::ESKFNotAvailable;
    }
    if(!m_eskf->isInitialized()) {
        return INS_Error::ESKFNotInitialized;
    }

    if(previousSample.timestampCycles <= 0) return INS_Error::IMUNotInited;
    if(!isCurrentIMUDataValid){
        INS_TRY(packetIMUData(previousSample, currentSample, m_currentIMUData));
        isCurrentIMUDataValid = true;
        return INS_Error::OK;
    }

    IMUData nextIMUData{};
    NavState nextNavState = m_currentNavState;

    INS_TRY(packetIMUData(previousSample, currentSample, nextIMUData));

    // 对IMU补偿零偏和比例因子
    if(IS_USE_IMU_OFFSET_COMPENSATION){
        nextIMUData.dtheta -= nextNavState.imuError.gyroBias_error * nextIMUData.dt;
        nextIMUData.dvelocity -= nextNavState.imuError.accelBias_error* nextIMUData.dt;

        const Eigen::Vector3d gyroScaleFactor =
            Eigen::Vector3d::Ones() + nextNavState.imuError.gyroScale_error;
        const Eigen::Vector3d accelScaleFactor =
            Eigen::Vector3d::Ones() + nextNavState.imuError.accelScale_error;

        nextIMUData.dtheta =
            nextIMUData.dtheta.cwiseQuotient(gyroScaleFactor);
        nextIMUData.dvelocity =
            nextIMUData.dvelocity.cwiseQuotient(accelScaleFactor);
    }

    // 机械编排
    if(!m_mech->updateVelocity(m_currentNavState, nextNavState, m_currentIMUData, nextIMUData)){
        return INS_Error::UpdateVelocityFailed;
    }
    if(!m_mech->updatePosition(m_currentNavState, nextNavState, nextIMUData)){
        return INS_Error::UpdatePositionFailed;
    }
    if(!m_mech->updateAttitude(m_currentNavState, nextNavState, m_currentIMUData, nextIMUData)){
        return INS_Error::UpdateAttitudeFailed;
    }

    // ESKF Prediction步
    if(IS_USE_ESKF_PREDICTION){
        if (!m_eskf->predict(m_currentNavState, nextIMUData)) {
            return INS_Error::ESKFPredictFailed;
        }
    }

    // ESKF Correction步
    if (isGNSSSampleAvailableForCorrection(gnssSample)) {
        m_lastGNSSSequence = gnssSample->sequence;

        if (m_gnssModel != nullptr) {
            GNSSMeasurement gnssMeasurement{};
            gnssMeasurement.timestamp =
                static_cast<fp64>(gnssSample->timeOfWeekMs) * 0.001;
            gnssMeasurement.positionBLH = gnssSample->positionBLH;
            gnssMeasurement.positionStdNED = gnssSample->positionStdNED.cast<fp64>();
            gnssMeasurement.isTimestampValid = true;
            gnssMeasurement.isPositionValid = true;
            gnssMeasurement.isAccuracyValid = true;

            LinearizedMeasurement measurement{};
            if (m_gnssModel->build(nextNavState, gnssMeasurement, measurement)) {
                if (m_eskf->correct(measurement)) {
                    INS_TRY(errorFeedback(nextNavState));
                }
            }
        }
    }

    m_previousIMUData = m_currentIMUData;
    m_currentIMUData = nextIMUData;
    m_previousNavState = m_currentNavState;
    m_currentNavState = nextNavState;

    return INS_Error::OK;
}

bool ESKF_INS::isGNSSSampleAvailableForCorrection(const GNSS::GNSSSample *gnssSample) const {
    if (!IS_USE_ESKF_CORRECTION) return false;
    if (gnssSample == nullptr) return false;
    if (gnssSample->sequence == 0U) return false;
    if (gnssSample->sequence == m_lastGNSSSequence) return false;
    if (!DWT_IsInit()) return false;
    if (gnssSample->timestampCycles == 0U) return false;
    if (SystemCoreClock == 0U) return false;

    const bool has3DPosition =
        gnssSample->fixType == GNSS::NavigationFixType::Fix3D ||
        gnssSample->fixType == GNSS::NavigationFixType::GNSSAndDeadReckoning;
    if (!has3DPosition) return false;

    if (!gnssSample->isTimestampValid) return false;
    if (!gnssSample->isPositionValid) return false;
    if (!gnssSample->isAccuracyValid) return false;
    if (!gnssSample->positionBLH.allFinite()) return false;
    if (!gnssSample->positionStdNED.allFinite()) return false;
    if (!(gnssSample->positionStdNED.array() > 0.0f).all()) return false;

    const uint32_t currentTimestampCycles = DWT_GetTimestamp();
    const uint32_t dataAgeCycles = currentTimestampCycles - gnssSample->timestampCycles;
    const fp64 dataAgeSeconds =
        static_cast<fp64>(dataAgeCycles) / static_cast<fp64>(SystemCoreClock);

    if (dataAgeSeconds > MAX_GNSS_DATA_AGE_S) return false;

    return true;
}

INS_Error ESKF_INS::packetIMUData(const IMU::IMUSample &previousSample, const IMU::IMUSample &currentSample, IMUData &IMUData){
    const uint32_t deltaCycles = currentSample.timestampCycles - previousSample.timestampCycles;
    const fp64 deltaTime = static_cast<fp64>(deltaCycles) / static_cast<fp64>(SystemCoreClock); // 转换为秒
    if(deltaTime <= 0) return INS_Error::DeltaTimeTooSmall;
    if(deltaTime > MAX_IMU_DELTA_TIME_S) return INS_Error::DeltaTimeTooLarge;
    IMUData.dt = deltaTime;
    IMUData.dtheta = (previousSample.angularRateBody.cast<fp64>() + currentSample.angularRateBody.cast<fp64>()) * 0.5 * deltaTime;
    IMUData.dvelocity = (previousSample.specificForceBody.cast<fp64>() + currentSample.specificForceBody.cast<fp64>()) * 0.5 * deltaTime;
    return INS_Error::OK;
}

INS_Error ESKF_INS::errorFeedback(NavState &nextNavState){
    static constexpr fp64 MIN_QUATERNION_NORM = 1e-8;
    static constexpr fp64 MIN_SCALE_FACTOR = 1e-8;
    static constexpr fp64 HALF_PI = 1.57079632679489661923;
    static constexpr fp64 PI = 3.14159265358979323846;

    if (m_eskf == nullptr) return INS_Error::ESKFNotAvailable;
    if (!m_eskf->isInitialized()) return INS_Error::ESKFNotInitialized;
    if (!nextNavState.position.allFinite()) return INS_Error::ESKFFeedbackFailed;
    if (std::abs(nextNavState.position[0]) >= HALF_PI) return INS_Error::ESKFFeedbackFailed;
    if (std::abs(nextNavState.position[1]) > PI) return INS_Error::ESKFFeedbackFailed;
    if (!nextNavState.velocity.allFinite()) return INS_Error::ESKFFeedbackFailed;
    if (!nextNavState.attitude.coeffs().allFinite()) return INS_Error::ESKFFeedbackFailed;
    if (!nextNavState.imuError.gyroBias_error.allFinite()) return INS_Error::ESKFFeedbackFailed;
    if (!nextNavState.imuError.accelBias_error.allFinite()) return INS_Error::ESKFFeedbackFailed;
    if (!nextNavState.imuError.gyroScale_error.allFinite()) return INS_Error::ESKFFeedbackFailed;
    if (!nextNavState.imuError.accelScale_error.allFinite()) return INS_Error::ESKFFeedbackFailed;

    NavState temNavState = nextNavState;

    {// 作用域，用于释放栈
        // 获取误差状态
        const Eigen::Vector3d positionError = m_eskf->getPositionErrorState();
        const Eigen::Vector3d velocityError = m_eskf->getVelocityErrorState();
        const Eigen::Vector3d attitudeError = m_eskf->getAttitudeErrorState();
        if (!positionError.allFinite()) return INS_Error::ESKFFeedbackFailed;
        if (!velocityError.allFinite()) return INS_Error::ESKFFeedbackFailed;
        if (!attitudeError.allFinite()) return INS_Error::ESKFFeedbackFailed;

        // 位速姿补偿
        const Eigen::Matrix3d positionConversion = Earth::DRi(temNavState.position);
        if (!positionConversion.allFinite()) return INS_Error::ESKFFeedbackFailed;
        temNavState.position -= positionConversion * positionError;
        temNavState.velocity -= velocityError;
        const Eigen::Quaterniond correctionQuaternion = Rotation::rotvec2quaternion(attitudeError);
        if (!correctionQuaternion.coeffs().allFinite()) return INS_Error::ESKFFeedbackFailed;
        temNavState.attitude = correctionQuaternion * temNavState.attitude;
        const fp64 quaternionNorm = temNavState.attitude.norm();
        if (!std::isfinite(quaternionNorm)) return INS_Error::ESKFFeedbackFailed;
        if (quaternionNorm < MIN_QUATERNION_NORM) return INS_Error::ESKFFeedbackFailed;
        temNavState.attitude.normalize();
    } // 作用域，用于释放栈

    { // 作用域，用于释放栈
        const Eigen::Vector3d gyroBiasError = m_eskf->getGyroBiasErrorState();
        const Eigen::Vector3d accelBiasError = m_eskf->getAccelBiasErrorState();
        if (!gyroBiasError.allFinite()) return INS_Error::ESKFFeedbackFailed;
        if (!accelBiasError.allFinite()) return INS_Error::ESKFFeedbackFailed;

        temNavState.imuError.gyroBias_error += gyroBiasError;
        temNavState.imuError.accelBias_error += accelBiasError;
    } // 作用域，用于释放栈

    { // 作用域，用于释放栈
        const Eigen::Vector3d gyroScaleError = m_eskf->getGyroScaleErrorState();
        const Eigen::Vector3d accelScaleError = m_eskf->getAccelScaleErrorState();
        if (!gyroScaleError.allFinite()) return INS_Error::ESKFFeedbackFailed;
        if (!accelScaleError.allFinite()) return INS_Error::ESKFFeedbackFailed;

        temNavState.imuError.gyroScale_error += gyroScaleError;
        temNavState.imuError.accelScale_error += accelScaleError;
    } // 作用域，用于释放栈

    if (!temNavState.position.allFinite()) return INS_Error::ESKFFeedbackFailed;
    if (std::abs(temNavState.position[0]) >= HALF_PI) return INS_Error::ESKFFeedbackFailed;
    if (std::abs(temNavState.position[1]) > PI) return INS_Error::ESKFFeedbackFailed;
    if (!temNavState.velocity.allFinite()) return INS_Error::ESKFFeedbackFailed;
    if (!temNavState.attitude.coeffs().allFinite()) return INS_Error::ESKFFeedbackFailed;
    if (!temNavState.imuError.gyroBias_error.allFinite()) return INS_Error::ESKFFeedbackFailed;
    if (!temNavState.imuError.accelBias_error.allFinite()) return INS_Error::ESKFFeedbackFailed;
    if (!temNavState.imuError.gyroScale_error.allFinite()) return INS_Error::ESKFFeedbackFailed;
    if (!temNavState.imuError.accelScale_error.allFinite()) return INS_Error::ESKFFeedbackFailed;

    const Eigen::Vector3d gyroScaleFactor =
        Eigen::Vector3d::Ones() + temNavState.imuError.gyroScale_error;
    const Eigen::Vector3d accelScaleFactor =
        Eigen::Vector3d::Ones() + temNavState.imuError.accelScale_error;
    if ((gyroScaleFactor.array() <= MIN_SCALE_FACTOR).any()) return INS_Error::ESKFFeedbackFailed;
    if ((accelScaleFactor.array().abs() < MIN_SCALE_FACTOR).any()) return INS_Error::ESKFFeedbackFailed;

    // 闭环ESKF别忘了奥
    if(!m_eskf->resetErrorState()){
        return INS_Error::ESKFResetFailed;
    }

    nextNavState = temNavState;

    return INS_Error::OK;
}
}
