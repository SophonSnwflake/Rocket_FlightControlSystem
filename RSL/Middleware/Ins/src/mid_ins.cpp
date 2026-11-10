#include "mid_ins.hpp"
#include "mid_ins_para.hpp"
#include "dvc_gnss.hpp"
#include "drv_time.h"
#include "earth.h"
#include "rotation.h"

#include <cmath>

static uint32_t howManyFuckingPositionUpdateFailed = 0;
static uint32_t howManyFuckingVelocityUpdateFailed = 0;
static uint32_t howManyFuckingAttitudeUpdateFailed = 0;
static uint32_t howManyFuckingPredictUpdateFailed = 0;

namespace INS{

ESKF_INS::ESKF_INS(INSMechanization *mech, INSESKF *eskf, GNSSMeasurementModel *gnssModel) :
    m_mech(mech),
    m_eskf(eskf),
    m_gnssModel(gnssModel)
{}

INS_Error ESKF_INS::init() {
    static constexpr Eigen::Index POSITION_ID = 0;
    static constexpr Eigen::Index VELOCITY_ID = 3;
    static constexpr Eigen::Index ATTITUDE_ID = 6;
    static constexpr Eigen::Index GYRO_BIAS_ID = 9;
    static constexpr Eigen::Index ACCEL_BIAS_ID = 12;
    static constexpr Eigen::Index GYRO_SCALE_ID = 15;
    static constexpr Eigen::Index ACCEL_SCALE_ID = 18;
    static constexpr fp64 DEG_TO_RAD = 0.01745329251994329577;

    isINSInitialized = false;

    if (m_mech == nullptr) return INS_Error::MechanizationNotAvailable;
    if (m_eskf == nullptr) return INS_Error::ESKFNotAvailable;
    if (m_gnssModel == nullptr) return INS_Error::GNSSModelNotAvailable;

    // 测试用初始标准差，位置和速度单位分别为m和m/s，姿态单位为rad
    const Eigen::Vector3d positionStd(0.1, 0.1, 0.2);
    const Eigen::Vector3d velocityStd(0.05, 0.05, 0.05);
    const Eigen::Vector3d attitudeStd(
        0.5 * DEG_TO_RAD,
        0.5 * DEG_TO_RAD,
        1.0 * DEG_TO_RAD);
    const Eigen::Vector3d gyroBiasStd =
        Eigen::Vector3d::Constant(50.0 * DEG_TO_RAD / 3600.0);
    const Eigen::Vector3d accelBiasStd = Eigen::Vector3d::Constant(250.0e-5);
    const Eigen::Vector3d gyroScaleStd = Eigen::Vector3d::Constant(1000.0e-6);
    const Eigen::Vector3d accelScaleStd = Eigen::Vector3d::Constant(1000.0e-6);

    ESKFStateMatrix initialCovariance = ESKFStateMatrix::Zero();
    initialCovariance.diagonal().segment<3>(POSITION_ID) = positionStd.array().square().matrix();
    initialCovariance.diagonal().segment<3>(VELOCITY_ID) = velocityStd.array().square().matrix();
    initialCovariance.diagonal().segment<3>(ATTITUDE_ID) = attitudeStd.array().square().matrix();
    initialCovariance.diagonal().segment<3>(GYRO_BIAS_ID) = gyroBiasStd.array().square().matrix();
    initialCovariance.diagonal().segment<3>(ACCEL_BIAS_ID) = accelBiasStd.array().square().matrix();
    initialCovariance.diagonal().segment<3>(GYRO_SCALE_ID) = gyroScaleStd.array().square().matrix();
    initialCovariance.diagonal().segment<3>(ACCEL_SCALE_ID) = accelScaleStd.array().square().matrix();

    ESKFNoiseParameters noiseParameters{};
    noiseParameters.gyroARW = Eigen::Vector3d::Constant(0.24 * DEG_TO_RAD / 60.0);
    noiseParameters.accelVRW = Eigen::Vector3d::Constant(0.24 / 60.0);
    noiseParameters.gyroBiasStd = gyroBiasStd;
    noiseParameters.accelBiasStd = accelBiasStd;
    noiseParameters.gyroScaleStd = gyroScaleStd;
    noiseParameters.accelScaleStd = accelScaleStd;
    noiseParameters.correlationTime = 3600.0;

    // 测试阶段暂不补偿杆臂，实飞前应替换为GNSS天线相对IMU的实测坐标
    const Eigen::Vector3f antennaLeverArmBody = Eigen::Vector3f::Zero();

    if (!m_eskf->init(initialCovariance, noiseParameters)) {
        return INS_Error::ESKFInitializationFailed;
    }
    if (!m_gnssModel->init(antennaLeverArmBody)) {
        return INS_Error::GNSSModelInitializationFailed;
    }

    m_currentNavState.position[0] = LAUNCH_PLACE_LATITUDE_RAD;
    m_currentNavState.position[1] = LAUNCH_PLACE_LONGITUDE_RAD;
    m_currentNavState.position[2] = LAUNCH_PLACE_HEIGHT_M;
    m_currentNavState.velocity.setZero();

    // 仅当机体系确实与NED重合时才正确
    m_currentNavState.attitude = Eigen::Quaterniond::Identity();

    m_previousNavState = m_currentNavState;

    m_currentIMUIncrement = IMUIncrement{};
    isCurrentIMUIncrementValid = false;

    m_lastGNSSSequence = 0U;

    isINSInitialized = true;
    return INS_Error::OK;
}

INS_Error ESKF_INS::update(const IMUIncrement &imuIncrement, GNSS::GNSSSample *gnssSample){
    if(!isINSInited())return INS_Error::INSNotInited;
    if (m_eskf == nullptr) {
        return INS_Error::ESKFNotAvailable;
    }
    if(!m_eskf->isInitialized()) {
        return INS_Error::ESKFNotInitialized;
    }

    if (!std::isfinite(imuIncrement.dt)) {
        isCurrentIMUIncrementValid = false;
        return INS_Error::DeltaTimeTooSmall;
    }
    if (imuIncrement.dt <= 0.0) {
        isCurrentIMUIncrementValid = false;
        return INS_Error::DeltaTimeTooSmall;
    }
    if (imuIncrement.dt > MAX_IMU_DELTA_TIME_S) {
        isCurrentIMUIncrementValid = false;
        return INS_Error::DeltaTimeTooLarge;
    }
    if (!imuIncrement.dtheta.allFinite()) {
        isCurrentIMUIncrementValid = false;
        return INS_Error::IMUNotInited;
    }
    if (!imuIncrement.dvelocity.allFinite()) {
        isCurrentIMUIncrementValid = false;
        return INS_Error::IMUNotInited;
    }

    IMUIncrement nextIMUIncrement = imuIncrement;
    NavState nextNavState = m_currentNavState;

    // 对IMU补偿零偏和比例因子
    if(IS_USE_IMU_OFFSET_COMPENSATION){
        nextIMUIncrement.dtheta -= nextNavState.imuError.gyroBias_error * nextIMUIncrement.dt;
        nextIMUIncrement.dvelocity -= nextNavState.imuError.accelBias_error * nextIMUIncrement.dt;

        const Eigen::Vector3d gyroScaleFactor =
            Eigen::Vector3d::Ones() + nextNavState.imuError.gyroScale_error;
        const Eigen::Vector3d accelScaleFactor =
            Eigen::Vector3d::Ones() + nextNavState.imuError.accelScale_error;

        nextIMUIncrement.dtheta =
            nextIMUIncrement.dtheta.cwiseQuotient(gyroScaleFactor);
        nextIMUIncrement.dvelocity =
            nextIMUIncrement.dvelocity.cwiseQuotient(accelScaleFactor);
    }

    if(!isCurrentIMUIncrementValid){
        m_currentIMUIncrement = nextIMUIncrement;
        isCurrentIMUIncrementValid = true;
        return INS_Error::OK;
    }

    // 机械编排
    if(!m_mech->updateVelocity(m_currentNavState, nextNavState, m_currentIMUIncrement, nextIMUIncrement)){
        howManyFuckingVelocityUpdateFailed++;
        return INS_Error::UpdateVelocityFailed;
    }
    if(!m_mech->updatePosition(m_currentNavState, nextNavState, nextIMUIncrement)){
        howManyFuckingPositionUpdateFailed++;
        return INS_Error::UpdatePositionFailed;
    }
    if(!m_mech->updateAttitude(m_currentNavState, nextNavState, m_currentIMUIncrement, nextIMUIncrement)){
        howManyFuckingAttitudeUpdateFailed++;
        return INS_Error::UpdateAttitudeFailed;
    }

    // ESKF Prediction步
    if(IS_USE_ESKF_PREDICTION){
        if (!m_eskf->predict(m_currentNavState, nextIMUIncrement)) {
            howManyFuckingPredictUpdateFailed++;
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

    m_currentIMUIncrement = nextIMUIncrement;
    m_previousNavState = m_currentNavState;
    m_currentNavState = nextNavState;

    return INS_Error::OK;
}

const NavState &ESKF_INS::getCurrentNavState() const {
    return m_currentNavState;
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
