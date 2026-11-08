#include "alg_ins_eskf.hpp"

#include <Eigen/Cholesky>

#include <cmath>

#include "earth.h"
#include "rotation.h"

namespace INS {

bool INSESKF::init(const ESKFStateMatrix &initialCovariance, const ESKFNoiseParameters &noiseParameters){
    if (!initialCovariance.allFinite()) return false;
    if (!initialCovariance.isApprox(initialCovariance.transpose())) return false;
    if ((initialCovariance.diagonal().array() < 0.0).any()) return false;
    if (!noiseParameters.gyroARW.allFinite()) return false;
    if (!noiseParameters.accelVRW.allFinite()) return false;
    if (!noiseParameters.gyroBiasStd.allFinite()) return false;
    if (!noiseParameters.accelBiasStd.allFinite()) return false;
    if (!noiseParameters.gyroScaleStd.allFinite()) return false;
    if (!noiseParameters.accelScaleStd.allFinite()) return false;
    if ((noiseParameters.gyroARW.array() < 0.0).any()) return false;
    if ((noiseParameters.accelVRW.array() < 0.0).any()) return false;
    if ((noiseParameters.gyroBiasStd.array() < 0.0).any()) return false;
    if ((noiseParameters.accelBiasStd.array() < 0.0).any()) return false;
    if ((noiseParameters.gyroScaleStd.array() < 0.0).any()) return false;
    if ((noiseParameters.accelScaleStd.array() < 0.0).any()) return false;
    if (!std::isfinite(noiseParameters.correlationTime)) return false;
    if (noiseParameters.correlationTime <= 0.0) return false;

    static constexpr Eigen::Index VRW_NOISE_ID = 0;
    static constexpr Eigen::Index ARW_NOISE_ID = 3;
    static constexpr Eigen::Index GYRO_BIAS_NOISE_ID = 6;
    static constexpr Eigen::Index ACCEL_BIAS_NOISE_ID = 9;
    static constexpr Eigen::Index GYRO_SCALE_NOISE_ID = 12;
    static constexpr Eigen::Index ACCEL_SCALE_NOISE_ID = 15;

    ESKFNoiseMatrix continuousNoiseMatrix = ESKFNoiseMatrix::Zero();
    const fp64 markovNoiseFactor = 2.0 / noiseParameters.correlationTime;

    for (Eigen::Index axis = 0; axis < 3; ++axis) {
        continuousNoiseMatrix(VRW_NOISE_ID + axis, VRW_NOISE_ID + axis) =
            noiseParameters.accelVRW[axis] * noiseParameters.accelVRW[axis];
        continuousNoiseMatrix(ARW_NOISE_ID + axis, ARW_NOISE_ID + axis) =
            noiseParameters.gyroARW[axis] * noiseParameters.gyroARW[axis];
        continuousNoiseMatrix(GYRO_BIAS_NOISE_ID + axis, GYRO_BIAS_NOISE_ID + axis) =
            markovNoiseFactor * noiseParameters.gyroBiasStd[axis] * noiseParameters.gyroBiasStd[axis];
        continuousNoiseMatrix(ACCEL_BIAS_NOISE_ID + axis, ACCEL_BIAS_NOISE_ID + axis) =
            markovNoiseFactor * noiseParameters.accelBiasStd[axis] * noiseParameters.accelBiasStd[axis];
        continuousNoiseMatrix(GYRO_SCALE_NOISE_ID + axis, GYRO_SCALE_NOISE_ID + axis) =
            markovNoiseFactor * noiseParameters.gyroScaleStd[axis] * noiseParameters.gyroScaleStd[axis];
        continuousNoiseMatrix(ACCEL_SCALE_NOISE_ID + axis, ACCEL_SCALE_NOISE_ID + axis) =
            markovNoiseFactor * noiseParameters.accelScaleStd[axis] * noiseParameters.accelScaleStd[axis];
    }

    if (!continuousNoiseMatrix.allFinite()) return false;

    m_errorState.setZero();
    m_P = initialCovariance;
    m_Q.setZero();
    m_Phi.setIdentity();
    m_Qc = continuousNoiseMatrix;
    m_noiseParameters = noiseParameters;
    m_isInitialized = true;

    return true;
}


bool INSESKF::predict(const NavState &navState, const IMUData &imuData) {
    if (!m_isInitialized) return false;

    INS_TRY(structPhiMatrix(navState, imuData, m_Phi));
    INS_TRY(structQMatrix(navState, imuData, m_Phi, m_Q));

    m_predictedErrorState = m_Phi * m_errorState;

    m_stateMatrixWorkspace.noalias() = m_Phi * m_P;
    m_predictedCovariance.noalias() =
        m_stateMatrixWorkspace * m_Phi.transpose();
    m_predictedCovariance += m_Q;

    if (!m_predictedErrorState.allFinite()) return false;
    if (!m_predictedCovariance.allFinite()) return false;

    m_predictedCovariance = (m_predictedCovariance + m_predictedCovariance.transpose()) * 0.5;

    m_errorState = m_predictedErrorState;
    m_P = m_predictedCovariance;

    return true;
}

bool INSESKF::correct(const LinearizedMeasurement &measurement) {
    if (!m_isInitialized) return false;
    if (!measurement.innovation.allFinite()) return false;
    if (!measurement.H.allFinite()) return false;
    if (!measurement.R.allFinite()) return false;
    if (!m_errorState.allFinite()) return false;
    if (!m_P.allFinite()) return false;
    if (!m_P.isApprox(m_P.transpose())) return false;
    if (!measurement.R.isApprox(measurement.R.transpose())) return false;

    // 检查观测噪声协方差矩阵是否为正定矩阵
    {
        Eigen::LDLT<ESKFMeasurementNoiseMatrix> measurementNoiseDecomposition(measurement.R);
        if (measurementNoiseDecomposition.info() != Eigen::Success) return false;
        if (!measurementNoiseDecomposition.vectorD().allFinite()) return false;
        if (!measurementNoiseDecomposition.isPositive()) return false;
        if ((measurementNoiseDecomposition.vectorD().array() <= 0.0).any()) return false;
    }

    // 计算P乘以H的转置，暂存在噪声驱动矩阵工作区的前三列
    m_noiseDriveMatrix.leftCols<ESKF_MEASUREMENT_DIMENSION>().noalias() =
        m_P * measurement.H.transpose();

    // 构建新息协方差矩阵S = HPH' + R
    m_matrix3Workspace.noalias() =
        measurement.H * m_noiseDriveMatrix.leftCols<ESKF_MEASUREMENT_DIMENSION>();
    m_matrix3Workspace += measurement.R;
    m_bodyToNavigationMatrix = m_matrix3Workspace.transpose();
    m_matrix3Workspace = (m_matrix3Workspace + m_bodyToNavigationMatrix) * 0.5;
    if (!m_matrix3Workspace.allFinite()) return false;

    Eigen::LDLT<ESKFMeasurementNoiseMatrix> innovationCovarianceDecomposition(m_matrix3Workspace);
    if (innovationCovarianceDecomposition.info() != Eigen::Success) return false;
    if (!innovationCovarianceDecomposition.vectorD().allFinite()) return false;
    if (!innovationCovarianceDecomposition.isPositive()) return false;
    if ((innovationCovarianceDecomposition.vectorD().array() <= 0.0).any()) return false;

    // 逐行求解S乘以K的转置等于H乘以P，避免显式计算S的逆矩阵
    for (Eigen::Index stateIndex = 0; stateIndex < ESKF_STATE_DIMENSION; ++stateIndex) {
        const ESKFMeasurementVector kalmanGainRow = innovationCovarianceDecomposition.solve(
            m_noiseDriveMatrix.block<1, ESKF_MEASUREMENT_DIMENSION>(stateIndex, 0).transpose());
        if (!kalmanGainRow.allFinite()) return false;

        m_noiseDriveMatrix.block<1, ESKF_MEASUREMENT_DIMENSION>(stateIndex, 0) =
            kalmanGainRow.transpose();
    }

    // 更新误差状态x = x + K(z - Hx)
    ESKFMeasurementVector innovationResidual = measurement.innovation;
    innovationResidual.noalias() -= measurement.H * m_errorState;
    if (!innovationResidual.allFinite()) return false;

    m_predictedErrorState.noalias() =
        m_noiseDriveMatrix.leftCols<ESKF_MEASUREMENT_DIMENSION>() * innovationResidual;
    m_predictedErrorState += m_errorState;
    if (!m_predictedErrorState.allFinite()) return false;

    // 使用Joseph形式更新协方差P = (I-KH)P(I-KH)' + KRK'
    m_noiseAtIntervalStart.noalias() =
        m_noiseDriveMatrix.leftCols<ESKF_MEASUREMENT_DIMENSION>() * measurement.H;
    m_stateMatrixWorkspace.setIdentity();
    m_stateMatrixWorkspace -= m_noiseAtIntervalStart;

    m_noiseAtIntervalStart.noalias() = m_stateMatrixWorkspace * m_P;
    m_predictedCovariance.noalias() =
        m_noiseAtIntervalStart * m_stateMatrixWorkspace.transpose();

    m_noiseMappingWorkspace.leftCols<ESKF_MEASUREMENT_DIMENSION>().noalias() =
        m_noiseDriveMatrix.leftCols<ESKF_MEASUREMENT_DIMENSION>() * measurement.R;
    m_noiseAtIntervalStart.noalias() =
        m_noiseMappingWorkspace.leftCols<ESKF_MEASUREMENT_DIMENSION>() *
        m_noiseDriveMatrix.leftCols<ESKF_MEASUREMENT_DIMENSION>().transpose();
    m_predictedCovariance += m_noiseAtIntervalStart;

    if (!m_predictedCovariance.allFinite()) return false;

    m_noiseAtIntervalEnd = m_predictedCovariance.transpose();
    m_predictedCovariance = (m_predictedCovariance + m_noiseAtIntervalEnd) * 0.5;

    m_errorState = m_predictedErrorState;
    m_P = m_predictedCovariance;

    return true;
}

/**
 * @brief 根据当前导航状态和IMU增量构建21维离散状态转移矩阵Phi。
 * @param navState 用于线性化误差模型的当前导航状态。
 * @param imuData 当前周期的角增量、速度增量和时间间隔。
 * @param Phi 成功时输出一阶离散化状态转移矩阵。
 * @return true 构建成功且输出矩阵有效。
 * @return false 输入无效或计算结果包含非有限值。
 */
bool INSESKF::structPhiMatrix(const NavState &navState, const IMUData &imuData, ESKFStateMatrix &Phi){
    // 状态索引与数值保护阈值
    static constexpr Eigen::Index POSITION_ID = 0;
    static constexpr Eigen::Index VELOCITY_ID = 3;
    static constexpr Eigen::Index ATTITUDE_ID = 6;
    static constexpr Eigen::Index GYRO_BIAS_ID = 9;
    static constexpr Eigen::Index ACCEL_BIAS_ID = 12;
    static constexpr Eigen::Index GYRO_SCALE_ID = 15;
    static constexpr Eigen::Index ACCEL_SCALE_ID = 18;
    static constexpr fp64 MIN_QUATERNION_NORM = 1e-8;
    static constexpr fp64 MIN_DENOMINATOR = 1e-8;
    static constexpr fp64 HALF_PI = 1.57079632679489661923;

    // 输入数据有效性检查
    if (!std::isfinite(imuData.dt)) return false;
    if (imuData.dt <= 0.0) return false;
    if (!navState.position.allFinite()) return false;
    if (!navState.velocity.allFinite()) return false;
    if (!navState.attitude.coeffs().allFinite()) return false;
    if (!imuData.dtheta.allFinite()) return false;
    if (!imuData.dvelocity.allFinite()) return false;

    // 纬度与姿态四元数有效性检查
    const fp64 latitude = navState.position[0];
    if (std::abs(latitude) >= HALF_PI) return false;

    const fp64 quaternionNorm = navState.attitude.norm();
    if (!std::isfinite(quaternionNorm) || quaternionNorm < MIN_QUATERNION_NORM) return false;

    // 计算纬度三角函数
    const fp64 sinLatitude = std::sin(latitude);
    const fp64 cosLatitude = std::cos(latitude);
    if (!std::isfinite(sinLatitude) ||
        !std::isfinite(cosLatitude) ||
        std::abs(cosLatitude) < MIN_DENOMINATOR) {
        return false;
    }

    // 计算地球曲率半径与重力参数
    const fp64 tanLatitude = sinLatitude / cosLatitude;
    const fp64 cosLatitudeSquared = cosLatitude * cosLatitude;
    const Eigen::Vector2d radii = Earth::meridianPrimeVerticalRadius(latitude);
    if (!radii.allFinite()) return false;

    const fp64 meridianRadiusHeight = radii[0] + navState.position[2];
    const fp64 primeVerticalRadiusHeight = radii[1] + navState.position[2];
    if (!std::isfinite(meridianRadiusHeight) ||
        !std::isfinite(primeVerticalRadiusHeight) ||
        std::abs(meridianRadiusHeight) < MIN_DENOMINATOR ||
        std::abs(primeVerticalRadiusHeight) < MIN_DENOMINATOR) {
        return false;
    }

    const fp64 gravity = Earth::gravity(navState.position);
    const fp64 gravityRadius = std::sqrt(radii[0] * radii[1]) + navState.position[2];
    if (!std::isfinite(gravity) ||
        !std::isfinite(gravityRadius) ||
        std::abs(gravityRadius) < MIN_DENOMINATOR) {
        return false;
    }

    // 计算姿态矩阵、比力和角速度
    m_bodyToNavigationMatrix = navState.attitude.normalized().toRotationMatrix();
    const Eigen::Vector3d acceleration = imuData.dvelocity / imuData.dt;
    const Eigen::Vector3d angularRate = imuData.dtheta / imuData.dt;
    if (!m_bodyToNavigationMatrix.allFinite() || !acceleration.allFinite() || !angularRate.allFinite()) return false;


    // 计算导航系下的地球自转角速度和运输角速度
    Eigen::Vector3d earthRateN;
    earthRateN << WGS84_WIE * cosLatitude,
                  0.0,
                  -WGS84_WIE * sinLatitude;

    Eigen::Vector3d transportRateN;
    transportRateN << navState.velocity[1] / primeVerticalRadiusHeight,
                      -navState.velocity[0] / meridianRadiusHeight,
                      -navState.velocity[1] * tanLatitude / primeVerticalRadiusHeight;

    // 初始化连续时间状态矩阵F，暂时复用Phi作为工作区
    Phi.setZero();
    m_matrix3Workspace.setZero();

    // 构建位置误差块：Fpp、Fpv
    m_matrix3Workspace(0, 0) = -navState.velocity[2] / meridianRadiusHeight;
    m_matrix3Workspace(0, 2) = navState.velocity[0] / meridianRadiusHeight;
    m_matrix3Workspace(1, 0) = navState.velocity[1] * tanLatitude / primeVerticalRadiusHeight;
    m_matrix3Workspace(1, 1) = -(navState.velocity[2] + navState.velocity[0] * tanLatitude) /
                               primeVerticalRadiusHeight;
    m_matrix3Workspace(1, 2) = navState.velocity[1] / primeVerticalRadiusHeight;
    Phi.block<3, 3>(POSITION_ID, POSITION_ID) = m_matrix3Workspace;
    Phi.block<3, 3>(POSITION_ID, VELOCITY_ID).setIdentity();

    // 构建速度对位置误差块：Fvp
    m_matrix3Workspace.setZero();
    m_matrix3Workspace(0, 0) = -2.0 * navState.velocity[1] * WGS84_WIE * cosLatitude /
                                   meridianRadiusHeight -
                               navState.velocity[1] * navState.velocity[1] /
                                   (meridianRadiusHeight * primeVerticalRadiusHeight * cosLatitudeSquared);
    m_matrix3Workspace(0, 2) = navState.velocity[0] * navState.velocity[2] /
                                   (meridianRadiusHeight * meridianRadiusHeight) -
                               navState.velocity[1] * navState.velocity[1] * tanLatitude /
                                   (primeVerticalRadiusHeight * primeVerticalRadiusHeight);
    m_matrix3Workspace(1, 0) = 2.0 * WGS84_WIE *
                                   (navState.velocity[0] * cosLatitude - navState.velocity[2] * sinLatitude) /
                                   meridianRadiusHeight +
                               navState.velocity[0] * navState.velocity[1] /
                                   (meridianRadiusHeight * primeVerticalRadiusHeight * cosLatitudeSquared);
    m_matrix3Workspace(1, 2) = (navState.velocity[1] * navState.velocity[2] +
                                navState.velocity[0] * navState.velocity[1] * tanLatitude) /
                               (primeVerticalRadiusHeight * primeVerticalRadiusHeight);
    m_matrix3Workspace(2, 0) = 2.0 * WGS84_WIE * navState.velocity[1] * sinLatitude /
                               meridianRadiusHeight;
    m_matrix3Workspace(2, 2) = -navState.velocity[1] * navState.velocity[1] /
                                   (primeVerticalRadiusHeight * primeVerticalRadiusHeight) -
                               navState.velocity[0] * navState.velocity[0] /
                                   (meridianRadiusHeight * meridianRadiusHeight) +
                               2.0 * gravity / gravityRadius;
    Phi.block<3, 3>(VELOCITY_ID, POSITION_ID) = m_matrix3Workspace;

    // 构建速度误差相关块：Fvv、Fvphi、Fvba、Fvsa
    m_matrix3Workspace.setZero();
    m_matrix3Workspace(0, 0) = navState.velocity[2] / meridianRadiusHeight;
    m_matrix3Workspace(0, 1) = -2.0 *
                               (WGS84_WIE * sinLatitude +
                                navState.velocity[1] * tanLatitude / primeVerticalRadiusHeight);
    m_matrix3Workspace(0, 2) = navState.velocity[0] / meridianRadiusHeight;
    m_matrix3Workspace(1, 0) = 2.0 * WGS84_WIE * sinLatitude +
                               navState.velocity[1] * tanLatitude / primeVerticalRadiusHeight;
    m_matrix3Workspace(1, 1) = (navState.velocity[2] + navState.velocity[0] * tanLatitude) /
                               primeVerticalRadiusHeight;
    m_matrix3Workspace(1, 2) = 2.0 * WGS84_WIE * cosLatitude +
                               navState.velocity[1] / primeVerticalRadiusHeight;
    m_matrix3Workspace(2, 0) = -2.0 * navState.velocity[0] / meridianRadiusHeight;
    m_matrix3Workspace(2, 1) = -2.0 *
                               (WGS84_WIE * cosLatitude +
                                navState.velocity[1] / primeVerticalRadiusHeight);
    Phi.block<3, 3>(VELOCITY_ID, VELOCITY_ID) = m_matrix3Workspace;
    Phi.block<3, 3>(VELOCITY_ID, ATTITUDE_ID) =
        Rotation::skewSymmetric(m_bodyToNavigationMatrix * acceleration);
    Phi.block<3, 3>(VELOCITY_ID, ACCEL_BIAS_ID) = m_bodyToNavigationMatrix;
    Phi.block<3, 3>(VELOCITY_ID, ACCEL_SCALE_ID) = m_bodyToNavigationMatrix * acceleration.asDiagonal();

    // 构建姿态对位置误差块：Fphip
    m_matrix3Workspace.setZero();
    m_matrix3Workspace(0, 0) = -WGS84_WIE * sinLatitude / meridianRadiusHeight;
    m_matrix3Workspace(0, 2) = navState.velocity[1] /
                               (primeVerticalRadiusHeight * primeVerticalRadiusHeight);
    m_matrix3Workspace(1, 2) = -navState.velocity[0] /
                               (meridianRadiusHeight * meridianRadiusHeight);
    m_matrix3Workspace(2, 0) = -WGS84_WIE * cosLatitude / meridianRadiusHeight -
                               navState.velocity[1] /
                                   (meridianRadiusHeight * primeVerticalRadiusHeight * cosLatitudeSquared);
    m_matrix3Workspace(2, 2) = -navState.velocity[1] * tanLatitude /
                               (primeVerticalRadiusHeight * primeVerticalRadiusHeight);
    Phi.block<3, 3>(ATTITUDE_ID, POSITION_ID) = m_matrix3Workspace;

    // 构建姿态误差相关块：Fphiv、Fphiphi、Fphibg、Fphisg
    m_matrix3Workspace.setZero();
    m_matrix3Workspace(0, 1) = 1.0 / primeVerticalRadiusHeight;
    m_matrix3Workspace(1, 0) = -1.0 / meridianRadiusHeight;
    m_matrix3Workspace(2, 1) = -tanLatitude / primeVerticalRadiusHeight;
    Phi.block<3, 3>(ATTITUDE_ID, VELOCITY_ID) = m_matrix3Workspace;
    Phi.block<3, 3>(ATTITUDE_ID, ATTITUDE_ID) =
        -Rotation::skewSymmetric(earthRateN + transportRateN);
    Phi.block<3, 3>(ATTITUDE_ID, GYRO_BIAS_ID) = -m_bodyToNavigationMatrix;
    Phi.block<3, 3>(ATTITUDE_ID, GYRO_SCALE_ID) =
        -m_bodyToNavigationMatrix * angularRate.asDiagonal();

    // 构建零偏和比例因子的一阶高斯-马尔科夫衰减块
    if (!std::isfinite(m_noiseParameters.correlationTime)) return false;
    if (m_noiseParameters.correlationTime <= 0.0) return false;

    const fp64 errorDecayRate = -1.0 / m_noiseParameters.correlationTime;
    m_matrix3Workspace = errorDecayRate * Eigen::Matrix3d::Identity();
    Phi.block<3, 3>(GYRO_BIAS_ID, GYRO_BIAS_ID) = m_matrix3Workspace;
    Phi.block<3, 3>(ACCEL_BIAS_ID, ACCEL_BIAS_ID) = m_matrix3Workspace;
    Phi.block<3, 3>(GYRO_SCALE_ID, GYRO_SCALE_ID) = m_matrix3Workspace;
    Phi.block<3, 3>(ACCEL_SCALE_ID, ACCEL_SCALE_ID) = m_matrix3Workspace;

    // 将连续时间状态矩阵F一阶离散化为Phi = I + F * dt
    Phi *= imuData.dt;
    Phi.diagonal().array() += 1.0;

    // 检查离散状态转移矩阵
    if (!Phi.allFinite()) {
        Phi.setZero();
        return false;
    }

    return true;
}

/**
 * @brief 根据连续时间噪声和当前姿态构建21维离散系统噪声矩阵Qd。
 * @return true 构建成功，false 输入或计算结果无效。
 */
bool INSESKF::structQMatrix(const NavState &navState,
                           const IMUData &imuData,
                           const ESKFStateMatrix &Phi,
                           ESKFStateMatrix &Qd){
    // 误差状态索引
    static constexpr Eigen::Index VELOCITY_ID = 3;
    static constexpr Eigen::Index ATTITUDE_ID = 6;
    static constexpr Eigen::Index GYRO_BIAS_ID = 9;
    static constexpr Eigen::Index ACCEL_BIAS_ID = 12;
    static constexpr Eigen::Index GYRO_SCALE_ID = 15;
    static constexpr Eigen::Index ACCEL_SCALE_ID = 18;

    // 连续时间驱动噪声索引
    static constexpr Eigen::Index VRW_NOISE_ID = 0;
    static constexpr Eigen::Index ARW_NOISE_ID = 3;
    static constexpr Eigen::Index GYRO_BIAS_NOISE_ID = 6;
    static constexpr Eigen::Index ACCEL_BIAS_NOISE_ID = 9;
    static constexpr Eigen::Index GYRO_SCALE_NOISE_ID = 12;
    static constexpr Eigen::Index ACCEL_SCALE_NOISE_ID = 15;
    static constexpr fp64 MIN_QUATERNION_NORM = 1e-8;

    // 输入数据有效性检查
    if (!m_isInitialized) return false;
    if (!std::isfinite(imuData.dt)) return false;
    if (imuData.dt <= 0.0) return false;
    if (!navState.attitude.coeffs().allFinite()) return false;
    if (!Phi.allFinite()) return false;
    if (!m_Qc.allFinite()) return false;

    const fp64 quaternionNorm = navState.attitude.norm();
    if (!std::isfinite(quaternionNorm)) return false;
    if (quaternionNorm < MIN_QUATERNION_NORM) return false;

    // 构建从机体坐标系到导航坐标系的姿态矩阵
    m_bodyToNavigationMatrix = navState.attitude.normalized().toRotationMatrix();
    if (!m_bodyToNavigationMatrix.allFinite()) return false;

    // 构建系统噪声驱动矩阵G
    m_noiseDriveMatrix.setZero();
    m_noiseDriveMatrix.block<3, 3>(VELOCITY_ID, VRW_NOISE_ID) = m_bodyToNavigationMatrix;
    m_noiseDriveMatrix.block<3, 3>(ATTITUDE_ID, ARW_NOISE_ID) = m_bodyToNavigationMatrix;
    m_noiseDriveMatrix.block<3, 3>(GYRO_BIAS_ID, GYRO_BIAS_NOISE_ID).setIdentity();
    m_noiseDriveMatrix.block<3, 3>(ACCEL_BIAS_ID, ACCEL_BIAS_NOISE_ID).setIdentity();
    m_noiseDriveMatrix.block<3, 3>(GYRO_SCALE_ID, GYRO_SCALE_NOISE_ID).setIdentity();
    m_noiseDriveMatrix.block<3, 3>(ACCEL_SCALE_ID, ACCEL_SCALE_NOISE_ID).setIdentity();

    // 将连续时间驱动噪声Qc映射并离散化到误差状态空间
    m_noiseMappingWorkspace.noalias() = m_noiseDriveMatrix * m_Qc;
    m_noiseAtIntervalStart.noalias() =
        m_noiseMappingWorkspace * m_noiseDriveMatrix.transpose();
    m_noiseAtIntervalStart *= imuData.dt;

    m_stateMatrixWorkspace.noalias() = Phi * m_noiseAtIntervalStart;
    m_noiseAtIntervalEnd.noalias() =
        m_stateMatrixWorkspace * Phi.transpose();

    m_discreteNoiseMatrix = (m_noiseAtIntervalStart + m_noiseAtIntervalEnd) * 0.5;

    if (!m_discreteNoiseMatrix.allFinite()) return false;

    // 所有计算成功后再更新输出，避免失败时留下半更新矩阵
    Qd = m_discreteNoiseMatrix;
    return true;
}

Eigen::Vector3d INSESKF::getPositionErrorState() const{
    static constexpr Eigen::Index ACCEL_BIAS_ID = 0;
    return m_errorState.segment<3>(ACCEL_BIAS_ID);
}

Eigen::Vector3d INSESKF::getVelocityErrorState() const{
    static constexpr Eigen::Index VEL_BIAS_ID = 3;
    return m_errorState.segment<3>(VEL_BIAS_ID);
}

Eigen::Vector3d INSESKF::getAttitudeErrorState() const{
    static constexpr Eigen::Index ATT_BIAS_ID = 6;
    return m_errorState.segment<3>(ATT_BIAS_ID);
}

Eigen::Vector3d INSESKF::getGyroBiasErrorState() const {
    static constexpr Eigen::Index GYRO_BIAS_ID = 9;
    return m_errorState.segment<3>(GYRO_BIAS_ID);
}

Eigen::Vector3d INSESKF::getAccelBiasErrorState() const {
    static constexpr Eigen::Index ACCEL_BIAS_ID = 12;
    return m_errorState.segment<3>(ACCEL_BIAS_ID);
}

Eigen::Vector3d INSESKF::getGyroScaleErrorState() const {
    static constexpr Eigen::Index GYRO_SCALE_ID = 15;
    return m_errorState.segment<3>(GYRO_SCALE_ID);
}

Eigen::Vector3d INSESKF::getAccelScaleErrorState() const {
    static constexpr Eigen::Index ACCEL_SCALE_ID = 18;
    return m_errorState.segment<3>(ACCEL_SCALE_ID);
}

// 想不到吧我把它放到最后面了杂鱼杂鱼 >_<
bool INSESKF::resetErrorState(){
    if (!m_isInitialized) return false;
    m_errorState.setZero();
    return true;
}



}
