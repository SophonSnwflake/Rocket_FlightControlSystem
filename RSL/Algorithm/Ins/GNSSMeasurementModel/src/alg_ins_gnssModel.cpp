#include "alg_ins_gnssModel.hpp"

#include <cmath>

#include "earth.h"
#include "rotation.h"

namespace INS {

bool GNSSMeasurementModel::init(const Eigen::Vector3f &antennaLeverArmBody) {
    if (!antennaLeverArmBody.allFinite()) return false;

    m_antennaLeverArmBody = antennaLeverArmBody;
    m_isInitialized = true;

    return true;
}

bool GNSSMeasurementModel::build(const NavState &navState, const GNSSMeasurement &gnssMeasurement, LinearizedMeasurement &measurement){
    if (!m_isInitialized) return false;

    ESKFMeasurementVector innovation;
    ESKFObservationMatrix H;
    ESKFMeasurementNoiseMatrix R;

    INS_TRY(structInnovation(navState, gnssMeasurement, innovation));
    INS_TRY(structHMatrix(navState, H));
    INS_TRY(structRMatrix(gnssMeasurement, R));

    measurement.innovation = innovation;
    measurement.H = H;
    measurement.R = R;

    return true;
}

bool GNSSMeasurementModel::structHMatrix(const NavState &navState, ESKFObservationMatrix &H) {
    static constexpr Eigen::Index POSITION_ID = 0;
    static constexpr Eigen::Index ATTITUDE_ID = 6;
    static constexpr fp64 MIN_QUATERNION_NORM = 1e-8;

    if (!m_isInitialized) return false;
    if (!navState.attitude.coeffs().allFinite()) return false;

    const fp64 quaternionNorm = navState.attitude.norm();
    if (!std::isfinite(quaternionNorm)) return false;
    if (quaternionNorm < MIN_QUATERNION_NORM) return false;

    m_matrix3Workspace = navState.attitude.normalized().toRotationMatrix();
    m_vector3Workspace = m_antennaLeverArmBody.cast<fp64>();
    m_antennaLeverArmNavigation = m_matrix3Workspace * m_vector3Workspace;

    H.setZero();

    // 位置误差观测块
    H.block<3, 3>(0, POSITION_ID).setIdentity();

    // 杆臂引起的姿态误差观测块
    H.block<3, 3>(0, ATTITUDE_ID) = Rotation::skewSymmetric(m_antennaLeverArmNavigation);

    return true;
}

bool GNSSMeasurementModel::structInnovation(const NavState &navState, const GNSSMeasurement &gnssMeasurement, ESKFMeasurementVector &innovation) {
    static constexpr fp64 MIN_QUATERNION_NORM = 1e-8;
    static constexpr fp64 MIN_LATITUDE_COSINE = 1e-8;
    static constexpr fp64 HALF_PI = 1.57079632679489661923;

    if (!m_isInitialized) return false;
    if (!gnssMeasurement.isPositionValid) return false;
    if (!navState.position.allFinite()) return false;
    if (!navState.attitude.coeffs().allFinite()) return false;
    if (!gnssMeasurement.positionBLH.allFinite()) return false;
    if (std::abs(navState.position[0]) > HALF_PI) return false;
    if (std::abs(gnssMeasurement.positionBLH[0]) > HALF_PI) return false;

    const fp64 quaternionNorm = navState.attitude.norm();
    if (!std::isfinite(quaternionNorm)) return false;
    if (quaternionNorm < MIN_QUATERNION_NORM) return false;

    const fp64 latitudeCosine = std::cos(navState.position[0]);
    if (!std::isfinite(latitudeCosine)) return false;
    if (std::abs(latitudeCosine) < MIN_LATITUDE_COSINE) return false;

    m_matrix3Workspace = navState.attitude.normalized().toRotationMatrix();
    m_vector3Workspace = m_antennaLeverArmBody.cast<fp64>();
    m_antennaLeverArmNavigation = m_matrix3Workspace * m_vector3Workspace;

    m_matrix3Workspace = Earth::DRi(navState.position);
    if (!m_matrix3Workspace.allFinite()) return false;

    // 将IMU位置转换到GNSS天线相位中心位置
    m_vector3Workspace =
        navState.position + m_matrix3Workspace * m_antennaLeverArmNavigation;
    if (!m_vector3Workspace.allFinite()) return false;

    m_matrix3Workspace = Earth::DR(navState.position);
    if (!m_matrix3Workspace.allFinite()) return false;

    // 误差状态定义为INS预测误差，因此新息采用预测位置减GNSS观测位置
    m_innovationWorkspace =
        m_matrix3Workspace * (m_vector3Workspace - gnssMeasurement.positionBLH);
    if (!m_innovationWorkspace.allFinite()) return false;

    innovation = m_innovationWorkspace;
    return true;
}

bool GNSSMeasurementModel::structRMatrix(const GNSSMeasurement &gnssMeasurement, ESKFMeasurementNoiseMatrix &R) {
    if (!gnssMeasurement.isAccuracyValid) return false;
    if (!gnssMeasurement.positionStdNED.allFinite()) return false;
    if ((gnssMeasurement.positionStdNED.array() <= 0.0).any()) return false;

    m_vector3Workspace =
        gnssMeasurement.positionStdNED.cwiseProduct(gnssMeasurement.positionStdNED);

    if (!m_vector3Workspace.allFinite()) return false;
    if ((m_vector3Workspace.array() <= 0.0).any()) return false;

    // GNSS位置观测噪声协方差矩阵，主对角线为NED三轴定位标准差的平方
    R.setZero();
    R.diagonal() = m_vector3Workspace;

    return true;
}

}
