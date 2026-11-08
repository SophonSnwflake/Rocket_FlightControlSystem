/**
 ******************************************************************************
 * @file           : alg_ins_gnssModel.hpp
 * @brief          : INS GNSS measurement model interface
 ******************************************************************************
 * @attention
 * Copyright (c) Jihao Wang XJTLU.
 * All rights reserved.
 *
 ******************************************************************************
 */

#pragma once

#include "alg_ins_types.hpp"

namespace INS {

class GNSSMeasurementModel {
public:
    GNSSMeasurementModel() = default;
    ~GNSSMeasurementModel() = default;

    bool init(const Eigen::Vector3f &antennaLeverArmBody);
    bool build(const NavState &navState, const GNSSMeasurement &gnssMeasurement, LinearizedMeasurement &measurement);

private:
    Eigen::Vector3f m_antennaLeverArmBody = Eigen::Vector3f::Zero(); // 竿壁的机体坐标系位置向量，单位为米
    bool m_isInitialized = false;

private:
    // 矩阵计算工作区，仅保存计算过程中的临时结果
    Eigen::Matrix3d m_matrix3Workspace = Eigen::Matrix3d::Zero();
    Eigen::Vector3d m_vector3Workspace = Eigen::Vector3d::Zero();
    Eigen::Vector3d m_antennaLeverArmNavigation = Eigen::Vector3d::Zero();
    ESKFMeasurementVector m_innovationWorkspace = ESKFMeasurementVector::Zero();

    bool structInnovation(const NavState &navState, const GNSSMeasurement &gnssMeasurement, ESKFMeasurementVector &innovation);
    bool structHMatrix(const NavState &navState, ESKFObservationMatrix &H);
    bool structRMatrix(const GNSSMeasurement &gnssMeasurement, ESKFMeasurementNoiseMatrix &R);
};

}
