/**
 ******************************************************************************
 * @file           : alg_ins_eskf.hpp
 * @brief          : INS error-state Kalman filter interface
 ******************************************************************************
 * @attention
 * Copyright (c) Jihao Wang XJTLU.
 * All rights reserved.
 *
 ******************************************************************************
 */

#pragma once

#include <Eigen/Core>

#include "RSL_common.h"
#include "alg_ins_types.hpp"

namespace INS {

class INSESKF {
private:
    // 滤波器状态与参数
    ESKFErrorStateVector m_errorState = ESKFErrorStateVector::Zero();
    ESKFStateMatrix m_P = ESKFStateMatrix::Zero(); // 协方差矩阵
    ESKFStateMatrix m_Q = ESKFStateMatrix::Zero(); // 离散系统噪声矩阵Qd
    ESKFStateMatrix m_Phi = ESKFStateMatrix::Zero(); // 离散的状态转移矩阵，Phi = I + F*dt
    ESKFNoiseMatrix m_Qc = ESKFNoiseMatrix::Zero(); // 连续时间驱动噪声矩阵
    ESKFNoiseParameters m_noiseParameters{};

    bool m_isInitialized = false;

private:
    // 矩阵计算工作区，仅保存计算过程中的临时结果
    Eigen::Matrix3d m_bodyToNavigationMatrix = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d m_matrix3Workspace = Eigen::Matrix3d::Zero();
    ESKFErrorStateVector m_predictedErrorState = ESKFErrorStateVector::Zero();
    ESKFStateMatrix m_predictedCovariance = ESKFStateMatrix::Zero();
    ESKFStateMatrix m_stateMatrixWorkspace = ESKFStateMatrix::Zero();
    ESKFNoiseDriveMatrix m_noiseDriveMatrix = ESKFNoiseDriveMatrix::Zero();
    ESKFNoiseDriveMatrix m_noiseMappingWorkspace = ESKFNoiseDriveMatrix::Zero();
    ESKFStateMatrix m_noiseAtIntervalStart = ESKFStateMatrix::Zero();
    ESKFStateMatrix m_noiseAtIntervalEnd = ESKFStateMatrix::Zero();
    ESKFStateMatrix m_discreteNoiseMatrix = ESKFStateMatrix::Zero();

public:
    INSESKF() = default;
    ~INSESKF() = default;

    bool init(const ESKFStateMatrix &initialCovariance, const ESKFNoiseParameters &noiseParameters);
    bool predict(const NavState &navState, const IMUData &imuData);

    bool correct(const LinearizedMeasurement &measurement);

    bool isInitialized() const { return m_isInitialized;}
    Eigen::Vector3d getGyroBiasErrorState() const;
    Eigen::Vector3d getAccelBiasErrorState() const;
    Eigen::Vector3d getGyroScaleErrorState() const;
    Eigen::Vector3d getAccelScaleErrorState() const;
    Eigen::Vector3d getPositionErrorState() const;
    Eigen::Vector3d getVelocityErrorState() const;
    Eigen::Vector3d getAttitudeErrorState() const;

    bool resetErrorState(); // 重置误差状态向量
    // 为什么呢？因为名义状态补偿过后，应该假设误差状态为零，这叫闭环ESKF，而不是开环ESKF噢
    // 这也意味着预测步里面的状态转移矩阵乘出来一直是0噢，
    // 但是当前ESKF保留了先验估计值的计算，因为祖宗之法不可变
    // 杂鱼杂鱼
    // 这意味着实际上预测步的误差转移只依靠协方差更新噢宝宝

private:
    // 预测步
    bool structPhiMatrix(const NavState &navState, const IMUData &imuData, ESKFStateMatrix &Phi);
    bool structQMatrix(const NavState &navState, const IMUData &imuData, const ESKFStateMatrix &Phi, ESKFStateMatrix &Qd);

    // 更新步
    bool structHMatrix(const NavState &navState, const ESKFObservationMatrix &H);
    bool structRMatrix(const GNSSMeasurement &gnssMeasurement, const ESKFMeasurementNoiseMatrix &R);
};

}
