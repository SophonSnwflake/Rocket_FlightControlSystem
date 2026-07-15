#pragma once

#include "RSL_common.h"
#include <cmath>
#include <type_traits>
#include <array>
#include <Eigen/Dense>

template <typename T = fp32, int StateSize = 1, int MeasSize = 1, int ControlSize = 0>
class KalmanFilter {
public:
    // Eigen矩阵类型定义
    using StateVector   = Eigen::Vector<T, StateSize>;
    using MeasVector    = Eigen::Vector<T, MeasSize>;
    using ControlVector = Eigen::Vector<T, ControlSize>;
    using StateMatrix   = Eigen::Matrix<T, StateSize, StateSize>;
    using MeasMatrix    = Eigen::Matrix<T, MeasSize, MeasSize>;
    using ObsMatrix     = Eigen::Matrix<T, MeasSize, StateSize>;
    using GainMatrix    = Eigen::Matrix<T, StateSize, MeasSize>;
    using ControlMatrix = Eigen::Matrix<T, StateSize, ControlSize>;

private:
StateVector m_state;        // 状态向量 x
    StateMatrix m_covariance;   // 协方差矩阵 P
    StateMatrix m_transition;   // 状态转移矩阵 F
    StateMatrix m_processNoise; // 过程噪声协方差 Q
    ObsMatrix m_observation;    // 观测矩阵 H
    MeasMatrix m_measNoise;     // 测量噪声协方差 R
    ControlMatrix m_control;    // 控制矩阵 B

    // 临时变量和缓存
    StateVector m_statePred;
    StateMatrix m_covPred;
    GainMatrix m_gain;
    StateMatrix m_identityMatrix; // 单位矩阵预定义，避免重复构造

    // 动态测量调整相关
    bool m_useAutoAdjustment;                    // 启用自动调整
    int m_validMeasurementCount;                 // 有效测量数量
    std::array<int, MeasSize> m_measurementMap;  // 测量与状态的映射关系
    std::array<T, MeasSize> m_measurementDegree; // 测量度量（H矩阵对应元素值）
    std::array<T, MeasSize> m_rDiagonalElements; // R矩阵对角元素

    // 过度收敛保护
    StateVector m_stateMinVariance; // 状态最小方差保护

    // 数值稳定性相关
    static constexpr T EPSILON = static_cast<T>(1e-6f); // 单精度浮点适配的数值稳定性阈值
};