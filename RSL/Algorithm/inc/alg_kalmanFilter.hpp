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

public:
    KalmanFilter() : m_useAutoAdjustment(false), m_validMeasurementCount(0)
    {
        initializeMatrices();
        initializeDynamicAdjustment();
    }

    explicit KalmanFilter(T processNoiseVar, T measurementNoiseVar = static_cast<T>(0.1),bool useAutoAdjustment = false) : 
            m_useAutoAdjustment(useAutoAdjustment), m_validMeasurementCount(0){
            initializeMatrices();
            initializeDynamicAdjustment();
                
    }

    inline void update(const MeasVector &measurement)
    {
        // 扩展点：预处理回调
        onPreUpdate(measurement);

        // 预测步骤
        predict();

        //动态调整矩阵（如果启用）
        if(m_useAutoAdjustment){
            auto [validMeas, validH, validR] = performDynamicAdjustment(measurement);
            if(m_validMeasurementCount > 0){
                updateWithDynamicMeasurement(validMeas, validH, validR);
            } 
            else{
                m_state = m_statePred;
                m_covariance = m_covPred;
            }
        }
        else{
            //标准更新流程
            performStandardUpdate(measurement);
        }

        //过度收敛保护
        applyConvergenceProtection();

        //扩展点：后处理回调
        onPostUpdate();
    }

    /**
     * @brief 带控制输入的测量更新
     */
    template <bool EnableControl = (ControlSize > 0)>
    typename std::enable_if<EnableControl, void>::type
    update(const MeasVector &measurement, const ControlVector &control)
    {
        // 扩展点：预处理回调
        onPreUpdate(measurement);

        // 预测步骤
        predict(control);

        // 动态调整矩阵（如果启用）
        if (m_useAutoAdjustment) {
            auto [validMeas, validH, validR] = performDynamicAdjustment(measurement);
            if (m_validMeasurementCount > 0) {
                updateWithDynamicMeasurement(validMeas, validH, validR);
            } else {
                // 无有效测量，仅使用预测结果
                m_state      = m_statePred;
                m_covariance = m_covPred;
            }
        } else {
            // 标准更新流程
            performStandardUpdate(measurement);
        }

        // 过度收敛保护
        applyConvergenceProtection();

        // 扩展点：后处理回调
        onPostUpdate();
    }

    /**
     * @brief 预测步骤（无控制输入）
     */
    inline void predict()
    {
        // 扩展点：预测回调
        onPredict();

        // 状态预测 x- = F * x (可通过customStateTransition自定义)
        m_statePred = customStateTransition(m_state);

        // 协方差预测 P- = F * P * F^T + Q
        m_covPred = m_transition * m_covariance * m_transition.transpose() + m_processNoise;
    }

    template <bool EnableControl = (ControlSize > 0)>
    typename std::enable_if<EnableControl, void>::type
    predict(const ControlVector &control)
    {
        //扩展点：预测回调
        onPredict();

        // 状态预测
        m_statePred = customStateTransition(m_state) + m_control * control;

        //协方差预测
        m_covPred = m_transition * m_covariance * m_transition.transpose() + m_processNoise;
    }

    /**
     * @brief 重置滤波器
     */
    void reset() 
    {
        m_state.setZero();
        m_covariance.setIdentity();
    }

    /**
     * @brief 设置状态转移矩阵
     */
    void setTransitionMatrix(const StateMatrix &F)
    {
        m_transition = F;
    }

    /**
     * @brief 设置观测矩阵
     */
    void setObservationMatrix(const ObsMatrix &H)
    {
        m_observation = H;
    }

    /**
     * @brief 设置过程噪声协方差
     */
    void setProcessNoise(const StateMatrix &Q)
    {
        m_processNoise = Q;
    }

    /**
     * @brief 设置测量噪声协方差
     */
    void setMeasurementNoise(const MeasMatrix &R)
    {
        m_measNoise = R;
    }

    /**
     * @brief 获取当前状态
     */
    const StateVector &getState() const
    {
        return m_state;
    }

    /**
     * @brief 获取当前协方差矩阵
     */
    const StateMatrix &getCovariance() const
    {
        return m_covariance;
    }

    /**
     * @brief 设置初始状态
     */
    void setState(const StateVector &x)
    {
        m_state = x;
    }

    /**
     * @brief 设置初始协方差
     */
    void setCovariance(const StateMatrix &P)
    {
        m_covariance = P;
    }

    /**
     * @brief 设置控制矩阵
     */
    template <bool EnableControl = (ControlSize > 0)>
    typename std::enable_if<EnableControl, void>::type
    setControlMatrix(const ControlMatrix &B)
    {
        m_control = B;
    }

    /**
     * @brief 获取卡尔曼增益矩阵
     */
    const GainMatrix &getKalmanGain() const
    {
        return m_gain;
    }

    /**
     * @brief 检查滤波器是否已初始化
     */
    bool isInitialized() const
    {
        return m_covariance.determinant() > EPSILON;
    }

    /**
     * @brief 设置动态测量调整参数
     * @param measurementMap 测量与状态的映射关系 (1-based index)
     * @param measurementDegree 测量度量（H矩阵对应元素值）
     * @param rDiagonalElements R矩阵对角元素
     */
    void setDynamicAdjustmentParams(const std::array<int, MeasSize> &measurementMap,
                                    const std::array<T, MeasSize> &measurementDegree,
                                    const std::array<T, MeasSize> &rDiagonalElements)
    {
        m_measurementMap    = measurementMap;
        m_measurementDegree = measurementDegree;
        m_rDiagonalElements = rDiagonalElements;
    }

    /**
     * @brief 启用/禁用动态测量调整
     */
    void enableAutoAdjustment(bool enable)
    {
        m_useAutoAdjustment = enable;
    }

    /**
     * @brief 设置状态最小方差保护
     */
    void setStateMinVariance(const StateVector &minVariance)
    {
        m_stateMinVariance = minVariance;
    }

    /**
     * @brief 获取有效测量数量
     */
    int getValidMeasurementCount() const
    {
        return m_validMeasurementCount;
    }

    /**
     * @brief 获取预测状态
     */
    const StateVector &getPredictedState() const
    {
        return m_statePred;
    }

    /**
     * @brief 获取预测协方差
     */
    const StateMatrix &getPredictedCovariance() const
    {
        return m_covPred;
    }

protected:
    // 扩展点：为高级滤波器（EKF/UKF等）预留的虚函数接口

    /**
     * @brief 预更新回调（扩展点）
     * @details EKF可重写此函数进行线性化等预处理
     */
    virtual void onPreUpdate(const MeasVector &measurement)
    {
        // 默认空实现，派生类可重写
        (void)measurement; // 避免未使用参数警告
    }

    /**
     * @brief 后更新回调（扩展点）
     * @details 派生类可重写此函数进行后处理
     */
    virtual void onPostUpdate()
    {
        // 默认空实现，派生类可重写
    }

    /**
     * @brief 预测步骤回调（扩展点）
     * @details EKF可重写此函数自定义预测逻辑
     */
    virtual void onPredict()
    {
        // 默认空实现，派生类可重写
    }

    /**
     * @brief 自定义状态转移（扩展点）
     * @details UKF等非线性滤波器可重写此函数
     */
    virtual StateVector customStateTransition(const StateVector &state)
    {
        // 默认使用线性状态转移
        return m_transition * state;
    }

    /**
     * @brief 自定义测量模型（扩展点）
     * @details EKF等可重写此函数自定义测量预测
     */
    virtual MeasVector customMeasurementModel(const StateVector &state)
    {
        // 默认使用线性测量模型
        return m_observation * state;
    }

private:
    /**
     * @brief 初始化所有矩阵
     */
    void initializeMatrices()
    {
        // 状态和协方差初始化
        m_state.setZero();
        m_covariance.setIdentity();
        m_identityMatrix.setIdentity();

        // 系统矩阵初始化
        m_transition.setIdentity();
        m_processNoise = StateMatrix::Identity() * static_cast<T>(0.01);

        // 观测相关矩阵初始化
        m_observation.setZero();
        for (int i = 0; i < MeasSize && i < StateSize; ++i) {
            m_observation(i, i) = static_cast<T>(1);
        }
        m_measNoise = MeasMatrix::Identity() * static_cast<T>(0.1);

        // 控制矩阵初始化（仅当ControlSize > 0时）
        if constexpr (ControlSize > 0) {
            m_control.setZero();
        }

        // 临时变量初始化
        m_statePred.setZero();
        m_covPred.setZero();
        m_gain.setZero();
    }

    /**
     * @brief 初始化动态调整相关参数
     */
    void initializeDynamicAdjustment()
    {
        // 默认映射：测量i对应状态i
        for (int i = 0; i < MeasSize; ++i) {
            m_measurementMap[i]    = std::min(i + 1, StateSize); // 1-based index
            m_measurementDegree[i] = static_cast<T>(1);
            m_rDiagonalElements[i] = static_cast<T>(0.1);
        }

        // 初始化最小方差保护
        m_stateMinVariance.setConstant(static_cast<T>(1e-6));
    }

    /**
     * @brief 执行动态测量调整
     * @return [有效测量向量, 有效H矩阵, 有效R矩阵]
     */
    std::tuple<Eigen::VectorX<T>, Eigen::MatrixX<T>, Eigen::MatrixX<T>>
    performDynamicAdjustment(const MeasVector &measurement)
    {
        m_validMeasurementCount = 0;

        // 计算有效测量数量
        for (int i = 0; i < MeasSize; ++i) {
            if (std::abs(measurement(i)) > EPSILON) { // 非零即为有效
                m_validMeasurementCount++;
            }
        }

        if (m_validMeasurementCount == 0) {
            return {Eigen::VectorX<T>(), Eigen::MatrixX<T>(), Eigen::MatrixX<T>()};
        }

        // 构建动态矩阵
        Eigen::VectorX<T> validMeas(m_validMeasurementCount);
        Eigen::MatrixX<T> validH(m_validMeasurementCount, StateSize);
        Eigen::MatrixX<T> validR(m_validMeasurementCount, m_validMeasurementCount);

        validH.setZero();
        validR.setZero();

        int validIdx = 0;
        for (int i = 0; i < MeasSize; ++i) {
            if (std::abs(measurement(i)) > EPSILON) {
                validMeas(validIdx) = measurement(i);

                // 构建H矩阵行
                int stateIdx = m_measurementMap[i] - 1; // 转换为0-based
                if (stateIdx >= 0 && stateIdx < StateSize) {
                    validH(validIdx, stateIdx) = m_measurementDegree[i];
                }

                // 构建R矩阵对角元素
                validR(validIdx, validIdx) = m_rDiagonalElements[i];

                validIdx++;
            }
        }

        return {validMeas, validH, validR};
    }

    /**
     * @brief 使用动态调整后的矩阵进行更新
     */
    void updateWithDynamicMeasurement(const Eigen::VectorX<T> &validMeas,
                                      const Eigen::MatrixX<T> &validH,
                                      const Eigen::MatrixX<T> &validR)
    {
        // 计算创新协方差 S = H * P- * H^T + R
        Eigen::MatrixX<T> S = validH * m_covPred * validH.transpose() + validR;

        // 检查矩阵奇异性
        Eigen::FullPivLU<Eigen::MatrixX<T>> lu(S);
        if (lu.isInvertible()) {
            // 计算卡尔曼增益 K = P- * H^T * S^-1
            Eigen::MatrixX<T> K = m_covPred * validH.transpose() * lu.inverse();

            // 计算创新
            Eigen::VectorX<T> innovation = validMeas - validH * m_statePred;

            // 状态更新
            m_state = m_statePred + K * innovation;

            // 协方差更新（Joseph形式）
            Eigen::MatrixX<T> IKH = StateMatrix::Identity() - K * validH;
            m_covariance          = IKH * m_covPred * IKH.transpose() + K * validR * K.transpose();
        } else {
            // 矩阵奇异，跳过更新
            m_state      = m_statePred;
            m_covariance = m_covPred;
        }
    }

    /**
     * @brief 标准更新流程（无动态调整）
     */
    void performStandardUpdate(const MeasVector &measurement)
    {
        // 计算创新协方差 S = H * P- * H^T + R
        MeasMatrix S = m_observation * m_covPred * m_observation.transpose() + m_measNoise;

        // 检查矩阵奇异性，使用LU分解求解代替直接求逆提高数值稳定性
        Eigen::FullPivLU<MeasMatrix> lu(S);
        if (lu.isInvertible()) {
            // 计算卡尔曼增益 K = P- * H^T * S^-1
            m_gain = m_covPred * m_observation.transpose() * lu.inverse();

            // 计算创新 innovation = z - H * x-
            MeasVector innovation = measurement - m_observation * m_statePred;

            // 状态更新 x = x- + K * innovation
            m_state = m_statePred + m_gain * innovation;

            // 协方差更新 P = (I - K * H) * P- (Joseph形式)
            StateMatrix IKH = m_identityMatrix - m_gain * m_observation;
            m_covariance    = IKH * m_covPred * IKH.transpose() +
                           m_gain * m_measNoise * m_gain.transpose();
        } else {
            // 矩阵奇异，跳过更新步骤，仅保留预测结果
            m_state      = m_statePred;
            m_covariance = m_covPred;
        }
    }

    /**
     * @brief 应用过度收敛保护
     */
    void applyConvergenceProtection()
    {
        // 限制协方差矩阵对角元素的最小值
        for (int i = 0; i < StateSize; ++i) {
            if (m_covariance(i, i) < m_stateMinVariance(i)) {
                m_covariance(i, i) = m_stateMinVariance(i);
            }
        }
    }

};