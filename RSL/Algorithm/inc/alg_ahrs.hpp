#pragma once

#include "RSL_common.h"
#include "alg_general.hpp"
#include "cmsis_os.h"
#include "stm32f411xe.h"
#include "alg_kalmanFilter.hpp"

class AHRS
{
// protected:
//仅供调试
public:
    using Vector3f = RSLMath::Vector3f;
    Vector3f m_gyro;
    Vector3f m_accel;
    Vector3f m_magnet;
    Vector3f m_eulerAngle;
    fp32 m_quaternion[4];
    Vector3f m_motionAccelBodyFrame;  // 机体坐标系下的运动加速度
    Vector3f m_motionAccelEarthFrame; // 大地坐标系下的运动加速度
    bool m_isAhrsInited;              // AHRS初始化完成标志

public:
    virtual ~AHRS() = default;
    virtual void reset();
    virtual void init() = 0; // AHRS初始化纯虚函数，在第一次update被调用时执行
    const Vector3f &update(const Vector3f &gyro, const Vector3f &accel, const Vector3f &magnet = Vector3f());
    virtual const Vector3f &getGyro() const;
    virtual const Vector3f &getAccel() const;
    virtual const Vector3f &getMotionAccelBodyFrame() const;
    virtual const Vector3f &getMotionAccelEarthFrame() const;
    const fp32 *getQuaternion() const;
    const Vector3f &getEulerAngle() const;

protected:
    AHRS();
    virtual void dataProcess() = 0;
    void convertQuaternionToEulerAngle();
    void initQuaternion();
    void calculateMotionAccel();
};

class QuaternionEKF : public AHRS{
private:
    using KF = KalmanFilter<fp32, 6, 3, 0>;
    KF m_kalmanFilter; // 卡尔曼滤波器实例
    // 采样频率相关
    fp32 m_sampleFreq;              // 固定采样频率(Hz), 为0则使用DWT自动计算
    uint32_t m_lastUpdateTimestamp; // DWT计数器历史值
    fp32 m_deltaTime;               // 实际采样周期(s)
    // 卡尔曼噪声参数
    fp32 m_quatProcessNoise; // 四元数更新过程噪声基准(Q矩阵)
    fp32 m_biasProcessNoise; // 陀螺仪零偏过程噪声基准(Q矩阵)
    fp32 m_measNoise;        // 加速度计量测噪声基准(R矩阵)
    fp32 m_lambda;           // 衰减系数(fading factor), 防止零偏协方差过度收敛
    // 滤波中间量
    Vector3f m_accelFiltered; // 低通滤波后的加速度值
    Vector3f m_gyroBias;      // 陀螺仪xy轴零偏估计
    // 卡方检测与自适应机制
    fp32 m_chiSquareThreshold;    // 卡方检验阈值
    fp32 m_accLpfCoef;            // 加速度计一阶低通滤波时间常数(s), 0表示不滤波
    bool m_isCheckChiSquare;      // 是否启用卡方检验
    bool m_convergeFlag;          // 滤波器收敛标志
    bool m_stableFlag;            // 载体静止稳定标志(角速度小且加速度接近重力)
    uint32_t m_errorCount;        // 连续卡方检验失败计数(用于发散保护)
    fp32 m_adaptiveGainScale;     // 自适应增益缩放因子
    fp32 m_gyroNorm;              // 当前角速度向量范数
    fp32 m_accelNorm;             // 当前加速度向量范数
    Vector3f m_orientationCosine; // 预测重力方向与各轴的余弦角


public:
    QuaternionEKF(fp32 sampleFreq         = 0.0f,
                  fp32 quatProcessNoise   = 10.0f,
                  fp32 biasProcessNoise   = 0.001f,
                  fp32 measNoise          = 1e6f,
                  fp32 lambda             = 1.0f,
                  fp32 accLpfCoef         = 0.0f,
                  bool isCheckChiSquare   = true,
                  fp32 chiSquareThreshold = 1e-8f);

    void reset() override;
    void init() override;

private:
    void dataProcess() override;
    void ekfProcess(fp32 gx, fp32 gy, fp32 gz, fp32 ax, fp32 ay, fp32 az);
};