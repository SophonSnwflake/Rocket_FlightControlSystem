/**
 ******************************************************************************
 * @file           : alg_ins_types.hpp
 * @brief          : 惯性导航中间数据类型定义
 * @brief          : INS intermediate data type definitions
 ******************************************************************************
 * @attention
 * Copyright (c) Jihao Wang XJTLU. 
 * All rights reserved.
 *
 ******************************************************************************
 */

#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "RSL_common.h"

namespace INS {

enum class INS_Error {
    OK = 0,
    IMUNotInited,
    INSNotInited,
    DeltaTimeTooSmall,
    DeltaTimeTooLarge,
    UpdateVelocityFailed,
    UpdatePositionFailed,
    UpdateAttitudeFailed,
    ESKFNotAvailable,
    ESKFNotInitialized,
    ESKFPredictFailed,
    MechanizationNotAvailable,
    ESKFResetFailed,
    ESKFFeedbackFailed
};

inline constexpr bool isINSSuccess(bool result) {
    return result;
}

inline constexpr bool isINSSuccess(INS_Error result) {
    return result == INS_Error::OK;
}

inline constexpr int ESKF_STATE_DIMENSION = 21;
inline constexpr int ESKF_MEASUREMENT_DIMENSION = 3;
inline constexpr int ESKF_NOISE_DIMENSION = 18;

using ESKFErrorStateVector = Eigen::Matrix<fp64, ESKF_STATE_DIMENSION, 1>;
using ESKFStateMatrix = Eigen::Matrix<fp64, ESKF_STATE_DIMENSION, ESKF_STATE_DIMENSION>;
using ESKFMeasurementVector = Eigen::Matrix<fp64, ESKF_MEASUREMENT_DIMENSION, 1>;
using ESKFObservationMatrix = Eigen::Matrix<fp64, ESKF_MEASUREMENT_DIMENSION, ESKF_STATE_DIMENSION>;
using ESKFMeasurementNoiseMatrix = Eigen::Matrix<fp64, ESKF_MEASUREMENT_DIMENSION, ESKF_MEASUREMENT_DIMENSION>;
using ESKFNoiseMatrix = Eigen::Matrix<fp64, ESKF_NOISE_DIMENSION, ESKF_NOISE_DIMENSION>;
using ESKFNoiseDriveMatrix = Eigen::Matrix<fp64, ESKF_STATE_DIMENSION, ESKF_NOISE_DIMENSION>;

struct ESKFNoiseParameters {
    Eigen::Vector3d gyroARW = Eigen::Vector3d::Zero(); // rad/sqrt(s)
    Eigen::Vector3d accelVRW = Eigen::Vector3d::Zero(); // m/s/sqrt(s)

    Eigen::Vector3d gyroBiasStd = Eigen::Vector3d::Zero(); // rad/s
    Eigen::Vector3d accelBiasStd = Eigen::Vector3d::Zero(); // m/s^2
    Eigen::Vector3d gyroScaleStd = Eigen::Vector3d::Zero(); // dimensionless
    Eigen::Vector3d accelScaleStd = Eigen::Vector3d::Zero(); // dimensionless

    fp64 correlationTime = 0.0; // s
};

struct LinearizedMeasurement {
    ESKFMeasurementVector innovation = ESKFMeasurementVector::Zero();
    ESKFObservationMatrix H = ESKFObservationMatrix::Zero();
    ESKFMeasurementNoiseMatrix R = ESKFMeasurementNoiseMatrix::Zero();
};

}

#define INS_TRY(expr)                                      \
    do                                                     \
    {                                                      \
        const auto insTryResult = (expr);                  \
        if (!::INS::isINSSuccess(insTryResult))            \
        {                                                  \
            return insTryResult;                           \
        }                                                  \
    } while (false)

enum class INSState {
    WaitingForInitialization, // 等待初始化
    Running
};


typedef struct {
    Eigen::Vector3d gyroBias_error = Eigen::Vector3d::Zero(); // rad/s, 陀螺仪零偏零偏
    Eigen::Vector3d accelBias_error = Eigen::Vector3d::Zero(); // m/s², 加速度计零偏零偏
    Eigen::Vector3d gyroScale_error = Eigen::Vector3d::Zero(); // 陀螺仪比例因子零偏
    Eigen::Vector3d accelScale_error = Eigen::Vector3d::Zero(); // 加速度计比例因子零偏

} IMUError;// 仪器名义零偏


typedef struct {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();; // [rad, rad, m]
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero(); // m/s, 导航坐标系，遵循NED
    Eigen::Quaternion<fp64> attitude = Eigen::Quaterniond::Identity(); // 四元数表示的姿态

    IMUError imuError = IMUError{};

} NavState; // 导航状态空间定义（名义状态）


typedef struct {
    fp64 dt = 0.0; // s, 时间间隔

    Eigen::Vector3d dtheta = Eigen::Vector3d::Zero(); // rad, 角增量
    Eigen::Vector3d dvelocity = Eigen::Vector3d::Zero(); // m/s, 速度增量

} IMUData; // 输入给惯性编排的IMU数据

struct GNSSMeasurement {
    fp64 timestamp = 0.0;

    Eigen::Vector3d positionBLH = Eigen::Vector3d::Zero(); // [rad, rad, m], 经纬度高程
    Eigen::Vector3d positionStdNED = Eigen::Vector3d::Zero(); // m, NED坐标系下的定位精度

    bool isPositionValid = false;
    bool isAccuracyValid = false;
    bool isTimestampValid = false;
};
