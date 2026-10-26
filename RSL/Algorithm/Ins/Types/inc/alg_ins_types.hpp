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
#include "alg_general.hpp"

enum class INSState {
    WaitingForInitialization, // 等待初始化
    Running
};

typedef struct {
    Eigen::Vector3d gyroBias_error = Eigen::Vector3d::Zero(); // rad/s, 陀螺仪零偏误差
    Eigen::Vector3d accelBias_error = Eigen::Vector3d::Zero(); // m/s², 加速度计零偏误差
    Eigen::Vector3d gyroScale_error = Eigen::Vector3d::Zero(); // 陀螺仪比例因子误差
    Eigen::Vector3d accelScale_error = Eigen::Vector3d::Zero(); // 加速度计比例因子误差

} IMUError;


typedef struct {
    Eigen::Vector3d position_error = Eigen::Vector3d::Zero(); // m, 导航坐标系，遵循NED
    Eigen::Vector3d velocity_error = Eigen::Vector3d::Zero(); // m/s, 导航坐标系，遵循NED
    Eigen::Vector3d attitude_error = Eigen::Vector3d::Zero(); // rad, 姿态小角度误差向量
    
    IMUError imuError; // IMU误差状态
    
} NavErrorstate; // 误差状态空间定义


typedef struct {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();; // [rad, rad, m]
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero(); // m/s, 导航坐标系，遵循NED
    Eigen::Quaternion<fp64> attitude = Eigen::Quaterniond::Identity(); // 四元数表示的姿态

} NavState; // 导航状态空间定义


typedef struct {
    fp64 dt = 0.0;; // s, 时间间隔

    Eigen::Vector3d dtheta = Eigen::Vector3d::Zero(); // rad, 角增量
    Eigen::Vector3d dvelocity = Eigen::Vector3d::Zero(); // m/s, 速度增量

} IMUData; // 输入给惯性编排的IMU数据