/**
 ******************************************************************************
 * @file           : alg_ins_mech.cpp
 * @brief          : 惯性导航机械编排实现
 * @brief          : INS mechanization implementation
 ******************************************************************************
 * @attention
 * Copyright (c) Jihao Wang XJTLU. 
 * All rights reserved.
 *
 ******************************************************************************
 */

#include "alg_ins_mech.hpp"
#include "alg_ins_mech_para.hpp"
#include "earth.h"
#include "rotation.h"

bool INSMechanization::insMech(const NavState &navState_previous, NavState &navState_current,const IMUData &imuData_previous, const IMUData &imuData_current, const INSState &insState){ 
    if(insState == INSState::Running){
        if(!updateVelocity(navState_previous, navState_current, imuData_previous, imuData_current, insState)){
            return false;
        }
        if(!updatePosition(navState_previous, navState_current, imuData_previous, imuData_current, insState)){
            return false;
        }
    }
    if(!updateAttitude(navState_previous, navState_current, imuData_previous, imuData_current, insState)){
        return false;
    }
    return true;
}

/**
 * @brief 使用相邻两帧IMU增量完成NED导航系速度更新。
 *
 * @param navState_previous 前一历元的名义位置、速度和姿态。
 * @param navState_current 输出当前历元速度的导航状态。
 * @param imuData_previous 前一采样周期的角增量和速度增量。
 * @param imuData_current 当前采样周期的角增量、速度增量及时间间隔。
 * @param insState INS运行状态；当前实现仅用于保持统一接口。
 * @return true 速度更新成功，输出速度有效。
 * @return false 输入时间、IMU数据或导航状态无效。
 */
bool INSMechanization::updateVelocity(const NavState &navState_previous, NavState &navState_current,const IMUData &imuData_previous, const IMUData &imuData_current, const INSState &insState){
// 当前速度 = 上一时刻速度 + 比力更新 + 场力更新

    static const double MAX_TIME_DT_S = 0.1; // 最大时间间隔
    static const double MIN_TIME_DT_S = 0.0001; // 最小时间间隔

    if(!std::isfinite(imuData_current.dt))return false;
    if(!std::isfinite(imuData_previous.dt))return false;
    if(imuData_current.dt > MAX_TIME_DT_S)return false;
    if(imuData_current.dt < MIN_TIME_DT_S)return false;
    if(imuData_previous.dt > MAX_TIME_DT_S)return false;
    if(imuData_previous.dt < MIN_TIME_DT_S)return false;
    if(!imuData_current.dtheta.allFinite())return false;
    if(!imuData_current.dvelocity.allFinite())return false;
    if(!imuData_previous.dtheta.allFinite())return false;
    if(!imuData_previous.dvelocity.allFinite())return false;
    if(!navState_previous.position.allFinite())return false;
    if(!navState_previous.velocity.allFinite())return false;
    if(!navState_previous.attitude.coeffs().allFinite())return false;

    const fp64 quaternionNorm = navState_previous.attitude.norm();
    if (!std::isfinite(quaternionNorm) || quaternionNorm < 1e-8) return false;

    Eigen::Matrix3d Cbn; // 上一次姿态旋转矩阵
    Eigen::Matrix3d Cnn; // 导航系旋转矩阵，表示前后两个时间状态下，由于机体位置改变而导致的姿态旋转
    Eigen::Quaterniond qnn; // 导航系旋转四元数
    Eigen::Quaterniond qee; // 地心地固系旋转四元数
    Eigen::Quaterniond qne; // 导航系到地心地固系旋转四元数
    Eigen::Vector3d d_vfb; // 机体系下的速度增量
    Eigen::Vector3d d_vfn; // 导航系下的速度增量
    Eigen::Vector3d d_vgn; // 场力项导致的速度增量
    Eigen::Vector3d wie_n; // 地球自转角速度在导航系下的表示
    Eigen::Vector3d wen_n; // 由于机体位置变化导致的导航系旋转角速度在导航系下的表示
    Eigen::Vector3d local_gravity; // 当地重力加速度
    Eigen::Vector3d mid_velocity; // 中间速度
    Eigen::Vector3d mid_position; // 中间位置

    Eigen::Vector3d temp1;
    Eigen::Vector3d temp2;
    Eigen::Vector3d temp3;

    Cbn = navState_previous.attitude.toRotationMatrix();

    // 根据previous的经纬度信息计算子午圈半径和卯酉圈半径
    Eigen::Vector2d rmrn = Earth::meridianPrimeVerticalRadius(navState_previous.position(0));

    wie_n << WGS84_WIE * cos(navState_previous.position[0]), 0, -WGS84_WIE * sin(navState_previous.position[0]);
    wen_n << navState_previous.velocity[1] / (rmrn[1] + navState_previous.position[2]), -navState_previous.velocity[0] / (rmrn[0] + navState_previous.position[2]),
        -navState_previous.velocity[1] * tan(navState_previous.position[0]) / (rmrn[1] + navState_previous.position[2]);

    temp1 = (wie_n + wen_n) * imuData_current.dt / 2;

    Cnn = Eigen::Matrix3d::Identity() - Rotation::skewSymmetric(temp1);

    // 机体系下，比力积分的线性近似
    d_vfb = imuData_current.dvelocity 
            + (0.5f) * (imuData_current.dtheta.cross(imuData_current.dvelocity)) 
            + (1.0f / 12.0f) * (imuData_previous.dtheta.cross(imuData_current.dvelocity) + imuData_previous.dvelocity.cross(imuData_current.dtheta));

    // 算出以 前一时刻位速姿数据 为基础的比力更新
    d_vfn = Cnn * Cbn * d_vfb;

    fp64 gravity = Earth::gravity(navState_previous.position);
    local_gravity << 0, 0, gravity;

    // 算出以 前一时刻位速姿数据 为基础的场力更新
    d_vgn = (local_gravity - (2 * wie_n + wen_n).cross(navState_previous.velocity)) * imuData_current.dt;

    // 计算中间速度
    mid_velocity = navState_previous.velocity + (d_vfn + d_vgn) / 2;

    // 计算中间位置
    qnn = Rotation::rotvec2quaternion(temp1);
    temp2 << 0, 0, -WGS84_WIE * imuData_current.dt / 2;
    qee = Rotation::rotvec2quaternion(temp2);
    qne = Earth::qne(navState_previous.position);
    qne = (qee * qne * qnn).normalized();
    mid_position[2] = navState_previous.position[2] - mid_velocity[2] * imuData_current.dt / 2;
    mid_position    = Earth::blh(qne, mid_position[2]);

    // 根据中间位置重新计算子午圈半径和卯酉圈半径
    rmrn = Earth::meridianPrimeVerticalRadius(mid_position[0]);
    wie_n << WGS84_WIE * cos(mid_position[0]), 0, -WGS84_WIE * sin(mid_position[0]);
    wen_n << mid_velocity[1] / (rmrn[1] + mid_position[2]), -mid_velocity[0] / (rmrn[0] + mid_position[2]),
        -mid_velocity[1] * tan(mid_position[0]) / (rmrn[1] + mid_position[2]);

    temp3 = (wie_n + wen_n) * imuData_current.dt / 2;
    Cnn   = Eigen::Matrix3d::Identity() - Rotation::skewSymmetric(temp3);
    d_vfn = Cnn * navState_previous.attitude.toRotationMatrix() * d_vfb;

    // 重新计算重力、哥式积分项
    local_gravity << 0, 0, Earth::gravity(mid_position);
    d_vgn = (local_gravity - (2 * wie_n + wen_n).cross(mid_velocity)) * imuData_current.dt;

    // 最终速度更新
    navState_current.velocity = navState_previous.velocity + d_vfn + d_vgn;
    return true;
}