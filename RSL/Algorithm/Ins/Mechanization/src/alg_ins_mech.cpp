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
#include <algorithm>

static const fp64 INSMECH_MAX_TIME_DT_S = 0.1; // 最大时间间隔
static const fp64 INSMECH_MIN_TIME_DT_S = 0.0001; // 最小时间间隔

bool INSMechanization::insMech(const NavState &navState_previous, NavState &navState_current,const IMUData &imuData_previous, const IMUData &imuData_current, const INSState &insState){ 
    if(insState == INSState::Running){
        if(!updateVelocity(navState_previous, navState_current, imuData_previous, imuData_current, insState)){
            return false;
        }
        if(!updatePosition(navState_previous, navState_current,imuData_current, insState)){
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

    // 检查合法性
    if(!std::isfinite(imuData_current.dt))return false;
    if(!std::isfinite(imuData_previous.dt))return false;
    if(imuData_current.dt > INSMECH_MAX_TIME_DT_S)return false;
    if(imuData_current.dt < INSMECH_MIN_TIME_DT_S)return false;
    if(imuData_previous.dt > INSMECH_MAX_TIME_DT_S)return false;
    if(imuData_previous.dt < INSMECH_MIN_TIME_DT_S)return false;
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


/**
 * @brief 使用中点积分方法完成 NED 导航系下的位置更新。
 *
 * 根据前一历元位置以及前后历元速度的平均值，计算中间时刻的位置、
 * 地球曲率半径、地球自转角速度和导航系运输角速度，随后通过导航系
 * 相对于地心地固系的完整周期旋转更新当前经纬度，并单独积分高度。
 *
 * @param navState_previous 前一历元导航状态；位置采用 BLH 格式，
 *                          依次为纬度(rad)、经度(rad)和高度(m)。
 * @param navState_current  当前历元导航状态。调用前其 velocity 应已经
 *                          完成更新；成功时本函数写入其 position。
 * @param imuData_current   当前 IMU 历元数据，本函数使用其中的时间间隔 dt。
 * @param insState          当前 INS 工作状态；当前实现中暂未直接使用。
 *
 * @return true  位置更新成功，navState_current.position 有效。
 * @return false 时间间隔、输入状态、中间计算结果或最终位置无效；
 *               此时不会修改 navState_current.position。
 *
 * @note 输入和输出速度均采用 NED，垂直速度向下为正，因此高度更新时
 *       需要减去垂直速度在当前采样周期内的积分量。
 */
bool INSMechanization::updatePosition(const NavState &navState_previous, NavState &navState_current, const IMUData &imuData_current, const INSState &insState){
    // 检查合法性
    if(imuData_current.dt > INSMECH_MAX_TIME_DT_S)return false;
    if(imuData_current.dt < INSMECH_MIN_TIME_DT_S)return false;
    if(!std::isfinite(imuData_current.dt)) return false;
    if(!navState_previous.position.allFinite()) return false;
    if(!navState_previous.velocity.allFinite()) return false;
    if(!navState_current.velocity.allFinite()) return false;
    if(std::abs(navState_previous.position[0]) > M_PI / 2) return false;

    // 计算纬度是否超小
    const fp64 previousCosLatitude =
    std::cos(navState_previous.position[0]);
    if(std::abs(previousCosLatitude) < 1e-8) return false;
    
    Eigen::Vector3d mid_velocity; // 中间速度
    Eigen::Vector3d mid_position; // 中间位置
    Eigen::Vector2d rmrn; // 子午圈半径和卯酉圈半径
    Eigen::Vector3d wie_n; // 地球自转角速度在导航系下的表示
    Eigen::Vector3d wen_n; // 由于机体位置变化导致的导航系旋转角速度在导航系下的表示
    Eigen::Quaterniond qnn; // 导航系旋转四元数
    Eigen::Quaterniond qee; // 地心地固系旋转四元数
    Eigen::Quaterniond qne; // 导航系到地心地固系旋转四元数

    // 计算中间时刻速度
    mid_velocity = (navState_previous.velocity + navState_current.velocity) / 2;
    if(!mid_velocity.allFinite()) return false;

    // 计算中间时刻位置
    mid_position = navState_previous.position + Earth::DRi(navState_previous.position) * mid_velocity * imuData_current.dt / 2; 
    if(!mid_position.allFinite()) return false;
    if(std::abs(mid_position[0]) >= M_PI / 2) return false;

    // 计算中间时刻地理参数
    rmrn = Earth::meridianPrimeVerticalRadius(mid_position[0]);
    const fp64 cosLatitude = std::cos(mid_position[0]);
    const fp64 rmHeight = rmrn[0] + mid_position[2];
    const fp64 rnHeight = rmrn[1] + mid_position[2];

    if(!std::isfinite(cosLatitude))return false;
    if(!std::isfinite(rmHeight))return false;
    if(!std::isfinite(rnHeight))return false;
    if(std::abs(cosLatitude) < 1e-8) return false;
    if(rmHeight < 1e-8) return false;
    if(rnHeight < 1e-8) return false;

    
    wie_n << WGS84_WIE * cosLatitude, 0, -WGS84_WIE * sin(mid_position[0]);
    wen_n << mid_velocity[1] / rnHeight, -mid_velocity[0] / rmHeight,
        -mid_velocity[1] * tan(mid_position[0]) / rnHeight;

    // 计算中间时刻导航系旋转四元数
    qnn = Rotation::rotvec2quaternion((wie_n + wen_n) * imuData_current.dt);
    qee = Rotation::rotvec2quaternion(Eigen::Vector3d(0, 0, -WGS84_WIE * imuData_current.dt));

    // 最终更新
    qne = Earth::qne(navState_previous.position);
    qne = qee * qne * qnn;

    // 检查合法性
    const fp64 qneNorm = qne.norm();
    if(!qne.coeffs().allFinite()) return false;
    if(!std::isfinite(qneNorm) || qneNorm < 1e-8) return false;

    qne.normalize();

    const fp64 newHeight = navState_previous.position[2] - mid_velocity[2] * imuData_current.dt;

    const Eigen::Vector3d newPosition = Earth::blh(qne, newHeight);
    if(!std::isfinite(newHeight)) return false;
    if(!newPosition.allFinite()) return false;
    navState_current.position    = newPosition;
    return true;
}


/**
 * @brief 使用相邻两帧 IMU 角增量预测当前历元姿态。
 *
 * 根据前后历元的位置和速度重建中间时刻导航状态，计算地球自转角速度
 * 与导航系运输角速度，并补偿导航坐标系在当前采样周期内的旋转。同时
 * 使用相邻两帧角增量进行二阶圆锥误差补偿，最终递推机体系到 NED
 * 导航系的姿态四元数。
 *
 * @param navState_previous 前一历元导航状态，提供位置、速度和姿态。
 * @param navState_current  当前历元导航状态；调用前位置和速度应已更新，
 *                          成功时本函数写入其 attitude。
 * @param imuData_previous  前一相邻采样周期的 IMU 角增量和时间间隔。
 * @param imuData_current   当前采样周期的 IMU 角增量和时间间隔。
 * @param insState          当前 INS 工作状态，用于检查调用状态是否合法。
 *
 * @return true  姿态预测成功，navState_current.attitude 为单位四元数。
 * @return false 输入状态、采样周期、地理参数、中间四元数或最终姿态无效；
 *               此时不会修改 navState_current.attitude。
 *
 * @note 两帧 IMU 数据必须时间连续且采样间隔近似相等；dtheta 的单位为
 *       rad，姿态四元数表示从机体系到 NED 导航系的旋转。
 */
bool INSMechanization::updateAttitude(const NavState &navState_previous, NavState &navState_current, const IMUData &imuData_previous, const IMUData &imuData_current, const INSState &insState){
// qbn = qnn @ qbn(k-1) @ qbb

    static constexpr fp64 MIN_QUATERNION_NORM = 1e-8;
    static constexpr fp64 MIN_COS_LATITUDE = 1e-8;
    static constexpr fp64 MIN_RADIUS_HEIGHT = 1e-8;

    if(insState != INSState::WaitingForInitialization && insState != INSState::Running) return false;
    if(!std::isfinite(imuData_previous.dt)) return false;
    if(!std::isfinite(imuData_current.dt)) return false;
    if(imuData_previous.dt < INSMECH_MIN_TIME_DT_S || imuData_previous.dt > INSMECH_MAX_TIME_DT_S) return false;
    if(imuData_current.dt < INSMECH_MIN_TIME_DT_S || imuData_current.dt > INSMECH_MAX_TIME_DT_S) return false;
    if(!imuData_previous.dtheta.allFinite()) return false;
    if(!imuData_current.dtheta.allFinite()) return false;
    if(!navState_previous.position.allFinite()) return false;
    if(!navState_current.position.allFinite()) return false;
    if(!navState_previous.velocity.allFinite()) return false;
    if(!navState_current.velocity.allFinite()) return false;
    if(!navState_previous.attitude.coeffs().allFinite()) return false;
    if(std::abs(navState_previous.position[0]) >= M_PI / 2) return false;
    if(std::abs(navState_current.position[0]) >= M_PI / 2) return false;

    const fp64 previousAttitudeNorm = navState_previous.attitude.norm();
    if(!std::isfinite(previousAttitudeNorm) || previousAttitudeNorm < MIN_QUATERNION_NORM) return false;

    Eigen::Vector3d mid_velocity; // 中间时刻速度
    Eigen::Vector3d mid_position; // 中间时刻位置
    Eigen::Quaterniond qne_pre; // 导航系到地球系的转换四元数，pre时刻
    Eigen::Quaterniond qne_mid; // 导航系到地球系的转换四元数，中间时刻
    Eigen::Quaterniond qne_cur; // 导航系到地球系的转换四元数, current时刻
    Eigen::Quaterniond qee; // 一个采样周期内地球自转导致的姿态四元数变化
    Eigen::Quaterniond qbb; // 自身转动
    Eigen::Quaterniond qnn; // 导航系转动
    Eigen::Vector3d temp1;

    // 重新计算中间时刻的速度和位置
    mid_velocity = (navState_previous.velocity + navState_current.velocity) / 2;
    if(!mid_velocity.allFinite()) return false;

    qne_pre   = Earth::qne(navState_previous.position);
    qne_cur   = Earth::qne(navState_current.position);
    if(!qne_pre.coeffs().allFinite()) return false;
    if(!qne_cur.coeffs().allFinite()) return false;

    const fp64 qnePreNorm = qne_pre.norm();
    const fp64 qneCurNorm = qne_cur.norm();
    if(!std::isfinite(qnePreNorm) || qnePreNorm < MIN_QUATERNION_NORM) return false;
    if(!std::isfinite(qneCurNorm) || qneCurNorm < MIN_QUATERNION_NORM) return false;

    qee       = Rotation::rotvec2quaternion(Eigen::Vector3d(0, 0, -WGS84_WIE * imuData_current.dt));
    if(!qee.coeffs().allFinite()) return false;

    Eigen::Quaterniond qne_delta = qne_pre.inverse() * qee.inverse() * qne_cur;
    const fp64 qneDeltaNorm = qne_delta.norm();
    if(!qne_delta.coeffs().allFinite()) return false;
    if(!std::isfinite(qneDeltaNorm) || qneDeltaNorm < MIN_QUATERNION_NORM) return false;
    qne_delta.normalize();

    temp1 = Rotation::quaternion2vector(qne_delta);
    if(!temp1.allFinite()) return false;

    qee       = Rotation::rotvec2quaternion(Eigen::Vector3d(0, 0, -WGS84_WIE * imuData_current.dt / 2));
    const Eigen::Quaterniond qnn_half = Rotation::rotvec2quaternion(temp1 / 2);
    if(!qee.coeffs().allFinite()) return false;
    if(!qnn_half.coeffs().allFinite()) return false;

    qne_mid = qee * qne_pre * qnn_half;
    const fp64 qneMidNorm = qne_mid.norm();
    if(!qne_mid.coeffs().allFinite()) return false;
    if(!std::isfinite(qneMidNorm) || qneMidNorm < MIN_QUATERNION_NORM) return false;
    qne_mid.normalize();

    const fp64 midHeight = (navState_current.position[2] + navState_previous.position[2]) / 2;
    if(!std::isfinite(midHeight)) return false;

    mid_position = Earth::blh(qne_mid, midHeight);
    if(!mid_position.allFinite()) return false;
    if(std::abs(mid_position[0]) >= M_PI / 2) return false;

    // 重新计算中间时刻地理参数
    Eigen::Vector2d rmrn;
    Eigen::Vector3d wie_n, wen_n;
    rmrn = Earth::meridianPrimeVerticalRadius(mid_position[0]);
    if(!rmrn.allFinite()) return false;

    const fp64 cosLatitude = std::cos(mid_position[0]);
    const fp64 rmHeight = rmrn[0] + mid_position[2];
    const fp64 rnHeight = rmrn[1] + mid_position[2];
    if(!std::isfinite(cosLatitude) || std::abs(cosLatitude) < MIN_COS_LATITUDE) return false;
    if(!std::isfinite(rmHeight) || rmHeight < MIN_RADIUS_HEIGHT) return false;
    if(!std::isfinite(rnHeight) || rnHeight < MIN_RADIUS_HEIGHT) return false;

    wie_n << WGS84_WIE * cosLatitude, 0, -WGS84_WIE * std::sin(mid_position[0]);
    wen_n << mid_velocity[1] / rnHeight, -mid_velocity[0] / rmHeight,
        -mid_velocity[1] * std::tan(mid_position[0]) / rnHeight;
    if(!wie_n.allFinite()) return false;
    if(!wen_n.allFinite()) return false;

    // 计算n系的旋转四元数 k-1时刻到k时刻变换
    temp1 = -(wie_n + wen_n) * imuData_current.dt;
    if(!temp1.allFinite()) return false;

    qnn   = Rotation::rotvec2quaternion(temp1);
    if(!qnn.coeffs().allFinite()) return false;

    if((std::abs(imuData_current.dt - imuData_previous.dt) / std::max(imuData_current.dt, imuData_previous.dt)) > 0.1) return false;

    // 计算b系旋转四元数 补偿二阶圆锥误差(基于相同采样间隔推导)
    temp1 = imuData_current.dtheta + imuData_previous.dtheta.cross(imuData_current.dtheta) / 12;
    if(!temp1.allFinite()) return false;

    qbb   = Rotation::rotvec2quaternion(temp1);
    if(!qbb.coeffs().allFinite()) return false;

    // 姿态更新完成
    Eigen::Quaterniond newAttitude = qnn * navState_previous.attitude * qbb;
    const fp64 newAttitudeNorm = newAttitude.norm();
    if(!newAttitude.coeffs().allFinite()) return false;
    if(!std::isfinite(newAttitudeNorm) || newAttitudeNorm < MIN_QUATERNION_NORM) return false;

    newAttitude.normalize();
    navState_current.attitude = newAttitude;
    return true;
}
