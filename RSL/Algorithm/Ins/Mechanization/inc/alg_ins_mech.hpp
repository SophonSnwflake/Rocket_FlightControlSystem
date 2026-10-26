/**
 ******************************************************************************
 * @file           : alg_ins_mech.hpp
 * @brief          : 惯性导航机械编排实现
 * @brief          : INS mechanization implementation
 ******************************************************************************
 * @attention
 * Copyright (c) Jihao Wang XJTLU. 
 * All rights reserved.
 *
 ******************************************************************************
 */


#pragma once

#include <cmath>
#include <Eigen/Dense>

#include "RSL_common.h"
#include "alg_general.hpp"
#include "alg_ins_types.hpp"

class INSMechanization {
public:
    INSMechanization() = default;
    ~INSMechanization() = default;

    bool insMech(const NavState &navState_previous, NavState &navState_current,const IMUData &imuData_previous, const IMUData &imuData_current, const INSState &insState);

    bool updateVelocity(const NavState &navState_previous, NavState &navState_current, const IMUData &imuData_previous, const IMUData &imuData_current, const INSState &insState);
    bool updatePosition(const NavState &navState_previous, NavState &navState_current, const IMUData &imuData_previous, const IMUData &imuData_current, const INSState &insState);
    bool updateAttitude(const NavState &navState_previous, NavState &navState_current, const IMUData &imuData_previous, const IMUData &imuData_current, const INSState &insState);
};