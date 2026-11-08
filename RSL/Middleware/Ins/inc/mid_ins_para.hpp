#pragma once

#include "math_const.h"

#define LAUNCH_PLACE_LATITUDE_RAD               (31.0f * MATH_PI / 180.0f) // 发射场纬度，单位：弧度
#define LAUNCH_PLACE_LONGITUDE_RAD              (120.0f * MATH_PI / 180.0f) // 发射场经度，单位：弧度
#define LAUNCH_PLACE_HEIGHT_M                   0.0f // 发射场海拔高度，单位：米

#define IS_USE_IMU_OFFSET_COMPENSATION          true // 是否使用IMU零偏补偿，true为使用，false为不使用
#define IS_USE_ESKF_PREDICTION                  true // 是否使用ESKF预测，true为使用，false为不使用, 
                                                      //只要你有脑子就会知道不预测的话，下面的修正也不能用 
#define IS_USE_ESKF_CORRECTION                  true // 是否使用ESKF误差状态修正，true为使用，false为不使用