#pragma once

#include "alg_ins_types.hpp"
#include "alg_ins_mech.hpp"
#include "dvc_imu.hpp"

namespace INS{
enum class INS_Error{
    OK = 0,
    IMUNotInited
};

class ESKF_INS{
private:
    IMU *m_imu;
    INSMechanization *m_mech;
public:
    ESKF_INS(IMU *imu, INSMechanization *mech);
    ~ESKF_INS() = default;

    INS_Error init();
    INS_Error update();
};
}