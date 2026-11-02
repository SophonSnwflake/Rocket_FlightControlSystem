#include "mid_ins.hpp"

namespace INS{

ESKF_INS::ESKF_INS(IMU *imu, INSMechanization *mech) :
    m_imu(imu),
    m_mech(mech)
{}

INS_Error ESKF_INS::init(){

}

INS_Error ESKF_INS::update(){
    if(!m_imu->isInited()){
        return INS_Error::IMUNotInited;
    }
    
}

}