#pragma once

#include "alg_ins_types.hpp"
#include "alg_ins_mech.hpp"
#include "alg_ins_eskf.hpp"
#include "alg_ins_gnssModel.hpp"
#include "dvc_gnss.hpp"
#include "dvc_imu.hpp"

namespace INS{

static constexpr fp64 MAX_GNSS_DATA_AGE_S = 1.0; // GNSS数据允许的最大年龄，单位：秒
static constexpr fp64 MAX_IMU_DELTA_TIME_S = 0.1; // 最大IMU数据间隔，超过此值认为IMU数据异常

class ESKF_INS{
private:
    INSMechanization *m_mech;
    INSESKF *m_eskf;
    GNSSMeasurementModel *m_gnssModel;

private:
    IMUData m_previousIMUData{};
    IMUData m_currentIMUData{};
    NavState m_previousNavState{};
    NavState m_currentNavState{};

    uint32_t m_lastGNSSSequence = 0U;

    bool isCurrentIMUDataValid = false;
    bool isINSInitialized = false;
public:
    ESKF_INS(INSMechanization *mec, INSESKF *eskf, GNSSMeasurementModel *gnssModel);
    ~ESKF_INS() = default;

    INS_Error init();
    INS_Error update(IMU::IMUSample &previousSample, IMU::IMUSample &currentSample, GNSS::GNSSSample *gnssSample);
    bool isINSInited() const { return isINSInitialized; }
private:
    INS_Error packetIMUData(const IMU::IMUSample &previousSample, const IMU::IMUSample &currentSample, IMUData &imuData);
    bool isGNSSSampleAvailableForCorrection(const GNSS::GNSSSample *gnssSample) const;
    INS_Error errorFeedback(NavState &nextNavState);
};
}
