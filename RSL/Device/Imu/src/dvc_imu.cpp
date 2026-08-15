#include "dvc_imu.hpp"
#include "drv_time.h"

#define SPI_TRY_TIMES 3

IMU::Vector3f IMU::solveAttitude()
{
    if(!readRawData()) return m_ahrs->getEulerAngle();
    dataCalibration();
    m_ahrs->update(m_gyroData, m_accelData, m_magnetData);
    return m_ahrs->getEulerAngle();
}

IMU::IMU(AHRS *ahrs) : 
    m_ahrs(ahrs), m_gyroRawData(), m_accelRawData(), m_magnetRawData() {}

BMI088::BMI088(AHRS *ahrs, SPIConfig accelSPIconfig,SPIConfig gyroSPIconfig,CalibrationInfo calibrationInfo, ErrorCallback errorCallback) :
    IMU(ahrs), m_accelSPIConfig(accelSPIconfig),m_gyroSPIConfig(gyroSPIconfig),m_calibrationInfo(calibrationInfo), m_errorCallback(errorCallback)
{
    m_errorCode = BMI088_NO_ERROR;
    m_Inited = false;
    m_tempDivider = 0;
} 

bool BMI088::init(){
    if(m_Inited == true) return true;
    m_errorCode = BMI088_NO_ERROR;
    if (selfTestAccel()){
        initAccel();
    }
    else{
        handleError(BMI088_SELF_TEST_ACCEL_ERROR);
        return false;
    }

    if (selfTestGyro()){  
        initGyro();
    }
    else{
        handleError(BMI088_SELF_TEST_GYRO_ERROR);
        return false;
    }

    m_Inited = true;

    return true;
}

void BMI088::dataCalibration()
{
    m_gyroData   = m_calibrationInfo.installSpinMatrix * (m_gyroRawData - m_calibrationInfo.gyroOffset);
    m_accelData  = m_calibrationInfo.installSpinMatrix * (m_accelRawData - m_calibrationInfo.accelOffset);
    m_magnetData = m_calibrationInfo.installSpinMatrix * (m_magnetRawData - m_calibrationInfo.magnetOffset);
}

void BMI088::handleError(BMI088::ErrorCode errorCode)
{
    m_errorCode = errorCode;
    if (m_errorCode != BMI088_NO_ERROR && m_errorCallback != nullptr)
        m_errorCallback(errorCode);
}

bool BMI088::readRawData()
{
    uint8_t buf[8] = {0};
    int16_t raw;                       // 原名 bmi088_raw_temp 有歧义,它不只装温度

    // ---- 加速度计 ----
    if (!readMutipleReg(m_accelSPIConfig, BMI088_ACCEL_XOUT_L, buf, 6)) return false;

    raw = (int16_t)((buf[1] << 8) | buf[0]);
    m_accelCounts[0]  = raw;
    m_accelRawData[0] = raw * ACCEL_SEN;
    raw = (int16_t)((buf[3] << 8) | buf[2]);
    m_accelCounts[1]  = raw;
    m_accelRawData[1] = raw * ACCEL_SEN;
    raw = (int16_t)((buf[5] << 8) | buf[4]);
    m_accelCounts[2]  = raw;
    m_accelRawData[2] = raw * ACCEL_SEN;

    // ---- 陀螺仪 ----
    if (!readMutipleReg(m_gyroSPIConfig, BMI088_GYRO_CHIP_ID, buf, 8)) return false;
    if (buf[0] != BMI088_GYRO_CHIP_ID_VALUE) return false;

    raw = (int16_t)((buf[3] << 8) | buf[2]);
    m_gyroCounts[0]  = raw;
    m_gyroRawData[0] = raw * GYRO_SEN;
    raw = (int16_t)((buf[5] << 8) | buf[4]);
    m_gyroCounts[1]  = raw;
    m_gyroRawData[1] = raw * GYRO_SEN;
    raw = (int16_t)((buf[7] << 8) | buf[6]);
    m_gyroCounts[2]  = raw;
    m_gyroRawData[2] = raw * GYRO_SEN;

    // ---- 温度:约 1Hz,读失败沿用旧值,不影响本帧 ----
    if (++m_tempDivider >= 400) {
        m_tempDivider = 0;
        if (readMutipleReg(m_accelSPIConfig, BMI088_TEMP_M, buf, 2)) {
            raw = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
            if (raw > 1023) raw -= 2048;
            m_temperature = raw * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
        }
    }

    return true;
}

bool BMI088::initAccel(){
    uint8_t res =0;
    uint8_t write_reg_num = 0;

    static const uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] =
        {
            {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
            {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
            {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
            {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_ERROR},
            {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
            {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}
        };
    
    //检查通信
    readSingleReg(m_accelSPIConfig, BMI088_ACC_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    readSingleReg(m_accelSPIConfig, BMI088_ACC_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    if(res != BMI088_ACC_CHIP_ID_VALUE){
        handleError(BMI088_NO_SENSOR);
        return false;
    }

    //ACCEL reset
    writeSingleReg(m_accelSPIConfig, BMI088_ACC_SOFTRESET,BMI088_ACC_SOFTRESET_VALUE);
    osDelay(BMI088_LONG_DELAY_TIME);

    //检查通信
    readSingleReg(m_accelSPIConfig, BMI088_ACC_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    readSingleReg(m_accelSPIConfig, BMI088_ACC_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    if(res != BMI088_ACC_CHIP_ID_VALUE){
        handleError(BMI088_NO_SENSOR);
        return false;
    }

    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num ++){
        writeSingleReg(m_accelSPIConfig, write_BMI088_accel_reg_data_error[write_reg_num][0], write_BMI088_accel_reg_data_error[write_reg_num][1]);
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        readSingleReg(m_accelSPIConfig, write_BMI088_accel_reg_data_error[write_reg_num][0], res);
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_accel_reg_data_error[write_reg_num][1]) {
            handleError((ErrorCode)write_BMI088_accel_reg_data_error[write_reg_num][2]);
            return false;
        }
    }
    return true;

}

bool BMI088::initGyro()
{
    uint8_t write_reg_num = 0;
    uint8_t res           = 0;
    static const uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] =
        {
            {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
            {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
            {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
            {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
            {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
            {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}

        };

    // check commiunication
    readSingleReg(m_gyroSPIConfig, BMI088_GYRO_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    readSingleReg(m_gyroSPIConfig, BMI088_GYRO_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    if (res != BMI088_GYRO_CHIP_ID_VALUE) {
        handleError(BMI088_NO_SENSOR);
        return false;
    }

    // reset the gyro sensor
    writeSingleReg(m_gyroSPIConfig, BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    osDelay(BMI088_LONG_DELAY_TIME);
    // check commiunication is normal after reset
    readSingleReg(m_gyroSPIConfig, BMI088_GYRO_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    readSingleReg(m_gyroSPIConfig, BMI088_GYRO_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    if (res != BMI088_GYRO_CHIP_ID_VALUE) {
        handleError(BMI088_NO_SENSOR);
        return false;
    }

    // set gyro sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++) {

        writeSingleReg(m_gyroSPIConfig, write_BMI088_gyro_reg_data_error[write_reg_num][0], write_BMI088_gyro_reg_data_error[write_reg_num][1]);
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        readSingleReg(m_gyroSPIConfig, write_BMI088_gyro_reg_data_error[write_reg_num][0], res);
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_gyro_reg_data_error[write_reg_num][1]) {
            handleError((ErrorCode)write_BMI088_gyro_reg_data_error[write_reg_num][2]);
            return false;
        }
    }
    return true;
}


bool BMI088::selfTestAccel(){
    int16_t self_test_accel[2][3];
    uint8_t buf[6] = {0,0,0,0,0,0};

    uint8_t res = 0;
    uint8_t write_reg_num = 0;

    static const uint8_t BMI088_SELF_TEST_ACCEL_Reg_Data_Error[6][3] = {
            {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_1600_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR },
            {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR },
            {BMI088_ACC_RANGE,BMI088_ACC_RANGE_24G, BMI088_ACC_RANGE_ERROR},
            {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
            {BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_POSITIVE_SIGNAL, BMI088_SELF_TEST_ACCEL_ERROR},
            {BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_NEGATIVE_SIGNAL, BMI088_SELF_TEST_ACCEL_ERROR}
    };


    //ACCEL reset
    writeSingleReg(m_accelSPIConfig, BMI088_ACC_SOFTRESET,BMI088_ACC_SOFTRESET_VALUE);
    osDelay(BMI088_LONG_DELAY_TIME);

    //检查通信
    readSingleReg(m_accelSPIConfig, BMI088_ACC_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    readSingleReg(m_accelSPIConfig, BMI088_ACC_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    if (res != BMI088_ACC_CHIP_ID_VALUE){
        handleError(BMI088_NO_SENSOR);
        return false;
    }

    //重新启动BMI088加速度计
    writeSingleReg(m_accelSPIConfig, BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    osDelay(BMI088_LONG_DELAY_TIME);

    //检查通信
    readSingleReg(m_accelSPIConfig, BMI088_ACC_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    readSingleReg(m_accelSPIConfig, BMI088_ACC_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    if (res != BMI088_ACC_CHIP_ID_VALUE){
        handleError(BMI088_NO_SENSOR);
        return false;
    }

    for (write_reg_num = 0;write_reg_num < 4; write_reg_num ++){
        writeSingleReg(m_accelSPIConfig,BMI088_SELF_TEST_ACCEL_Reg_Data_Error[write_reg_num][0], BMI088_SELF_TEST_ACCEL_Reg_Data_Error[write_reg_num][1]);
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        readSingleReg(m_accelSPIConfig,BMI088_SELF_TEST_ACCEL_Reg_Data_Error[write_reg_num][0], res);
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != BMI088_SELF_TEST_ACCEL_Reg_Data_Error[write_reg_num][1]){
            handleError((ErrorCode)BMI088_SELF_TEST_ACCEL_Reg_Data_Error[write_reg_num][2]);
            return false;
        }
        osDelay(BMI088_LONG_DELAY_TIME);
    }

    for (write_reg_num = 0;write_reg_num <2; write_reg_num ++){
        writeSingleReg(m_accelSPIConfig, BMI088_SELF_TEST_ACCEL_Reg_Data_Error[write_reg_num + 4][0],BMI088_SELF_TEST_ACCEL_Reg_Data_Error[write_reg_num + 4][1]);
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        readSingleReg(m_accelSPIConfig,BMI088_SELF_TEST_ACCEL_Reg_Data_Error[write_reg_num + 4][0], res);
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != BMI088_SELF_TEST_ACCEL_Reg_Data_Error[write_reg_num + 4][1]){
            handleError((ErrorCode)BMI088_SELF_TEST_ACCEL_Reg_Data_Error[write_reg_num + 4][2]);
            return false;
        }
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        osDelay(BMI088_LONG_DELAY_TIME); 

        readMutipleReg(m_accelSPIConfig, BMI088_ACCEL_XOUT_L, buf, 6);
        self_test_accel[write_reg_num][0] = (int16_t)((buf[1]) << 8) | buf[0];
        self_test_accel[write_reg_num][1] = (int16_t)((buf[3]) << 8) | buf[2];
        self_test_accel[write_reg_num][2] = (int16_t)((buf[5]) << 8) | buf[4];

    }

    //关闭自检
    writeSingleReg(m_accelSPIConfig, BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_OFF);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    osDelay(BMI088_LONG_DELAY_TIME);
    readSingleReg(m_accelSPIConfig, BMI088_ACC_SELF_TEST, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    if (res != BMI088_ACC_SELF_TEST_OFF){
        handleError(BMI088_ACC_SELF_TEST_ERROR);
        return false;
    }

    //重新启动BMI088加速度计
    writeSingleReg(m_accelSPIConfig, BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    osDelay(BMI088_LONG_DELAY_TIME);

    uint16_t minus1 = self_test_accel[0][0] - self_test_accel[1][0];
    uint16_t minus2 = self_test_accel[0][1] - self_test_accel[1][1];
    uint16_t miuns3 = self_test_accel[0][2] - self_test_accel[1][2];

    if ((self_test_accel[0][0] - self_test_accel[1][0] < 1365) || (self_test_accel[0][1] - self_test_accel[1][1] < 1365) || (self_test_accel[0][2] - self_test_accel[1][2] < 680)) {
        handleError(BMI088_SELF_TEST_ACCEL_ERROR);
        return false;
    }

    readSingleReg(m_accelSPIConfig, BMI088_ACC_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    readSingleReg(m_accelSPIConfig, BMI088_ACC_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    
    return true;
}

bool BMI088::selfTestGyro(){
    uint8_t res;
    uint8_t retry = 0;

    // reset the gyro sensor
    writeSingleReg(m_gyroSPIConfig, BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    osDelay(BMI088_LONG_DELAY_TIME);

    readSingleReg(m_gyroSPIConfig, BMI088_GYRO_CHIP_ID, res);
    osDelay(BMI088_LONG_DELAY_TIME);

    if(res != BMI088_GYRO_CHIP_ID_VALUE){
        handleError(BMI088_NO_SENSOR);
        return false;
    }

    writeSingleReg(m_gyroSPIConfig, BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    osDelay(BMI088_LONG_DELAY_TIME);

    readSingleReg(m_gyroSPIConfig, BMI088_GYRO_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    if(res != BMI088_GYRO_CHIP_ID_VALUE){
        handleError(BMI088_NO_SENSOR);
        return false;
    }

    writeSingleReg(m_gyroSPIConfig, BMI088_GYRO_SELF_TEST, BMI088_GYRO_TRIG_BIST);
    osDelay(BMI088_LONG_DELAY_TIME);

    do{
        readSingleReg(m_gyroSPIConfig, BMI088_GYRO_SELF_TEST, res);
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
        retry ++;
    }while(!(res & BMI088_GYRO_BIST_RDY) && retry < 10);

    if (retry == 10){
        handleError(BMI088_SELF_TEST_GYRO_ERROR);
        return false;
    }

    return true;
}

RSLMath::Vector3f IMU::getGyroBias(){
    return m_ahrs->getGyroBias();
}


inline bool BMI088::readSingleReg(const SPIConfig &SPIconfig, uint8_t reg, uint8_t &prxData){
    reg |= 0x80;
    uint8_t dummy =0xFF;
    SPIGuard guard(SPIconfig, SpiLockTimeoutMs);
    if (!guard.ok()) {
        m_busTimeoutCount++;
        return false;
    }
    SPI_Transmit(SPIconfig.hspi, &reg, 1, 1000);
    if (&SPIconfig == &m_accelSPIConfig){
        if(SPI_Receive(SPIconfig.hspi, &dummy, 1, 1000)!=HAL_OK) return false;
    }
    return SPI_Receive(SPIconfig.hspi, &prxData, 1, 1000) == HAL_OK;
}


bool BMI088::readMutipleReg(const SPIConfig &SPIconfig, uint8_t reg, uint8_t *prxData, uint8_t length){
    reg |= 0x80;
    uint8_t dummy = 0xFF;
    SPIGuard guard(SPIconfig, SpiLockTimeoutMs);
    if (!guard.ok()) {
        m_busTimeoutCount++;
        return false;
    }
    SPI_Transmit(SPIconfig.hspi, &reg, 1, 1000);
    if (&SPIconfig == &m_accelSPIConfig){
        if(SPI_Transmit(SPIconfig.hspi, &dummy, 1, 1000)!=HAL_OK) return false;
    }
    return SPI_Receive(SPIconfig.hspi, prxData, length, 1000) == HAL_OK;
}

bool BMI088::writeSingleReg(const SPIConfig &SPIconfig, uint8_t reg,uint8_t txData){
    SPIGuard guard(SPIconfig, SpiLockTimeoutMs);
    if (!guard.ok()) {
        m_busTimeoutCount++;
        return false;
    }
    if(SPI_Transmit(SPIconfig.hspi, &reg, 1, 1000)!=HAL_OK) return false;
    return SPI_Transmit(SPIconfig.hspi, &txData, 1, 1000) == HAL_OK;
}

