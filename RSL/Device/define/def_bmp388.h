#ifndef __DEF_H
#define __DEF_H

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_READ_LENGTH             32U
#define SPI_DUMMY_BYTES             1U
#define MAX_WRITE_LENGTH            32U

/**\name Register Address */
#define REG_CHIP_ID                        UINT8_C(0x00)
#define REG_ERR                            UINT8_C(0x02)
#define REG_SENS_STATUS                    UINT8_C(0x03)
#define REG_DATA                           UINT8_C(0x04)
#define REG_EVENT                          UINT8_C(0x10)
#define REG_INT_STATUS                     UINT8_C(0x11)
#define REG_FIFO_LENGTH                    UINT8_C(0x12)
#define REG_FIFO_DATA                      UINT8_C(0x14)
#define REG_FIFO_WM                        UINT8_C(0x15)
#define REG_FIFO_CONFIG_1                  UINT8_C(0x17)
#define REG_FIFO_CONFIG_2                  UINT8_C(0x18)
#define REG_INT_CTRL                       UINT8_C(0x19)
#define REG_IF_CONF                        UINT8_C(0x1A)
#define REG_PWR_CTRL                       UINT8_C(0x1B)
#define REG_OSR                            UINT8_C(0X1C)
#define REG_ODR                            UINT8_C(0x1D)
#define REG_CONFIG                         UINT8_C(0x1F)
#define REG_CALIB_DATA                     UINT8_C(0x31)
#define REG_CMD                            UINT8_C(0x7E)

/**\name Soft reset command */
#define SOFT_RESET                         UINT8_C(0xB6)

/**\name Macros related to size */
#define LEN_CALIB_DATA                     UINT8_C(21)
#define LEN_P_AND_T_HEADER_DATA            UINT8_C(7)
#define LEN_P_OR_T_HEADER_DATA             UINT8_C(4)
#define LEN_P_T_DATA                       UINT8_C(6)
#define LEN_GEN_SETT                       UINT8_C(7)
#define LEN_P_DATA                         UINT8_C(3)
#define LEN_T_DATA                         UINT8_C(3)
#define LEN_SENSOR_TIME                    UINT8_C(3)
#define FIFO_MAX_FRAMES                    UINT8_C(73)


// PWR_CTRL 寄存器
#define PRESS_ENABLE_MASK     0x01U
#define TEMP_ENABLE_MASK      0x02U

// OSR 寄存器
#define PRESS_OSR_MASK        0x07U  // bit 2:0
#define TEMP_OSR_MASK         0x38U  // bit 5:3
#define TEMP_OSR_POS          3U

// ODR 寄存器
#define ODR_MASK              0x1FU  // bit 4:0

// CONFIG 寄存器
#define IIR_FILTER_MASK       0x0EU  // bit 3:1
#define IIR_FILTER_POS        1U

// PWR_CTRL 工作模式字段
#define OP_MODE_MASK                    0x30U  // bit 5:4
#define OP_MODE_POS                     4U

// ERR 寄存器中的配置错误位
#define ERR_CONF_MASK                   0x04U  // bit 2

// 从非睡眠模式切换到睡眠模式后的等待时间
#define MODE_TRANSITION_DELAY_MS        5U

// BMP388 测量时间参数，单位：微秒
#define MEAS_OVERHEAD_US          234U
#define PRESS_SETTLE_TIME_US      392U
#define TEMP_SETTLE_TIME_US       313U
#define ADC_CONVERSION_TIME_US    2000U

// 配置字段最大编码
#define MAX_OVERSAMPLING_CODE     0x05U
#define MAX_ODR_CODE              0x11U
#define MAX_IIR_FILTER_CODE       0x07U

/**\name Status macros */
#define CMD_RDY                            UINT8_C(0x10)
#define DRDY_PRESS                         UINT8_C(0x20)
#define DRDY_TEMP                          UINT8_C(0x40)

#define CHIP_ID_BMP388                     UINT8_C(0x50)

#define ERR_CMD                            UINT8_C(0x02)


#ifdef __cplusplus
}
#endif

#endif