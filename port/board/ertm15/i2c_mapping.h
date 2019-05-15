#ifndef I2C_MAPPING_H_
#define I2C_MAPPING_H_

#include "i2c.h"

#define I2CMODE_POOLING         1
#define I2CMODE_INTERRUPT       0
#define SPEED_100KHZ            100000

// BUS_ID
// 0 - FMC1
// 1 - FMC2
// 3 - CPU_ID
//
///////////////////////

enum {
    I2C_BUS_UNKNOWN_ID = 0x00,
    I2C_BUS_MAC_ID,
    I2C_BUS_IPMI_ID,
    I2C_BUS_TEMP1_ID,
    I2C_BUS_TEMP2_ID,
    I2C_BUS_CNT
};

enum {
    CHIP_ID_TMP100_PS,
    CHIP_ID_TMP100_LO_RF,
    CHIP_ID_TMP100_DDS_LO,
    CHIP_ID_TMP100_LTC6150,
    CHIP_ID_TMP100_REF_RF,
    CHIP_ID_TMP100_DDS_REF,
    CHIP_ID_TMP100_OXCO1,
    CHIP_ID_TMP100_OXCO2,
    CHIP_ID_TMP100_CKLA_FANOUT,
    CHIP_ID_TMP100_CKLB_FANOUT,
    CHIP_ID_MAC,
    I2C_CHIP_CNT
};

typedef enum {
	I2C1_ID,			/**< ID I2C1 */
	I2C2_ID,			/**< ID I2C2 */
	I2C3_ID,			/**< ID I2C3 */
	I2C_NUM_INTERFACE/**< Number of I2C interfaces in the chip */
} I2C_ID_T ;

#define I2C_MUX_CNT    2

i2c_mux_state_t i2c_mux[I2C_MUX_CNT];
i2c_bus_mapping_t i2c_bus_map[I2C_BUS_CNT];
i2c_chip_mapping_t i2c_chip_map[I2C_CHIP_CNT];

#endif
