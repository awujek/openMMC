#include "i2c.h"
#include "i2c_mapping.h"
#include "port.h"

i2c_mux_state_t i2c_mux[I2C_MUX_CNT] = {
    { I2C1_ID, -1, 0 },
    { I2C3_ID, -1, 0 }
};

i2c_bus_mapping_t i2c_bus_map[I2C_BUS_CNT] = {
    [I2C_BUS_MAC_ID]  = { I2C1_ID, 0, 1 },
    [I2C_BUS_IPMI_ID] = { I2C2_ID, 0, 1 },
    [I2C_BUS_TEMP_ID] = { I2C3_ID, 0, 1 },
};

i2c_chip_mapping_t i2c_chip_map[I2C_CHIP_CNT] = {
    [CHIP_ID_TMP100_FPGA] = { I2C_BUS_TEMP_ID, 0x90 },
    [CHIP_ID_TMP100_DCDC] = { I2C_BUS_TEMP_ID, 0x92 },
    [CHIP_ID_MAC]         = { I2C_BUS_MAC_ID,  0xA0 },
};

bool i2c_set_mux_bus( uint8_t bus_id, i2c_mux_state_t *i2c_mux, int8_t new_state )
{
    i2c_mux->state = new_state;
    return true;
}

uint8_t i2c_get_mux_bus( uint8_t bus_id, i2c_mux_state_t *i2c_mux )
{
    return i2c_mux->state;
}
