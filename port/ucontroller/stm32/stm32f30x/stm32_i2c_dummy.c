/*
 *   openMMC  --
 *
 *   Copyright (C) 2019 CERN
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*!
 * @file stm_i2c_dummy.c
 * @author Adam Wujek <adam.wujek@cern.ch>, CERN
 * @date April 2019
 *
 * @brief Functions needed to be able to compile ipmb.c
 */

#include "port.h"
#include "string.h"
#include "i2c_mapping.h"
#include "stm32f30x_i2c_cpal.h"
#include "stm32f30x.h"
#include "task.h"
#include "semphr.h"

uint8_t xI2CSlaveReceive( I2C_ID_T id, uint8_t * rx_buff, uint8_t buff_len, uint32_t timeout )
{
    while (1) {
	vTaskDelay(1000);
    }
}


void vI2CSlaveSetup ( I2C_ID_T id, uint8_t slave_addr )
{
}
