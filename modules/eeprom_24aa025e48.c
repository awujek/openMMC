/*
 *   openMMC -- Open Source modular IPM Controller firmware
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
 *
 *   @license GPL-3.0+ <http://spdx.org/licenses/GPL-3.0+>
 */

/**
 * @file   eeprom_24aa025e48.c
 * @author Adam Wujek <adam.wujek@cern.ch>
 *
 * @brief  24AA025E48 EEPROM module interface implementation
 *
 * @ingroup 24AA025E48
 */

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "string.h"

/* Project Includes */
#include "eeprom_24aa025e48.h"
#include "port.h"
#include "i2c.h"

size_t eeprom_24aa025e48_read(uint8_t id, uint16_t address, uint8_t *rx_data,
			      size_t buf_len, TickType_t timeout)
{
    uint8_t i2c_addr;
    uint8_t i2c_interface;
    uint8_t rx_len = 0;
    uint8_t addr8;

    addr8 = address;

    if ( rx_data == NULL ) {
        return 0;
    }

    if (i2c_take_by_chipid( id, &i2c_addr, &i2c_interface, timeout ) ) {
        /* Sets address register */
        xI2CMasterWriteRead(i2c_interface, i2c_addr, addr8, rx_data, buf_len);

        i2c_give( i2c_interface );
    }

    return rx_len;
}

#define PAGE_SIZE 16
#define ADDR_SIZE 1
size_t eeprom_24aa025e48_write(uint8_t id, uint16_t address,
			       uint8_t *tx_data, size_t buf_len,
			       TickType_t timeout)
{
    uint8_t i2c_addr;
    uint8_t i2c_interface;
    uint8_t bytes_to_write;
    uint8_t page_buf[PAGE_SIZE + ADDR_SIZE];
    uint16_t curr_addr;

    size_t tx_len = 0;

    if ( tx_data == NULL ) {
        return 0;
    }

    printf("%s buf_len %d\n", __func__, buf_len);
    if (i2c_take_by_chipid( id, &i2c_addr, &i2c_interface, timeout)) {
        curr_addr = address;

        while (tx_len < buf_len) {
            bytes_to_write = PAGE_SIZE - (curr_addr % PAGE_SIZE);

            if (bytes_to_write > ( buf_len - tx_len )) {
                bytes_to_write = ( buf_len - tx_len );
            }
            page_buf[0] = curr_addr;

            memcpy(&page_buf[ADDR_SIZE], tx_data + tx_len, bytes_to_write);

            /* Write the data */
	    printf("%s write %d\n", __func__, bytes_to_write + ADDR_SIZE);
            tx_len += xI2CMasterWrite( i2c_interface, i2c_addr, &page_buf[0] , bytes_to_write + ADDR_SIZE);
	    printf("before delay tx_len%d\n", tx_len);
            vTaskDelay(10);
	    printf("after delay tx_len%d\n", tx_len);
            tx_len -= ADDR_SIZE; /* Remove the addr size from the count */
            curr_addr += bytes_to_write;
        }
        i2c_give( i2c_interface );
    }

    return tx_len;
}
