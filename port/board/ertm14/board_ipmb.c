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
 * @file board_ipmb.h
 * @author Adam Wujek <adam.wujek@cern.ch>, CERN
 *
 * @brief Board specific definitions used in IPMB Layer
 */

/* Project includes */
#include "ipmb.h"
#include "port.h"


/*
 *==============================================================
 * MMC ADDRESSING
 *==============================================================
 */

/**
 * @brief Table holding all possible address values in IPMB specification
 * @see get_ipmb_addr()
 */
const unsigned char IPMBL_TABLE[IPMBL_TABLE_SIZE] = {
    0x70, 0x8A, 0x72, 0x8E, 0x92, 0x90, 0x74, 0x8C, 0x76,
    0x98, 0x9C, 0x9A, 0xA0, 0xA4, 0x88, 0x9E, 0x86, 0x84,
    0x78, 0x94, 0x7A, 0x96, 0x82, 0x80, 0x7C, 0x7E, 0xA2 };

uint8_t get_ipmb_addr( void )
{
    uint8_t ga0, ga1, ga2;
    uint8_t index;

    /* to detect if pins are not connected, first pull them down, read status
     * then pull up. If value has changed this mean the pin is floating */
    gpio_pins_def[GPIO_GA0].pupd = GPIO_PuPd_DOWN;
    gpio_pins_def[GPIO_GA1].pupd = GPIO_PuPd_DOWN;
    gpio_pins_def[GPIO_GA2].pupd = GPIO_PuPd_DOWN;
    /* for STM 2nd and 3rd params are ignored */
    gpio_set_pin_dir(PIN_PORT(GPIO_GA0), 0, 0);
    gpio_set_pin_dir(PIN_PORT(GPIO_GA1), 0, 0);
    gpio_set_pin_dir(PIN_PORT(GPIO_GA2), 0, 0);


    ga0 = gpio_read_pin(PIN_PORT(GPIO_GA0), PIN_NUMBER(GPIO_GA0));
    ga1 = gpio_read_pin(PIN_PORT(GPIO_GA1), PIN_NUMBER(GPIO_GA1));
    ga2 = gpio_read_pin(PIN_PORT(GPIO_GA2), PIN_NUMBER(GPIO_GA2));

    gpio_pins_def[GPIO_GA0].pupd = GPIO_PuPd_UP;
    gpio_pins_def[GPIO_GA1].pupd = GPIO_PuPd_UP;
    gpio_pins_def[GPIO_GA2].pupd = GPIO_PuPd_UP;
    /* for STM 2nd and 3rd params are ignored */
    gpio_set_pin_dir(PIN_PORT(GPIO_GA0), 0, 0);
    gpio_set_pin_dir(PIN_PORT(GPIO_GA1), 0, 0);
    gpio_set_pin_dir(PIN_PORT(GPIO_GA2), 0, 0);

    /* TODO: it may be necessary to add some delay before re-reading pin
     * statuses */
    if ( ga0 != gpio_read_pin(PIN_PORT(GPIO_GA0), PIN_NUMBER(GPIO_GA0)) ){
        ga0 = UNCONNECTED;
    }

    if ( ga1 != gpio_read_pin(PIN_PORT(GPIO_GA1), PIN_NUMBER(GPIO_GA1)) ){
        ga1 = UNCONNECTED;
    }

    if ( ga2 != gpio_read_pin(PIN_PORT(GPIO_GA2), PIN_NUMBER(GPIO_GA2)) ){
        ga2 = UNCONNECTED;
    }

    /* Transform the 3-based code in a decimal number */
    index = (9 * ga2) + (3 * ga1) + (1 * ga0);

    if ( index >= IPMBL_TABLE_SIZE ) {
        return 0;
    }
    printf("ipmi address 0x%02x\r\n", IPMBL_TABLE[index]);
    return IPMBL_TABLE[index];
}
