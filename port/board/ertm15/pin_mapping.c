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
 * @defgroup ERTM15 eRTM15 Board Port
 * @ingroup BOARD_PORTS
 */

/**
 * @file ertm15/pin_mapping.h
 * @brief Hardware pin definitions for eRTM15
 *
 * @ingroup ERTM15_PIN_MAPPING
 */

/**
 * @defgroup ERTM15_PIN_MAPPING eRTM15 Pin Mapping
 * @ingroup ERTM15
 * @{
 */

#include "port.h"
#include "pin_mapping.h"


struct pin_def gpio_pins_def[GPIO_MAX] = 
{
    [GPIO_LEDS_UCLK]         = {PORTB,  0, GPIO_Mode_OUT, GPIO_PuPd_DOWN},
    [GPIO_LEDS_SCLK]         = {PORTB,  1, GPIO_Mode_OUT, GPIO_PuPd_DOWN},
    [GPIO_LEDS_SER]          = {PORTB,  2, GPIO_Mode_OUT, GPIO_PuPd_DOWN},

    [GPIO_P1V8_PG]           = {PORTB, 13, GPIO_Mode_IN,  GPIO_PuPd_DOWN},
    [GPIO_P3V3_PG]           = {PORTB, 15, GPIO_Mode_IN,  GPIO_PuPd_DOWN},
    [GPIO_P3V3_PLL_PG]       = {PORTB, 12, GPIO_Mode_IN,  GPIO_PuPd_DOWN},
    [GPIO_P3V6_PG]           = {PORTB, 11, GPIO_Mode_IN,  GPIO_PuPd_DOWN},
    [GPIO_P5V0_PG]           = {PORTB, 14, GPIO_Mode_IN,  GPIO_PuPd_DOWN},

    [GPIO_P3V3_DIV]          = {PORTC,  0, GPIO_Mode_IN,  GPIO_PuPd_NOPULL},
    [GPIO_P9V0_LO_DIV]       = {PORTC,  2, GPIO_Mode_IN,  GPIO_PuPd_NOPULL},
    [GPIO_P9V0_REF_DIV]      = {PORTC,  3, GPIO_Mode_IN,  GPIO_PuPd_NOPULL},
    [GPIO_P12V_DIV]          = {PORTA, 12, GPIO_Mode_IN,  GPIO_PuPd_NOPULL},
    [GPIO_POCXO_DIV]         = {PORTC,  1, GPIO_Mode_IN,  GPIO_PuPd_NOPULL},

    [GPIO_PVADJ_OCXO_EN]     = {PORTA,  5, GPIO_Mode_OUT, GPIO_PuPd_DOWN},
    [GPIO_OCXO_CURR]         = {PORTC,  4, GPIO_Mode_OUT, GPIO_PuPd_DOWN}, /* ? */
    [GPIO_RESET_LOGIC]       = {PORTB,  5, GPIO_Mode_OUT, GPIO_PuPd_DOWN},

    [GPIO_DCDC_EN_UCLK]      = {PORTB,  8, GPIO_Mode_OUT, GPIO_PuPd_DOWN},
    [GPIO_DCDC_EN_SCLK]      = {PORTB,  9, GPIO_Mode_OUT, GPIO_PuPd_DOWN},
    [GPIO_DCDC_EN_SER]       = {PORTB, 10, GPIO_Mode_OUT, GPIO_PuPd_DOWN},

    [GPIO_DDS_REF_POWERDOWN] = {PORTA,  9, GPIO_Mode_OUT, GPIO_PuPd_DOWN},
    [GPIO_DDS_LO_POWERDOWN]  = {PORTA, 10, GPIO_Mode_OUT, GPIO_PuPd_DOWN},
    [GPIO_CLKA_POWERDOWN]    = {PORTA, 11, GPIO_Mode_OUT, GPIO_PuPd_DOWN},
    [GPIO_CLKB_POWERDOWN]    = {PORTA, 12, GPIO_Mode_OUT, GPIO_PuPd_DOWN},

    [GPIO_BP_PWR_ON]         = {PORTC,  7, GPIO_Mode_OUT, GPIO_PuPd_NOPULL},
    /* Entry below should be up, but to allow running without crate make it down */
    [GPIO_BP_ENABLE_N]       = {PORTC,  6, GPIO_Mode_IN,  GPIO_PuPd_UP},

    /* For GPIO_GA* pull direction is overwriten in get_ipmb_addr() */
    [GPIO_GA0]               = {PORTC, 13, GPIO_Mode_IN,  GPIO_PuPd_NOPULL},
    [GPIO_GA1]               = {PORTC, 14, GPIO_Mode_IN,  GPIO_PuPd_NOPULL},
    [GPIO_GA2]               = {PORTC, 15, GPIO_Mode_IN,  GPIO_PuPd_NOPULL},
};

