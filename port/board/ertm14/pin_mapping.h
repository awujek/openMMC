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
 * @defgroup ERTM14 eRTM14 Board Port
 * @ingroup BOARD_PORTS
 */

/**
 * @file ertm14/pin_mapping.h
 * @brief Hardware pin definitions for eRTM14
 *
 * @ingroup ERTM14_PIN_MAPPING
 */

/**
 * @defgroup ERTM14_PIN_MAPPING eRTM14 Pin Mapping
 * @ingroup ERTM14
 * @{
 */

#ifndef PIN_MAPPING_H_
#define PIN_MAPPING_H_

enum pin_enum {
    GPIO_LEDGREEN = 0,
    GPIO_LEDRED = 1,
    GPIO_SYNC_LEDGREEN,
    GPIO_SYNC_LEDRED,
    GPIO_P1V0_EN,
    GPIO_P1V0_PG,
    GPIO_P1V6_EN,
    GPIO_P1V6_PG,
    GPIO_P1V8_EN,
    GPIO_P1V8_PG,
    GPIO_P2V5_EN,
    GPIO_P2V5_PG,
    GPIO_P3V3_EN,
    GPIO_P3V3_PG,
    GPIO_P3V8_EN,
    GPIO_P3V8_PG,
    GPIO_PS_GTX_EN,
    GPIO_PS_GTX_PG,
    GPIO_FPGA_RESET,
    GPIO_FPGA_CONFIG_DONE,
    GPIO_FPGA_CRC_ERROR,
    GPIO_BP_PWR_ON,
    GPIO_BP_ENABLE_N,
    GPIO_P3V3_DIV,
    GPIO_P12V_DIV,
    GPIO_GA0,
    GPIO_GA1,
    GPIO_GA2,
    GPIO_MAX
};

extern struct pin_def gpio_pins_def[GPIO_MAX];

/* UART Interfaces */
#define UART_DEBUG      1

#endif

/**
 * @}
 */
