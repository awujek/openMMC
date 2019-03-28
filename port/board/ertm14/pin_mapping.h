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
    GPIO_MAX
};

extern struct pin_def gpio_pins_def[GPIO_MAX];

/* UART Interfaces */
#define UART_DEBUG      1

#endif

/**
 * @}
 */
