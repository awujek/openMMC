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

#include "port.h"
#include "pin_mapping.h"


struct pin_def gpio_pins_def[GPIO_MAX] = 
{
    [GPIO_LEDGREEN] = {PORTA, 12, GPIO_Mode_OUT, GPIO_PuPd_DOWN},
    [GPIO_LEDRED]   = {PORTA, 11, GPIO_Mode_OUT, GPIO_PuPd_DOWN},
    [GPIO_SYNC_LEDGREEN] = {PORTA, 10, GPIO_Mode_OUT, GPIO_PuPd_DOWN},
    [GPIO_SYNC_LEDRED]   = {PORTA, 9, GPIO_Mode_OUT, GPIO_PuPd_DOWN}

};

