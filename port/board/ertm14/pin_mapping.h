/*
 *   openMMC -- Open Source modular IPM Controller firmware
 *
 *   Copyright (C) 2016  Henrique Silva <henrique.silva@lnls.br>
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
 * @defgroup AFC_V3_1 AFCv3.1 Board Port
 * @ingroup BOARD_PORTS
 */

/**
 * @file afc-bpm/v3_1/pin_mapping.h
 * @brief Hardware pin definitions for AFCv3.1
 *
 * @ingroup AFC_V3_1_PIN_MAPPING
 */

/**
 * @defgroup AFC_V3_1_PIN_MAPPING AFCv3.1 Pin Mapping
 * @ingroup AFC_V3_1
 * @{
 */

#ifndef PIN_MAPPING_H_
#define PIN_MAPPING_H_

#include "../../../../modules/pin_cfg.h"

/* UART Interfaces */
#define UART_DEBUG      1

/* Board LEDs */
#define GPIO_LEDGREEN                   PIN_DEF( PORTA, 12, (GPIO_PuPd_DOWN), GPIO_Mode_OUT )
#define GPIO_LEDRED                     PIN_DEF( PORTA, 11, (GPIO_PuPd_DOWN), GPIO_Mode_OUT )


/* Pin initialization (config) list */
#define PIN_CFG_LIST                            \
        GPIO_LEDGREEN,                          \
        GPIO_LEDRED,                            

#endif

/**
 * @}
 */
