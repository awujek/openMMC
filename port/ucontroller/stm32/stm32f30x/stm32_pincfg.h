/*
 *   openMMC  --
 *
 *   Copyright (C) 2019  CERN
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

/**
 * @file stm32_pincfg.h
 * @brief Pin Config functions redirection for stm32f30x
 *
 * @author Adam Wujek <adam.wujek@cern.ch>, CERN
 */

#ifndef STM32_PINCFG_H_
#define STM32_PINCFG_H_

#include "port.h"

#define PIN_DEF( port, pin, func, dir ) ( (port << 24) | (pin << 16) | (func << 8) | dir )

#define PIN_PORT( pin_def )      ((pin_def & 0xFF000000) >> 24)
#define PIN_NUMBER( pin_def )    ((pin_def & 0x00FF0000) >> 16)
#define PIN_FUNC( pin_def )      ((pin_def & 0x0000FF00) >> 8)
#define PIN_DIR( pin_def )       ((pin_def & 0x000000FF) >> 0)

#define NON_GPIO 0xFF

/* For other mcus like Atmel's it should be PORTA, PORTB, etc */
#define PORTA 0
#define PORTB 1
#define PORTC 2
#define PORTD 3
#define PORTE 4
#define PORTF 5

/**
 * @brief       Sets I/O Control pin mux
 * @param       port    : GPIO port to mux
 * @param       pin     : GPIO pin to mux
 * @param       cfg     : Configuration bits to select pin mode/function
 */
void pin_config(void *cfg);

#endif
