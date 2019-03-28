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

#include "port.h"
#include "stm32_pincfg.h"

/**
 * @brief       Sets I/O Control pin mux
 * @param       port    : GPIO port to mux
 * @param       pin     : GPIO pin to mux
 * @param       cfg     : Configuration bits to select pin mode/function
 */
#define PIN_PORT( pin_def )      ((pin_def & 0xFF000000) >> 24)

void pin_config(void *cfg)
//uint8_t port, uint8_t pin, uint8_t func, uint8_t dir)
// void pin_config(int port, int pin, GPIOMode_TypeDef func, GPIOPuPd_TypeDef dir)
{
  //printf("stm32_gpio_configure port %p pin %08x\n", port, pin->pin );
    uint32_t *pin_cfg = cfg;
    uint8_t port = PIN_PORT(*(uint32_t *)cfg);

    if (port == PORTA)
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    else if (port == PORTB)
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    else if (port == PORTC)
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
    else if (port == PORTD)
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);
    else if (port == PORTE)
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, ENABLE);

    if ( PIN_DIR(*pin_cfg) != NON_GPIO ) {
        /* Config GPIO direction */
        gpio_set_pin_dir( PIN_PORT(*pin_cfg), PIN_NUMBER(*pin_cfg), PIN_FUNC(*pin_cfg), PIN_DIR(*pin_cfg));
    }
}

