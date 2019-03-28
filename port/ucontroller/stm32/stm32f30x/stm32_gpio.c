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
 * @brief GPIO functions redirection for STM32
 *
 * @author Adam Wujek  <adam.wujek@cern.ch>, CERN
 */

#include "port.h"
#include "pin_cfg.h"

GPIO_TypeDef* gpio_addr[] = {
    [PORTA] = GPIOA,
    [PORTB] = GPIOB,
    [PORTC] = GPIOC,
    [PORTD] = GPIOD
};

/**
 * @brief       Initialize GPIO block
 * @return      Nothing
 */
void gpio_init(void)
{

}

/**
 * @brief       Toggle an individual GPIO output pin to the opposite state
 * @param       port    : GPIO Port number where the pin is located
 * @param       pin     : pin number
 * @note        This commands only applies for pins selected as outputs. Writing '0' shouldn't affect the pin state
 */
void gpio_pin_toggle(uint8_t port, uint8_t pin)
{
    int val = !gpio_read_pin(port, pin);
    gpio_set_pin_state(port, pin, val);
}

/**
 * @brief       Set a GPIO pin direction
 * @param       port    : GPIO Port number where pin is located
 * @param       pin     : pin number
 * @param       dir     : true (1) for OUTPUT, false (0) for INPUT
 */

void gpio_set_pin_dir(uint8_t port, uint8_t pin, uint8_t func, uint8_t  dir)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_TypeDef* stm_port;

  //printf("stm32_gpio_configure port %p pin %08x\n", port, pin->pin );

    GPIO_InitStructure.GPIO_Pin = pin; // that's  (PC0 on STM32)
    GPIO_InitStructure.GPIO_Mode = dir;
    GPIO_InitStructure.GPIO_PuPd = func;
    GPIO_Init(gpio_addr[port], &GPIO_InitStructure);
}
