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

uint16_t pin_addr[] = {
    [0] = GPIO_Pin_0,
    [1] = GPIO_Pin_1,
    [2] = GPIO_Pin_2,
    [3] = GPIO_Pin_3,
    [4] = GPIO_Pin_4,
    [5] = GPIO_Pin_5,
    [6] = GPIO_Pin_6,
    [7] = GPIO_Pin_7,
    [8] = GPIO_Pin_8,
    [9] = GPIO_Pin_9,
    [10] = GPIO_Pin_10,
    [11] = GPIO_Pin_11,
    [12] = GPIO_Pin_12,
    [13] = GPIO_Pin_13,
    [14] = GPIO_Pin_14,
    [15] = GPIO_Pin_15
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

void gpio_set_pin_dir(uint8_t port, uint8_t pin, uint8_t dir)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /*printf("stm32_gpio_configure pin_id %d port %d (0x%x) pin %d (0x%x) mode %d pull %d\n\r",
	   port,
	   gpio_pins_def[port].port,
	   STM_PORT(port),
	   gpio_pins_def[port].pin,
	   STM_PIN(port),
	   gpio_pins_def[port].mode,
	   gpio_pins_def[port].pupd);*/

    GPIO_InitStructure.GPIO_Pin = STM_PIN(port);
    GPIO_InitStructure.GPIO_Mode = gpio_pins_def[port].mode;
    GPIO_InitStructure.GPIO_PuPd = gpio_pins_def[port].pupd;
    GPIO_Init(STM_PORT(port), &GPIO_InitStructure);
}
