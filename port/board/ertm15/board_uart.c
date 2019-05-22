/*
 *   openMMC  --
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
 */

/*!
 * @file stm32_uart.c
 * @author Adam Wujek, <adam.wujek@cern.ch>
 * @date March 2019
 *
 * @brief
 */

#include "port.h"

/* UART configuration is a board specific */
stm32_uart_cfg_t usart_cfg[UART_MAX_CNT] = {
    /* Use TX of uart5 on portC12 */
    [5] = { UART5, RCC_APB1Periph_UART5, &RCC_APB1PeriphClockCmd, RCC_AHBPeriph_GPIOC, GPIOC, GPIO_PinSource12, GPIO_AF_5,
		{ /* GPIO_InitStructure */
		.GPIO_Pin = GPIO_Pin_12,
		.GPIO_Mode = GPIO_Mode_AF,
		.GPIO_Speed = GPIO_Speed_50MHz,
		.GPIO_OType = GPIO_OType_PP,
		.GPIO_PuPd = GPIO_PuPd_UP,
		},
		{ /* USART_InitStructure */
		.USART_BaudRate = 115200,
		.USART_WordLength = USART_WordLength_8b,
		.USART_StopBits = USART_StopBits_1,
		.USART_Parity = USART_Parity_No,
		.USART_HardwareFlowControl = USART_HardwareFlowControl_None,
		.USART_Mode = USART_Mode_Tx,
		}
	},
};
