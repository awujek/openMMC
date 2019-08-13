/*
 *   openMMC  --
 *
 *   Copyright (C) 2015  Henrique Silva  <henrique.silva@lnls.br>
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
 * @file smt32_uart.h
 * @author Henrique Silva <henrique.silva@lnls.br>, LNLS
 *
 * @brief STM32F30x UART interface definitions
 */

#ifndef STM32_UART_H_
#define STM32_UART_H_

#include "string.h"
#include "stdio.h"

#define STM32_UART_ENABLE_RX 1
#define STM32_UART_ENABLE_TX 2

typedef struct stm32_uart_cfg {
    USART_TypeDef * dev;
    uint32_t periph;
    void (*periph_func)(uint32_t RCC_APBXPeriph, FunctionalState NewState);
    uint32_t periph_port;
    GPIO_TypeDef *port;
    uint8_t pin_mask;
    uint8_t pin_TX;
    uint8_t gpio_af_TX;
    uint8_t pin_RX;
    uint8_t gpio_af_RX;
    GPIO_InitTypeDef GPIO_InitStructure_TX;
    GPIO_InitTypeDef GPIO_InitStructure_RX;
    USART_InitTypeDef USART_InitStructure;
} stm32_uart_cfg_t;

/* Add +1 since they're counted from 1 */
#define UART_MAX_CNT (5+1)

extern stm32_uart_cfg_t usart_cfg[UART_MAX_CNT];

#define uart_set_baud( id, baud ) Chip_UART_SetBaud( usart_cfg[id].ptr, baud )
#define uart_config_data( id, cfg ) Chip_UART_ConfigData( usart_cfg[id].ptr, cfg )
#define uart_tx_enable( id ) Chip_UART_TXEnable( usart_cfg[id].ptr )
#define uart_tx_disable( id ) Chip_UART_TXDisable( usart_cfg[id].ptr )
#define uart_int_enable( id, mask ) Chip_UART_IntEnable( usart_cfg[id].ptr, mask )
#define uart_int_disable( id, mask ) Chip_UART_IntDisable( usart_cfg[id].ptr, mask )
#define uart_send_char( id, ch ) usart_blocking_write(id, ch)
#define uart_read_char( id ) usart_blocking_read( id )
#define uart_send( id, msg, len ) usart_write_buf(id, msg, len )
#define uart_read( id, buf, len ) Chip_UART_ReadBlocking( usart_cfg[id].ptr, buf, len )

void uart_init ( uint8_t id );

void usart_blocking_write(int id, int ch);
int usart_write_buf(int id, int *buf, size_t n);

uint16_t usart_blocking_read(int id);

#endif
