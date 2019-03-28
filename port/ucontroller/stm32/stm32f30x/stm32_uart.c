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

/*!
 * @file stm32_uart.c
 * @author Henrique Silva <henrique.silva@lnls.br>, LNLS
 * @date June 2016
 *
 * @brief
 */

#include "port.h"

const stm32_uart_cfg_t usart_cfg[6] = {
    [1] = { USART1, RCC_APB2Periph_USART1, RCC_AHBPeriph_GPIOC, GPIOC, GPIO_PinSource4, GPIO_AF_7 },
//     { USART2 },
//     { USART3 }
};

void uart_init ( uint8_t id )
{
//     Chip_Clock_SetPCLKDiv( usart_cfg[id].sysclk, SYSCTL_CLKDIV_2 );
// 
//     Chip_UART_Init( usart_cfg[id].ptr );
// 
//     /* Standard 19200 baud rate */
//     uart_set_baud( UART_DEBUG, 19200 );
// 
//     /* Defaults to 8N1, no parity */
//     uart_config_data( UART_DEBUG, ( UART_LCR_WLEN8 | UART_LCR_SBS_1BIT | UART_LCR_PARITY_DIS ) );
// 
//     uart_tx_enable ( UART_DEBUG );

  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;

  /* Enable GPIO & USART clock */
  RCC_APB2PeriphClockCmd(usart_cfg[id].periph, ENABLE);
  RCC_AHBPeriphClockCmd(usart_cfg[id].periph_port, ENABLE);

  /* Connect PC4 to USART1_Tx */
  GPIO_PinAFConfig(usart_cfg[id].port, usart_cfg[id].pin_TX, usart_cfg[id].gpio_af_TX);

  /* Configure USART Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(usart_cfg[id].port, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(usart_cfg[id].dev, &USART_InitStructure);

  /* Enable USART */
  USART_Cmd(usart_cfg[id].dev, ENABLE);
//   
    
    
//     GPIO_InitTypeDef GPIO_InitStructure;
//   USART_InitTypeDef USART_InitStructure;
// 
//   /* Enable GPIO & USART clock */
//   RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
//   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
// 
//   /* Connect PC4 to USART1_Tx */
//   GPIO_PinAFConfig(GPIOC, GPIO_PinSource4, GPIO_AF_7);
// 
//   /* Configure USART Tx as alternate function push-pull */
//   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
//   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//   GPIO_Init(GPIOC, &GPIO_InitStructure);
// 
//   USART_InitStructure.USART_BaudRate = 115200;
//   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//   USART_InitStructure.USART_StopBits = USART_StopBits_1;
//   USART_InitStructure.USART_Parity = USART_Parity_No;
//   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
// 
//   USART_Init(USART1, &USART_InitStructure);
// 
//   /* Enable USART */
//   USART_Cmd(USART1, ENABLE);
}

void usart_blocking_write(int id, int ch)
{
  /* Loop until transmit data register is empty */
   while (USART_GetFlagStatus(usart_cfg[id].dev, USART_FLAG_TXE) == RESET)
      
  {
  }

  /* e.g. write a character to the USART */
   USART_SendData(usart_cfg[id].dev, (uint8_t)ch);
//   USART_SendData(USART1, (uint8_t)ch);

  /* Loop until transmit data register is empty */
//   while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
  {
  }
}

int usart_write_buf(int id, int *buf, size_t n)
{
  size_t i;
  for (i = 0; i < n; i++)
    usart_blocking_write(id, buf[i]);
  return n;
}
