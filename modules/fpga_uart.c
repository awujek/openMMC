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
 * @file fpga_uart.c
 * @author Adam Wujek  <adam.wujek@cern.ch>, CERN
 *
 * @brief Interface functions for FPGA via uart
 *
 * @ingroup FPGA_UART
 */

#include "FreeRTOS.h"
#include "task.h"
#include "port.h"

#include "fpga_uart.h"
#include "task_priorities.h"
#include "sdr.h"

#define LONG_TIME 0xffff
#define OVERRUN_UART  1
#define OVERRUN_RBUFF 2

uint16_t recv_char = 'v';
#define RX_RBUFF_SIZE	33
static uint8_t rx_rbuff[RX_RBUFF_SIZE];
static uint8_t rx_rbuff_head = 0;
static uint8_t rx_rbuff_tail = 0;
static uint8_t uart3_overrun = 0;
static volatile uint8_t *uart3_buff_curr;
static volatile uint8_t *uart3_buff_end;
SemaphoreHandle_t uart3_tx_mutex;
SemaphoreHandle_t uart3_rx_mutex;

uint8_t uart_irq_recv(int id);

void rx_rbuff_inc_p(uint8_t *old)
{
    (*old)++;
    *old = *old % RX_RBUFF_SIZE;
}

void rx_buff_put(uint8_t item) {
    rx_rbuff[rx_rbuff_head] = item;
    rx_rbuff_inc_p(&rx_rbuff_head);
    if (rx_rbuff_head == rx_rbuff_tail) {
	uart3_overrun |= OVERRUN_RBUFF;
	rx_rbuff_inc_p(&rx_rbuff_tail);
    }
}

uint8_t rx_buff_get(void) {
    uint8_t item;
    while (rx_rbuff_head == rx_rbuff_tail) {
	/* buffer empty, wait for something */
	while( xSemaphoreTake( uart3_rx_mutex, LONG_TIME ) != pdTRUE ) {
	    /* wait for RX to complete */
	}
    }

    item = rx_rbuff[rx_rbuff_tail];
    rx_rbuff_inc_p(&rx_rbuff_tail);
    return item;
}


void USART3_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* ------------------ USART in mode Tramitter ------------------------------*/
    if (USART_GetITStatus(USART3, USART_IT_TXE) == SET) {
	printf("T");
	if (uart3_buff_curr != uart3_buff_end)
	    USART_SendData(usart_cfg[3].dev, (uint8_t)*(uart3_buff_curr++));
	else {
	    /* disable TX IRQ */
	    USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
	    /* Release semaphore */
	    xSemaphoreGiveFromISR( uart3_tx_mutex, &xHigherPriorityTaskWoken );
	}
}
    if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET) {
	xSemaphoreGiveFromISR( uart3_rx_mutex, &xHigherPriorityTaskWoken );
	recv_char = USART_ReceiveData(usart_cfg[3].dev);
	rx_buff_put(recv_char);
	printf("R");
    }
    if (USART_GetITStatus(USART3, USART_IT_ORE) == SET) {
	printf("RX overrrun!!!!!!!!\n");
	uart3_overrun |= OVERRUN_UART;
	USART_ClearFlag(USART3, USART_FLAG_ORE);
    }
    /* If xHigherPriorityTaskWoken was set to true you
    we should yield.  The actual macro used here is
    port specific. */
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/* blocking send using irqs */
void uart_irq_send(int id, uint8_t *buff, int len)
{
    uart3_buff_curr = buff;
    uart3_buff_end = buff + len;
    USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
    while( xSemaphoreTake( uart3_tx_mutex, LONG_TIME ) != pdTRUE ) {
	    /* wait for TX to complete */
	}
}

/* blocking send using irqs */
uint8_t uart_irq_recv(int id)
{
    return rx_buff_get();
}


//   USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);

void USART3_IRQConfig(void) {
  NVIC_InitTypeDef NVIC_InitStructure;
  /* NVIC configuration */
  /* Configure the Priority Group to 4 bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

  /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0xf;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/* Send board data to the FPGA RAM via SPI periodically */
void vTaskFPGA_COMM( void * Parameters )
{
    const TickType_t xFrequency = FPGA_UPDATE_RATE;
    TickType_t xLastWakeTime = xTaskGetTickCount();


//     /* Check if the FPGA has finished programming itself from the FLASH */
//     while (!gpio_read_pin( PIN_PORT(GPIO_FPGA_DONE_B), PIN_NUMBER(GPIO_FPGA_DONE_B))) {
//         vTaskDelay(FPGA_UPDATE_RATE);
//     }

    vTaskDelay(1000);
    xLastWakeTime =+ 1000;
    /* int for uart? */
    for ( ;; ) {
        /* Update diagnostic struct information */
// 	uart_send_char(FPGA_UART, recv_char);
	vTaskDelay(1000);
// 	recv_char = uart_read_char(FPGA_UART);
	char *aaa = "test";
	uart_irq_send(3, (uint8_t*)aaa, strlen(aaa));
// 	printf("aaaaaaaaaaaaaaaaa |%d|\n", recv_char);
// 	recv_char++;
// 	recv_char&=0xff;
	vTaskDelayUntil( &xLastWakeTime, xFrequency );
    }
}

/* Send board data to the FPGA RAM via SPI periodically */
void vTaskFPGA_COMM_read( void * Parameters )
{
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    vTaskDelay(2000);
    /* int for uart? */
    for ( ;; ) {
        /* Update diagnostic struct information */
// 	uart_send_char(FPGA_UART, recv_char);
	vTaskDelay(10);
// 	recv_char = uart_read_char(FPGA_UART);
	printf("%c", uart_irq_recv(3));
	if (uart3_overrun) {
	    printf("RX overrrun!!!!!!!! %d\n", uart3_overrun);
	    uart3_overrun = 0;
	}
    }
}


void fpga_uart_init( void )
{
    uart3_tx_mutex = xSemaphoreCreateBinary();
    uart3_rx_mutex = xSemaphoreCreateBinary();
    uart_init(FPGA_UART);
    USART3_IRQConfig();
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    xTaskCreate(vTaskFPGA_COMM,      "FPGA_COMM",      150, NULL, tskFPGA_COMM_PRIORITY, (TaskHandle_t *) NULL);
    xTaskCreate(vTaskFPGA_COMM_read, "FPGA_COMM_read", 150, NULL, tskFPGA_COMM_PRIORITY, (TaskHandle_t *) NULL);
}

#include "ipmi.h"
/* IPMI Handlers */
IPMI_HANDLER(ipmi_custom_snmp, NETFN_CUSTOM, 0, ipmi_msg * req, ipmi_msg * rsp )
{
    uint8_t len = rsp->data_len = 0;

//     uint8_t id = req->data[0];

    printf("%s!! req->data_len %d\n",__func__, req->data_len);
    uart_irq_send(3, req->data, req->data_len);
    rsp->data[len++] = 'a';
    rsp->data[len++] = 'd';
    rsp->data[len++] = 'a';
    rsp->data[len++] = 'm';

    rsp->completion_code = IPMI_CC_OK;

    rsp->data_len = len;
}
