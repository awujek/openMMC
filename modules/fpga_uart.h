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
/**
 * @defgroup FPGA_UART FPGA_UART - communication with FPGA via UART
 *
 */

#ifndef FPGA_UART_H_
#define FPGA_UART_H_

#define FPGA_UPDATE_RATE        5000    // in ms
#define FPGA_UART               3

/**
 * @brief FPGA communication Task
 *
 * @param Parameters Pointer to parameters passed to the task upon initialization.
 */
void vTaskFPGA_COMM( void * Parameters );

/**
 * @brief Initializes the FPGA communication Task
 *
 */
void fpga_uart_init( void );

#endif
