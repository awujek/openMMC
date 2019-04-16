/*
 *   openMMC -- Open Source modular IPM Controller firmware
 *
 *   Copyright (C) 2015-2016  Henrique Silva <henrique.silva@lnls.br>
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
 * @defgroup TMP100 TMP100 - Temperature Sensor
 * @ingroup SENSORS
 *
 * The TMP100 is a temperature sensor with I2C interface.
 * The host can query the TMP100 at any time to read temperature. <br>
 */

/**
 * @file tmp100.h
 * @author Adam Wujek  <adam.wujek@cern.ch>, CERN
 *
 * @brief Definitions for TMP100 I2C Temperature Sensor
 *
 * @ingroup TMP100
 */

#ifndef TMP100_H_
#define TMP100_H_

/**
 * @brief Rate at which the TMP100 sensors are read (in ms)
 */
#define TMP100_UPDATE_RATE        500

extern TaskHandle_t vTaskTMP100_Handle;

extern const SDR_type_01h_t SDR_TMP100_FPGA;
extern const SDR_type_01h_t SDR_TMP100_DCDC;

/**
 * @brief Initializes TMP100 monitoring task
 *
 * @return None
 */
void TMP100_init( void );

/**
 * @brief Monitoring task for TMP100 sensor
 *
 * This task unblocks after every #TMP100_UPDATE_RATE ms and updates the read from all the TMP100 sensors listed in this module's SDR table
 *
 * @param Parameters Pointer to parameter list passed to task upon initialization (not used here)
 */
void vTaskTMP100( void* Parameters );

#endif
