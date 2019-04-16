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
 * @file tmp100.c
 * @author Adam Wujek  <adam.wujek@cern.ch>, CERN
 *
 * @brief Interface functions for TMP100 I2C Temperature Sensor
 *
 * @ingroup TMP100
 */

/* FreeRTOS Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "port.h"

/* Project Includes */
#include "sdr.h"
#include "task_priorities.h"
#include "i2c.h"
#include "i2c_mapping.h"
#include "tmp100.h"
#include "utils.h"
#include "uart_debug.h"

TaskHandle_t vTaskTMP100_Handle;

void tmp100_init(uint8_t i2c_interf, uint8_t addr){
    uint8_t tmp[2];

    tmp[0] = 0x1; /* reg pointer to reg 1 */
    tmp[1] = 0x60; /* continuous conversion, comparator mode, 12-bit resolution */
    xI2CMasterWrite(i2c_interf, addr, tmp, 2);
    tmp[0] = 0x0; /* reg pointer to reg 0 */
    xI2CMasterWrite(i2c_interf, addr, tmp, 1);
}

uint8_t tmp100_read(uint8_t i2c_interf, uint8_t addr, uint16_t *converted_temp){
    uint8_t tmp[2];
    uint8_t read_b;

    read_b = xI2CMasterRead(i2c_interf, addr, tmp, 2);
    *converted_temp = tmp[0];
    *converted_temp <<= 8;
    *converted_temp |= tmp[1];
    return read_b;
}

void vTaskTMP100( void* Parameters )
{
    const TickType_t xFrequency = TMP100_UPDATE_RATE;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint8_t i2c_addr, i2c_interf;
    sensor_t * temp_sensor;
    uint32_t temp;
    uint16_t converted_temp;

    for ( temp_sensor = sdr_head; temp_sensor != NULL; temp_sensor = temp_sensor->next) {

	if ( temp_sensor->task_handle == NULL ) {
	    continue;
	}

	/* Check if this task should update the selected SDR */
	if ( *(temp_sensor->task_handle) != xTaskGetCurrentTaskHandle() ) {
	    continue;
	}

	/* Try to gain the I2C bus */
	if ( i2c_take_by_chipid( temp_sensor->chipid, &i2c_addr, &i2c_interf, portMAX_DELAY ) == pdTRUE ) {
	    /* Update the temperature reading */
	    printf("TMP100 I2C%d addr %x\r\n", i2c_interf, i2c_addr);
	    tmp100_init(i2c_interf, i2c_addr);

	    /* Check for threshold events */
	    i2c_give(i2c_interf);
	}
    }

    for ( ;; ) {
        /* Iterate through the SDR Table to find all the TMP100 entries */

        for ( temp_sensor = sdr_head; temp_sensor != NULL; temp_sensor = temp_sensor->next) {

            if ( temp_sensor->task_handle == NULL ) {
                continue;
            }

            /* Check if this task should update the selected SDR */
            if ( *(temp_sensor->task_handle) != xTaskGetCurrentTaskHandle() ) {
                continue;
            }

            /* Try to gain the I2C bus */
            if ( i2c_take_by_chipid( temp_sensor->chipid, &i2c_addr, &i2c_interf, portMAX_DELAY ) == pdTRUE ) {
                /* Update the temperature reading */
		if (tmp100_read(i2c_interf, i2c_addr, &converted_temp) == 2) {
                    temp_sensor->readout_value = converted_temp;
		    temp = converted_temp;
		    temp *= 100;
		    printf("%s %d.%02d\r\n", ((SDR_type_01h_t *)temp_sensor->sdr)->IDstring, (int)temp/25600, (int)(temp%25600)/256);
                }

                /* Check for threshold events */
                i2c_give(i2c_interf);
                /* TODO: check_sensor_event(temp_sensor); */
            }
        }
        vTaskDelayUntil( &xLastWakeTime, xFrequency );
    }
}


void TMP100_init( void )
{
    xTaskCreate( vTaskTMP100, "TMP100", 200, (void *) NULL, tskTMP100SENSOR_PRIORITY, &vTaskTMP100_Handle);
}
