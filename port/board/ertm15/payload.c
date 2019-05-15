/*
 *   openMMC -- Open Source modular IPM Controller firmware
 *
 *   Copyright (C) 2015  Piotr Miedzik  <P.Miedzik@gsi.de>
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

/* FreeRTOS Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"

/* Project Includes */
#include "port.h"
#include "payload.h"
//#include "ipmi.h"
#include "task_priorities.h"
//#include "adn4604.h"
//#include "ad84xx.h"
//#include "hotswap.h"
#include "utils.h"
#include "fru.h"
#include "led.h"
#include "board_led.h"

/* payload states
 *   0 - No power
 *
 *   1 - Power Good wait
 *       Enable DCDC Converters
 *       Hotswap backend power failure and shutdown status clear
 *
 *   2 - FPGA setup
 *       One-time configurations (clock switch - ADN4604)
 *
 *   3 - FPGA on
 *
 *   4 - Power switching off
 *       Disable DCDC Converters
 *       Send "quiesced" event if requested
 *
 *   5 - Power quiesced
 *       Payload was safely turned off
 *       Wait until payload power goes down to restart the cycle
 */



// uint8_t payload_check_pgood( uint8_t *pgood_flag )
// {
//     sensor_t * p_sensor;
//     SDR_type_01h_t *sdr;
// 
//     extern const SDR_type_01h_t SDR_FMC1_12V;
// 
//     /* Iterate through the SDR Table to find all the LM75 entries */
//     for ( p_sensor = sdr_head; (p_sensor != NULL) || (p_sensor->task_handle == NULL); p_sensor = p_sensor->next) {
//         if (p_sensor->sdr == &SDR_FMC1_12V) {
//             sdr = ( SDR_type_01h_t * ) p_sensor->sdr;
//             *pgood_flag = ( ( p_sensor->readout_value >= (sdr->lower_critical_thr ) ) &&
//                             ( p_sensor->readout_value <= (sdr->upper_critical_thr ) ) );
//             return 1;
//         }
//     }
// 
//     return 0;
// }

#define POWER_GOOD_TIMEOUT_TICKS 100


int switch_power( const char* name, enum pin_enum pin_en, enum pin_enum pin_pg, uint8_t en, uint8_t pg)
{
    uint8_t val;
    int i;

    /* TODO: In the next revision of a board use en instead of pg to distinguish
     * if the power is being enabled or disabled */
    printf("%s PSU: %s", pg ? "Enabling" : "Disabling", name );
    /* for STM32 second parameter is ignored */
    gpio_set_pin_state(pin_en, 0, en);
    for(i = 0; i < POWER_GOOD_TIMEOUT_TICKS; i++) {
	val = gpio_read_pin(pin_pg, 0);
	if (val == pg) {
	    printf("done\r\n");
	    return 1;
	}
	printf(".");
	vTaskDelay(1);
    }
    printf("Timeout\r\n");
    return 0;
}

/**
 * @brief Set eRTM15's DCDC Converters state
 *
 * @param on DCDCs state
 *
 */
/* TODO: Follow the power up sequence described on schematics */
void setDC_DC_Converters( bool state )
{
    if (
	/* Main power */
	switch_power("P3V6",    GPIO_P3V6_EN,   GPIO_P3V6_PG,   !state, state)
    ) {
	printf("FPGA Power %s\r\n", state ? "Enabled" : "Disabled");
    }

}

void dump_powergood(void)
{
    printf("%d\r\n",
	gpio_read_pin(GPIO_P3V6_PG,  0)
    );
}

void dump_poweren(void)
{
    printf("%d\r\n",
	gpio_read_pin(GPIO_P3V6_EN,  0)
    );
}

static uint8_t payload_check_powergood(void)
{
    return (
	/* Main power */
	gpio_read_pin(GPIO_P3V6_PG,  0)
    );
}

EventGroupHandle_t ertm15_payload_evt = NULL;

void payload_send_message( uint8_t fru_id, EventBits_t msg)
{
    if ( (fru_id == FRU_AMC) && ertm15_payload_evt ) {
        xEventGroupSetBits( ertm15_payload_evt, msg );
    }
}

TaskHandle_t vTaskPayload_Handle;

void payload_init( void )
{

#ifndef BENCH_TEST
    /* Wait until ENABLE# signal is asserted ( ENABLE == 0) */
    while ( gpio_read_pin( PIN_PORT(GPIO_BP_ENABLE_N), 0 ) == 1 ) {};
#endif

    xTaskCreate( vTaskPayload, "Payload", 120, NULL, tskPAYLOAD_PRIORITY, &vTaskPayload_Handle );

    ertm15_payload_evt = xEventGroupCreate();
}

void vTaskPayload( void *pvParameters )
{
    uint8_t state = PAYLOAD_NO_POWER;
    /* Use arbitrary state value to force the first state update */
    uint8_t new_state = -1;

    /* Payload power good flag */
    uint8_t PP_good = 1; /* remove? */

    /* Payload DCDCs good flag */
    uint8_t DCDC_good = 0;

    uint8_t QUIESCED_req = 0;
    EventBits_t current_evt;

//     extern sensor_t * hotswap_amc_sensor;

    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

//     gpio_set_pin_state( PIN_PORT(GPIO_FPGA_PROGRAM_B), PIN_NUMBER(GPIO_FPGA_PROGRAM_B), GPIO_LEVEL_HIGH );

    for ( ;; ) {
//         check_fpga_reset();

        /* Initialize one of the FMC's DCDC so we can measure when the Payload Power is present */
//         gpio_set_pin_state( PIN_PORT(GPIO_EN_FMC1_P12V), PIN_NUMBER(GPIO_EN_FMC1_P12V), GPIO_LEVEL_HIGH );

        new_state = state;

        current_evt = xEventGroupGetBits( ertm15_payload_evt );

        if ( current_evt & PAYLOAD_MESSAGE_QUIESCE ) {
            QUIESCED_req = 1;
            xEventGroupClearBits( ertm15_payload_evt, PAYLOAD_MESSAGE_QUIESCE );
        }

        if ( current_evt & PAYLOAD_MESSAGE_COLD_RST ) {
            state = PAYLOAD_SWITCHING_OFF;
            xEventGroupClearBits( ertm15_payload_evt, PAYLOAD_MESSAGE_COLD_RST );
        }

        if ( (current_evt & PAYLOAD_MESSAGE_REBOOT) || (current_evt & PAYLOAD_MESSAGE_WARM_RST) ) {
//             fpga_soft_reset();
            xEventGroupClearBits( ertm15_payload_evt, PAYLOAD_MESSAGE_REBOOT );
        }

//         payload_check_pgood(&PP_good);
        DCDC_good = payload_check_powergood();
// 	printf("DCDC_good %d\r\n", DCDC_good);
// 	dump_powergood();
	//gpio_read_pin( PIN_PORT(GPIO_DCDC_PGOOD), PIN_NUMBER(GPIO_DCDC_PGOOD) );

        switch(state) {

        case PAYLOAD_NO_POWER:
            if (PP_good) {
                new_state = PAYLOAD_POWER_GOOD_WAIT;
            }
            break;

        case PAYLOAD_POWER_GOOD_WAIT:

            /* Clear hotswap sensor backend power failure bits */
//             hotswap_clear_mask_bit( HOTSWAP_AMC, HOTSWAP_BACKEND_PWR_SHUTDOWN_MASK );
//             hotswap_clear_mask_bit( HOTSWAP_AMC, HOTSWAP_BACKEND_PWR_FAILURE_MASK );

            if ( QUIESCED_req/* || ( PP_good == 0 )*/) {
                new_state = PAYLOAD_SWITCHING_OFF;
            } else if ( DCDC_good == 1 ) {
                new_state = PAYLOAD_STATE_FPGA_SETUP;
            } else if ( DCDC_good == 0 ) {
		/* Turn DCDC converters on */
		setDC_DC_Converters( true );
	    }
            break;

        case PAYLOAD_STATE_FPGA_SETUP:
            new_state = PAYLOAD_FPGA_ON;
            break;

        case PAYLOAD_FPGA_ON:
            if ( QUIESCED_req == 1 || PP_good == 0 || DCDC_good == 0 ) {
		printf("Turn off %d %d %d\r\n", QUIESCED_req, PP_good, DCDC_good );
                new_state = PAYLOAD_SWITCHING_OFF;
            }
            break;

        case PAYLOAD_SWITCHING_OFF:
            setDC_DC_Converters( false );

            /* Respond to quiesce event if any */
            if ( QUIESCED_req ) {
//                 hotswap_set_mask_bit( HOTSWAP_AMC, HOTSWAP_QUIESCED_MASK );
//                 hotswap_send_event( hotswap_amc_sensor, HOTSWAP_STATE_QUIESCED );
//                 hotswap_clear_mask_bit( HOTSWAP_AMC, HOTSWAP_QUIESCED_MASK );
                QUIESCED_req = 0;
            }
            new_state = PAYLOAD_QUIESCED;
            break;

        case PAYLOAD_QUIESCED:
            /* Wait until power goes down to restart the cycle */
            if (PP_good == 0 && DCDC_good == 0) {
                new_state = PAYLOAD_NO_POWER;
            }
            break;

        default:
            break;
        }

        state = new_state;
        vTaskDelayUntil( &xLastWakeTime, PAYLOAD_BASE_DELAY );
    }
}

