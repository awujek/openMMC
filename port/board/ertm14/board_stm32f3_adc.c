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
 * @file board_stm32f3_adc.c
 * @author Adam Wujek <adam.wujek@cern.ch>, CERN
 *
 * @brief Board specific definitions used by STM's ADC
 */

/* Project includes */
#include "FreeRTOS.h"
#include "task.h"

#include "port.h"
#include "stm32f3_adc.h"
#include "board_stm32f3_adc.h"

ADC_CommonInitTypeDef ADC_CommonInitStructure = {
    .ADC_Mode = ADC_Mode_Independent,
    .ADC_Clock = ADC_Clock_SynClkModeDiv4,
    .ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled,
    .ADC_DMAMode = ADC_DMAMode_OneShot,
    .ADC_TwoSamplingDelay = 0,
};

ADC_InitTypeDef ADC_InitStructure = {
    .ADC_ContinuousConvMode = ADC_ContinuousConvMode_Enable,
    .ADC_Resolution = ADC_Resolution_12b,
    .ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0,
    .ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None,
    .ADC_DataAlign = ADC_DataAlign_Right,
    .ADC_OverrunMode = ADC_OverrunMode_Disable,
    .ADC_AutoInjMode = ADC_AutoInjec_Disable,
    .ADC_NbrOfRegChannel = 1,
};

const struct stm32f3_adc_def stm32f3_adc_config[] = {
    [STM32F3_ADC1] = {
	.ADCx = ADC1,
	.RCC_PLLCLK = RCC_ADC12PLLCLK_Div16,
	.RCC_AHBPeriph = RCC_AHBPeriph_ADC12,
	.ADC_CommonInitStructure = &ADC_CommonInitStructure,
	.ADC_InitStructure = &ADC_InitStructure,
	},
};

const struct stm32f3_adc_ch_def stm32f3_adc_ch_config[] = {
    [STM32F3_ADC_P3V3]     = { ADC1, ADC_Channel_7 },
    [STM32F3_ADC_P12V]     = { ADC1, ADC_Channel_6 },
    [STM32F3_ADC_INT_TEMP] = { ADC1, ADC_Channel_16 },
};

void board_stm32f3_adc_init(void) {
    /* allow readouts of internal temperature sensor */
    ADC_TempSensorCmd(ADC1, ENABLE);

    /* allow readouts of internal vref voltage, may be used for testing */
    /* ADC_VrefintCmd(ADC1, ENABLE);
    ADC_VrefintCmd(ADC2, ENABLE); */
}
