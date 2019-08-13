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
 * @file board_stm32f3_adc.h
 * @author Adam Wujek <adam.wujek@cern.ch>, CERN
 *
 * @brief Board specific definitions used by STM's ADC
 */

#ifndef BOARD_STM32F3_ADC_H_
#define BOARD_STM32F3_ADC_H_

#include "port.h"

enum STM32F3_ADC_CH {
    STM32F3_ADC_P3V3,
    STM32F3_ADC_P12V,
    STM32F3_ADC_INT_TEMP,
    STM32F3_ADC_CH_MAX_CNT
};

enum STM32F3_ADC_ID {
    STM32F3_ADC1,
    STM32F3_ADC_ID_MAX_CNT
};


extern const struct stm32f3_adc_def stm32f3_adc_config[STM32F3_ADC_ID_MAX_CNT];
extern const struct stm32f3_adc_ch_def stm32f3_adc_ch_config[STM32F3_ADC_CH_MAX_CNT];

void board_stm32f3_adc_init(void);

#endif /* BOARD_STM32F3_ADC_H_ */
