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
 * @defgroup ERTM15 eRTM15 Board Port
 * @ingroup BOARD_PORTS
 */

/**
 * @file ertm15/pin_mapping.h
 * @brief Hardware pin definitions for eRTM15
 *
 * @ingroup ERTM15_PIN_MAPPING
 */

/**
 * @defgroup ERTM15_PIN_MAPPING eRTM15 Pin Mapping
 * @ingroup ERTM15
 * @{
 */

#ifndef PIN_MAPPING_H_
#define PIN_MAPPING_H_

enum pin_enum {
    GPIO_LEDS_UCLK = 0,
    GPIO_LEDS_SCLK,
    GPIO_LEDS_SER,

    GPIO_REF_LEDGREEN,   /* Serial GPIOs */
    GPIO_REF_LEDRED,     /* Serial GPIOs */
    GPIO_LO_LEDGREEN,    /* Serial GPIOs */
    GPIO_LO_LEDRED,      /* Serial GPIOs */
    GPIO_CLK_LEDGREEN,   /* Serial GPIOs */
    GPIO_CLK_LEDRED,     /* Serial GPIOs */
    GPIO_POWER_LEDGREEN, /* Serial GPIOs */
    GPIO_POWER_LEDRED,   /* Serial GPIOs */

    GPIO_DCDC_EN_UCLK,
    GPIO_DCDC_EN_SCLK,
    GPIO_DCDC_EN_SER,

    GPIO_P3V6_EN,       /* Serial GPIOs */
    GPIO_P3V3_EN,       /* Serial GPIOs */
    GPIO_P1V8_EN,       /* Serial GPIOs */
    GPIO_P3V3_PLL_EN,   /* Serial GPIOs */
    GPIO_P5V0_EN,       /* Serial GPIOs */
    GPIO_P9V0_LO_EN,    /* Serial GPIOs */
    GPIO_P9V0_REF_EN,   /* Serial GPIOs */

    GPIO_P1V8_PG,
    GPIO_P3V3_PG,
    GPIO_P3V3_PLL_PG,
    GPIO_P3V6_PG,
    GPIO_P5V0_PG,

    GPIO_P3V3_DIV,
    GPIO_P9V0_LO_DIV,
    GPIO_P9V0_REF_DIV,
    GPIO_P12V_DIV,
    GPIO_POCXO_DIV,

    GPIO_PVADJ_OCXO_EN,
    GPIO_OCXO_CURR,
    GPIO_RESET_LOGIC,

    GPIO_DDS_REF_POWERDOWN,
    GPIO_DDS_LO_POWERDOWN,
    GPIO_CLKA_POWERDOWN,
    GPIO_CLKB_POWERDOWN,

    GPIO_BP_PWR_ON,
    GPIO_BP_ENABLE_N,
    GPIO_GA0,
    GPIO_GA1,
    GPIO_GA2,
    GPIO_MAX
};

extern struct pin_def gpio_pins_def[GPIO_MAX];

/* UART Interfaces */
#define UART_DEBUG      1 /* TODO: fix */

#endif

/**
 * @}
 */
