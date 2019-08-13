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

/*! @file fru.c
 */

#include "port.h"
#include "fru.h"
#include "fru_editor.h"
#include "eeprom_24aa025e48.h"
#include "i2c_mapping.h"

fru_data_t fru[FRU_COUNT] = {
    [FRU_AMC] = {
        .cfg = {
            .eeprom_id = CHIP_ID_MAC,
            .build_f = amc_fru_info_build,
            .read_f = eeprom_24aa025e48_read,
            .write_f = eeprom_24aa025e48_write,
        },
        .runtime = false
    },
};

