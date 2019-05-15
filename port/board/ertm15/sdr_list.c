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

/* Project Includes */
#include "sdr.h"
#include "utils.h"
#include "i2c_mapping.h"

/* Sensors includes */
#include "sensors.h"
#include "fpga_spi.h"

/* SDR List */
/* TODO: Verify all values below! */

#ifdef MODULE_TMP100
const SDR_type_01h_t SDR_TMP100_PS = {

    .hdr.recID_LSB = 0x00, /* Filled by sdr_insert_entry() */
    .hdr.recID_MSB = 0x00,
    .hdr.SDRversion = 0x51,
    .hdr.rectype = TYPE_01,
    .hdr.reclength = sizeof(SDR_type_01h_t) - sizeof(SDR_entry_hdr_t),

    .ownerID = 0x00, /* i2c address, -> SDR_Init */
    .ownerLUN = 0x00, /* sensor owner LUN */
    .sensornum = 0x00, /* Filled by sdr_insert_entry() */

    /* record body bytes */
    .entityID = 0xC1, /* entity id: AMC Module */
    .entityinstance = 0x00, /* entity instance -> SDR_Init */
    .sensorinit = 0x7f, /* init: event generation + scanning enabled */
    .sensorcap = 0x68, /* capabilities: auto re-arm,*/
    .sensortype = SENSOR_TYPE_TEMPERATURE, /* sensor type */
    .event_reading_type = 0x01, /* sensor reading*/
    .assertion_event_mask = { 0xFF, /* LSB assert event mask: 3 bit value */
                              0x0F }, /* MSB assert event mask */
    .deassertion_event_mask = { 0xFF, /* LSB deassert event mask: 3 bit value */
                                0x0F }, /* MSB deassert event mask */
    .readable_threshold_mask = 0x3F, /* LSB: readabled Threshold mask: all thresholds are readabled:  */
    .settable_threshold_mask = 0x3F, /* MSB: setabled Threshold mask: all thresholds are setabled: */
    .sensor_units_1 = 0x00, /* sensor units 1 :*/
    .sensor_units_2 = 0x01, /* sensor units 2 :*/
    .sensor_units_3 = 0x00, /* sensor units 3 :*/
    .linearization = 0x00, /* Linearization */
    .M = 5, /* M */
    .M_tol = 0x00, /* M - Tolerance */
    .B = 0x00, /* B */
    .B_accuracy = 0x00, /* B - Accuracy */
    .acc_exp_sensor_dir = 0x00, /* Sensor direction */
    .Rexp_Bexp = 0xF0, /* R-Exp , B-Exp */
    .analog_flags = 0x03, /* Analogue characteristics flags */
    .nominal_reading = (20 << 1), /* Nominal reading */
    .normal_max = (50 << 1), /* Normal maximum */
    .normal_min = (10 << 1), /* Normal minimum */
    .sensor_max_reading = 0xFF, /* Sensor Maximum reading */
    .sensor_min_reading = 0x00, /* Sensor Minimum reading */
    .upper_nonrecover_thr = (80 << 1), /* Upper non-recoverable Threshold */
    .upper_critical_thr = (75 << 1), /* Upper critical Threshold */
    .upper_noncritical_thr = (65 << 1), /* Upper non critical Threshold */
    .lower_nonrecover_thr = (0 << 1), /* Lower non-recoverable Threshold */
    .lower_critical_thr = (5 << 1), /* Lower critical Threshold */
    .lower_noncritical_thr = (10 << 1), /* Lower non-critical Threshold */
    .pos_thr_hysteresis = 2, /* positive going Threshold hysteresis value */
    .neg_thr_hysteresis = 2, /* negative going Threshold hysteresis value */
    .reserved1 = 0x00, /* reserved */
    .reserved2 = 0x00, /* reserved */
    .OEM = 0x00, /* OEM reserved */
    .IDtypelen = 0xc0 | STR_SIZE("TEMP Power Supp"), /* 8 bit ASCII, number of bytes */
    .IDstring = "TEMP Power Supp" /* sensor string */
};

const SDR_type_01h_t SDR_TMP100_LO_RF = {

    .hdr.recID_LSB = 0x00, /* Filled by sdr_insert_entry() */
    .hdr.recID_MSB = 0x00,
    .hdr.SDRversion = 0x51,
    .hdr.rectype = TYPE_01,
    .hdr.reclength = sizeof(SDR_type_01h_t) - sizeof(SDR_entry_hdr_t),

    .ownerID = 0x00, /* i2c address, -> SDR_Init */
    .ownerLUN = 0x00, /* sensor owner LUN */
    .sensornum = 0x00, /* Filled by sdr_insert_entry() */

    /* record body bytes */
    .entityID = 0xC1, /* entity id: AMC Module */
    .entityinstance = 0x00, /* entity instance -> SDR_Init */
    .sensorinit = 0x7f, /* init: event generation + scanning enabled */
    .sensorcap = 0x68, /* capabilities: auto re-arm,*/
    .sensortype = SENSOR_TYPE_TEMPERATURE, /* sensor type */
    .event_reading_type = 0x01, /* sensor reading*/
    .assertion_event_mask = { 0xFF, /* LSB assert event mask: 3 bit value */
                              0x0F }, /* MSB assert event mask */
    .deassertion_event_mask = { 0xFF, /* LSB deassert event mask: 3 bit value */
                                0x0F }, /* MSB deassert event mask */
    .readable_threshold_mask = 0x3F, /* LSB: readabled Threshold mask: all thresholds are readabled:  */
    .settable_threshold_mask = 0x3F, /* MSB: setabled Threshold mask: all thresholds are setabled: */
    .sensor_units_1 = 0x00, /* sensor units 1 :*/
    .sensor_units_2 = 0x01, /* sensor units 2 :*/
    .sensor_units_3 = 0x00, /* sensor units 3 :*/
    .linearization = 0x00, /* Linearization */
    .M = 5, /* M */
    .M_tol = 0x00, /* M - Tolerance */
    .B = 0x00, /* B */
    .B_accuracy = 0x00, /* B - Accuracy */
    .acc_exp_sensor_dir = 0x00, /* Sensor direction */
    .Rexp_Bexp = 0xF0, /* R-Exp , B-Exp */
    .analog_flags = 0x03, /* Analogue characteristics flags */
    .nominal_reading = (20 << 1), /* Nominal reading */
    .normal_max = (50 << 1), /* Normal maximum */
    .normal_min = (10 << 1), /* Normal minimum */
    .sensor_max_reading = 0xFF, /* Sensor Maximum reading */
    .sensor_min_reading = 0x00, /* Sensor Minimum reading */
    .upper_nonrecover_thr = (80 << 1), /* Upper non-recoverable Threshold */
    .upper_critical_thr = (75 << 1), /* Upper critical Threshold */
    .upper_noncritical_thr = (65 << 1), /* Upper non critical Threshold */
    .lower_nonrecover_thr = (0 << 1), /* Lower non-recoverable Threshold */
    .lower_critical_thr = (5 << 1), /* Lower critical Threshold */
    .lower_noncritical_thr = (10 << 1), /* Lower non-critical Threshold */
    .pos_thr_hysteresis = 2, /* positive going Threshold hysteresis value */
    .neg_thr_hysteresis = 2, /* negative going Threshold hysteresis value */
    .reserved1 = 0x00, /* reserved */
    .reserved2 = 0x00, /* reserved */
    .OEM = 0x00, /* OEM reserved */
    .IDtypelen = 0xc0 | STR_SIZE("TEMP LO_RF amp"), /* 8 bit ASCII, number of bytes */
    .IDstring = "TEMP LO_RF amp" /* sensor string */
};

const SDR_type_01h_t SDR_TMP100_DDS_LO = {

    .hdr.recID_LSB = 0x00, /* Filled by sdr_insert_entry() */
    .hdr.recID_MSB = 0x00,
    .hdr.SDRversion = 0x51,
    .hdr.rectype = TYPE_01,
    .hdr.reclength = sizeof(SDR_type_01h_t) - sizeof(SDR_entry_hdr_t),

    .ownerID = 0x00, /* i2c address, -> SDR_Init */
    .ownerLUN = 0x00, /* sensor owner LUN */
    .sensornum = 0x00, /* Filled by sdr_insert_entry() */

    /* record body bytes */
    .entityID = 0xC1, /* entity id: AMC Module */
    .entityinstance = 0x00, /* entity instance -> SDR_Init */
    .sensorinit = 0x7f, /* init: event generation + scanning enabled */
    .sensorcap = 0x68, /* capabilities: auto re-arm,*/
    .sensortype = SENSOR_TYPE_TEMPERATURE, /* sensor type */
    .event_reading_type = 0x01, /* sensor reading*/
    .assertion_event_mask = { 0xFF, /* LSB assert event mask: 3 bit value */
                              0x0F }, /* MSB assert event mask */
    .deassertion_event_mask = { 0xFF, /* LSB deassert event mask: 3 bit value */
                                0x0F }, /* MSB deassert event mask */
    .readable_threshold_mask = 0x3F, /* LSB: readabled Threshold mask: all thresholds are readabled:  */
    .settable_threshold_mask = 0x3F, /* MSB: setabled Threshold mask: all thresholds are setabled: */
    .sensor_units_1 = 0x00, /* sensor units 1 :*/
    .sensor_units_2 = 0x01, /* sensor units 2 :*/
    .sensor_units_3 = 0x00, /* sensor units 3 :*/
    .linearization = 0x00, /* Linearization */
    .M = 5, /* M */
    .M_tol = 0x00, /* M - Tolerance */
    .B = 0x00, /* B */
    .B_accuracy = 0x00, /* B - Accuracy */
    .acc_exp_sensor_dir = 0x00, /* Sensor direction */
    .Rexp_Bexp = 0xF0, /* R-Exp , B-Exp */
    .analog_flags = 0x03, /* Analogue characteristics flags */
    .nominal_reading = (20 << 1), /* Nominal reading */
    .normal_max = (50 << 1), /* Normal maximum */
    .normal_min = (10 << 1), /* Normal minimum */
    .sensor_max_reading = 0xFF, /* Sensor Maximum reading */
    .sensor_min_reading = 0x00, /* Sensor Minimum reading */
    .upper_nonrecover_thr = (80 << 1), /* Upper non-recoverable Threshold */
    .upper_critical_thr = (75 << 1), /* Upper critical Threshold */
    .upper_noncritical_thr = (65 << 1), /* Upper non critical Threshold */
    .lower_nonrecover_thr = (0 << 1), /* Lower non-recoverable Threshold */
    .lower_critical_thr = (5 << 1), /* Lower critical Threshold */
    .lower_noncritical_thr = (10 << 1), /* Lower non-critical Threshold */
    .pos_thr_hysteresis = 2, /* positive going Threshold hysteresis value */
    .neg_thr_hysteresis = 2, /* negative going Threshold hysteresis value */
    .reserved1 = 0x00, /* reserved */
    .reserved2 = 0x00, /* reserved */
    .OEM = 0x00, /* OEM reserved */
    .IDtypelen = 0xc0 | STR_SIZE("TEMP DDS_LO"), /* 8 bit ASCII, number of bytes */
    .IDstring = "TEMP DDS_LO" /* sensor string */
};

const SDR_type_01h_t SDR_TMP100_LTC6150 = {

    .hdr.recID_LSB = 0x00, /* Filled by sdr_insert_entry() */
    .hdr.recID_MSB = 0x00,
    .hdr.SDRversion = 0x51,
    .hdr.rectype = TYPE_01,
    .hdr.reclength = sizeof(SDR_type_01h_t) - sizeof(SDR_entry_hdr_t),

    .ownerID = 0x00, /* i2c address, -> SDR_Init */
    .ownerLUN = 0x00, /* sensor owner LUN */
    .sensornum = 0x00, /* Filled by sdr_insert_entry() */

    /* record body bytes */
    .entityID = 0xC1, /* entity id: AMC Module */
    .entityinstance = 0x00, /* entity instance -> SDR_Init */
    .sensorinit = 0x7f, /* init: event generation + scanning enabled */
    .sensorcap = 0x68, /* capabilities: auto re-arm,*/
    .sensortype = SENSOR_TYPE_TEMPERATURE, /* sensor type */
    .event_reading_type = 0x01, /* sensor reading*/
    .assertion_event_mask = { 0xFF, /* LSB assert event mask: 3 bit value */
                              0x0F }, /* MSB assert event mask */
    .deassertion_event_mask = { 0xFF, /* LSB deassert event mask: 3 bit value */
                                0x0F }, /* MSB deassert event mask */
    .readable_threshold_mask = 0x3F, /* LSB: readabled Threshold mask: all thresholds are readabled:  */
    .settable_threshold_mask = 0x3F, /* MSB: setabled Threshold mask: all thresholds are setabled: */
    .sensor_units_1 = 0x00, /* sensor units 1 :*/
    .sensor_units_2 = 0x01, /* sensor units 2 :*/
    .sensor_units_3 = 0x00, /* sensor units 3 :*/
    .linearization = 0x00, /* Linearization */
    .M = 5, /* M */
    .M_tol = 0x00, /* M - Tolerance */
    .B = 0x00, /* B */
    .B_accuracy = 0x00, /* B - Accuracy */
    .acc_exp_sensor_dir = 0x00, /* Sensor direction */
    .Rexp_Bexp = 0xF0, /* R-Exp , B-Exp */
    .analog_flags = 0x03, /* Analogue characteristics flags */
    .nominal_reading = (20 << 1), /* Nominal reading */
    .normal_max = (50 << 1), /* Normal maximum */
    .normal_min = (10 << 1), /* Normal minimum */
    .sensor_max_reading = 0xFF, /* Sensor Maximum reading */
    .sensor_min_reading = 0x00, /* Sensor Minimum reading */
    .upper_nonrecover_thr = (80 << 1), /* Upper non-recoverable Threshold */
    .upper_critical_thr = (75 << 1), /* Upper critical Threshold */
    .upper_noncritical_thr = (65 << 1), /* Upper non critical Threshold */
    .lower_nonrecover_thr = (0 << 1), /* Lower non-recoverable Threshold */
    .lower_critical_thr = (5 << 1), /* Lower critical Threshold */
    .lower_noncritical_thr = (10 << 1), /* Lower non-critical Threshold */
    .pos_thr_hysteresis = 2, /* positive going Threshold hysteresis value */
    .neg_thr_hysteresis = 2, /* negative going Threshold hysteresis value */
    .reserved1 = 0x00, /* reserved */
    .reserved2 = 0x00, /* reserved */
    .OEM = 0x00, /* OEM reserved */
    .IDtypelen = 0xc0 | STR_SIZE("TEMP LTC6150"), /* 8 bit ASCII, number of bytes */
    .IDstring = "TEMP LTC6150" /* sensor string */
};

const SDR_type_01h_t SDR_TMP100_REF_RF = {

    .hdr.recID_LSB = 0x00, /* Filled by sdr_insert_entry() */
    .hdr.recID_MSB = 0x00,
    .hdr.SDRversion = 0x51,
    .hdr.rectype = TYPE_01,
    .hdr.reclength = sizeof(SDR_type_01h_t) - sizeof(SDR_entry_hdr_t),

    .ownerID = 0x00, /* i2c address, -> SDR_Init */
    .ownerLUN = 0x00, /* sensor owner LUN */
    .sensornum = 0x00, /* Filled by sdr_insert_entry() */

    /* record body bytes */
    .entityID = 0xC1, /* entity id: AMC Module */
    .entityinstance = 0x00, /* entity instance -> SDR_Init */
    .sensorinit = 0x7f, /* init: event generation + scanning enabled */
    .sensorcap = 0x68, /* capabilities: auto re-arm,*/
    .sensortype = SENSOR_TYPE_TEMPERATURE, /* sensor type */
    .event_reading_type = 0x01, /* sensor reading*/
    .assertion_event_mask = { 0xFF, /* LSB assert event mask: 3 bit value */
                              0x0F }, /* MSB assert event mask */
    .deassertion_event_mask = { 0xFF, /* LSB deassert event mask: 3 bit value */
                                0x0F }, /* MSB deassert event mask */
    .readable_threshold_mask = 0x3F, /* LSB: readabled Threshold mask: all thresholds are readabled:  */
    .settable_threshold_mask = 0x3F, /* MSB: setabled Threshold mask: all thresholds are setabled: */
    .sensor_units_1 = 0x00, /* sensor units 1 :*/
    .sensor_units_2 = 0x01, /* sensor units 2 :*/
    .sensor_units_3 = 0x00, /* sensor units 3 :*/
    .linearization = 0x00, /* Linearization */
    .M = 5, /* M */
    .M_tol = 0x00, /* M - Tolerance */
    .B = 0x00, /* B */
    .B_accuracy = 0x00, /* B - Accuracy */
    .acc_exp_sensor_dir = 0x00, /* Sensor direction */
    .Rexp_Bexp = 0xF0, /* R-Exp , B-Exp */
    .analog_flags = 0x03, /* Analogue characteristics flags */
    .nominal_reading = (20 << 1), /* Nominal reading */
    .normal_max = (50 << 1), /* Normal maximum */
    .normal_min = (10 << 1), /* Normal minimum */
    .sensor_max_reading = 0xFF, /* Sensor Maximum reading */
    .sensor_min_reading = 0x00, /* Sensor Minimum reading */
    .upper_nonrecover_thr = (80 << 1), /* Upper non-recoverable Threshold */
    .upper_critical_thr = (75 << 1), /* Upper critical Threshold */
    .upper_noncritical_thr = (65 << 1), /* Upper non critical Threshold */
    .lower_nonrecover_thr = (0 << 1), /* Lower non-recoverable Threshold */
    .lower_critical_thr = (5 << 1), /* Lower critical Threshold */
    .lower_noncritical_thr = (10 << 1), /* Lower non-critical Threshold */
    .pos_thr_hysteresis = 2, /* positive going Threshold hysteresis value */
    .neg_thr_hysteresis = 2, /* negative going Threshold hysteresis value */
    .reserved1 = 0x00, /* reserved */
    .reserved2 = 0x00, /* reserved */
    .OEM = 0x00, /* OEM reserved */
    .IDtypelen = 0xc0 | STR_SIZE("TEMP REF_RF"), /* 8 bit ASCII, number of bytes */
    .IDstring = "TEMP REF_RF" /* sensor string */
};

const SDR_type_01h_t SDR_TMP100_DDS_REF = {

    .hdr.recID_LSB = 0x00, /* Filled by sdr_insert_entry() */
    .hdr.recID_MSB = 0x00,
    .hdr.SDRversion = 0x51,
    .hdr.rectype = TYPE_01,
    .hdr.reclength = sizeof(SDR_type_01h_t) - sizeof(SDR_entry_hdr_t),

    .ownerID = 0x00, /* i2c address, -> SDR_Init */
    .ownerLUN = 0x00, /* sensor owner LUN */
    .sensornum = 0x00, /* Filled by sdr_insert_entry() */

    /* record body bytes */
    .entityID = 0xC1, /* entity id: AMC Module */
    .entityinstance = 0x00, /* entity instance -> SDR_Init */
    .sensorinit = 0x7f, /* init: event generation + scanning enabled */
    .sensorcap = 0x68, /* capabilities: auto re-arm,*/
    .sensortype = SENSOR_TYPE_TEMPERATURE, /* sensor type */
    .event_reading_type = 0x01, /* sensor reading*/
    .assertion_event_mask = { 0xFF, /* LSB assert event mask: 3 bit value */
                              0x0F }, /* MSB assert event mask */
    .deassertion_event_mask = { 0xFF, /* LSB deassert event mask: 3 bit value */
                                0x0F }, /* MSB deassert event mask */
    .readable_threshold_mask = 0x3F, /* LSB: readabled Threshold mask: all thresholds are readabled:  */
    .settable_threshold_mask = 0x3F, /* MSB: setabled Threshold mask: all thresholds are setabled: */
    .sensor_units_1 = 0x00, /* sensor units 1 :*/
    .sensor_units_2 = 0x01, /* sensor units 2 :*/
    .sensor_units_3 = 0x00, /* sensor units 3 :*/
    .linearization = 0x00, /* Linearization */
    .M = 5, /* M */
    .M_tol = 0x00, /* M - Tolerance */
    .B = 0x00, /* B */
    .B_accuracy = 0x00, /* B - Accuracy */
    .acc_exp_sensor_dir = 0x00, /* Sensor direction */
    .Rexp_Bexp = 0xF0, /* R-Exp , B-Exp */
    .analog_flags = 0x03, /* Analogue characteristics flags */
    .nominal_reading = (20 << 1), /* Nominal reading */
    .normal_max = (50 << 1), /* Normal maximum */
    .normal_min = (10 << 1), /* Normal minimum */
    .sensor_max_reading = 0xFF, /* Sensor Maximum reading */
    .sensor_min_reading = 0x00, /* Sensor Minimum reading */
    .upper_nonrecover_thr = (80 << 1), /* Upper non-recoverable Threshold */
    .upper_critical_thr = (75 << 1), /* Upper critical Threshold */
    .upper_noncritical_thr = (65 << 1), /* Upper non critical Threshold */
    .lower_nonrecover_thr = (0 << 1), /* Lower non-recoverable Threshold */
    .lower_critical_thr = (5 << 1), /* Lower critical Threshold */
    .lower_noncritical_thr = (10 << 1), /* Lower non-critical Threshold */
    .pos_thr_hysteresis = 2, /* positive going Threshold hysteresis value */
    .neg_thr_hysteresis = 2, /* negative going Threshold hysteresis value */
    .reserved1 = 0x00, /* reserved */
    .reserved2 = 0x00, /* reserved */
    .OEM = 0x00, /* OEM reserved */
    .IDtypelen = 0xc0 | STR_SIZE("TEMP DDS_REF"), /* 8 bit ASCII, number of bytes */
    .IDstring = "TEMP DDS_REF" /* sensor string */
};

const SDR_type_01h_t SDR_TMP100_OXCO1 = {

    .hdr.recID_LSB = 0x00, /* Filled by sdr_insert_entry() */
    .hdr.recID_MSB = 0x00,
    .hdr.SDRversion = 0x51,
    .hdr.rectype = TYPE_01,
    .hdr.reclength = sizeof(SDR_type_01h_t) - sizeof(SDR_entry_hdr_t),

    .ownerID = 0x00, /* i2c address, -> SDR_Init */
    .ownerLUN = 0x00, /* sensor owner LUN */
    .sensornum = 0x00, /* Filled by sdr_insert_entry() */

    /* record body bytes */
    .entityID = 0xC1, /* entity id: AMC Module */
    .entityinstance = 0x00, /* entity instance -> SDR_Init */
    .sensorinit = 0x7f, /* init: event generation + scanning enabled */
    .sensorcap = 0x68, /* capabilities: auto re-arm,*/
    .sensortype = SENSOR_TYPE_TEMPERATURE, /* sensor type */
    .event_reading_type = 0x01, /* sensor reading*/
    .assertion_event_mask = { 0xFF, /* LSB assert event mask: 3 bit value */
                              0x0F }, /* MSB assert event mask */
    .deassertion_event_mask = { 0xFF, /* LSB deassert event mask: 3 bit value */
                                0x0F }, /* MSB deassert event mask */
    .readable_threshold_mask = 0x3F, /* LSB: readabled Threshold mask: all thresholds are readabled:  */
    .settable_threshold_mask = 0x3F, /* MSB: setabled Threshold mask: all thresholds are setabled: */
    .sensor_units_1 = 0x00, /* sensor units 1 :*/
    .sensor_units_2 = 0x01, /* sensor units 2 :*/
    .sensor_units_3 = 0x00, /* sensor units 3 :*/
    .linearization = 0x00, /* Linearization */
    .M = 5, /* M */
    .M_tol = 0x00, /* M - Tolerance */
    .B = 0x00, /* B */
    .B_accuracy = 0x00, /* B - Accuracy */
    .acc_exp_sensor_dir = 0x00, /* Sensor direction */
    .Rexp_Bexp = 0xF0, /* R-Exp , B-Exp */
    .analog_flags = 0x03, /* Analogue characteristics flags */
    .nominal_reading = (20 << 1), /* Nominal reading */
    .normal_max = (50 << 1), /* Normal maximum */
    .normal_min = (10 << 1), /* Normal minimum */
    .sensor_max_reading = 0xFF, /* Sensor Maximum reading */
    .sensor_min_reading = 0x00, /* Sensor Minimum reading */
    .upper_nonrecover_thr = (80 << 1), /* Upper non-recoverable Threshold */
    .upper_critical_thr = (75 << 1), /* Upper critical Threshold */
    .upper_noncritical_thr = (65 << 1), /* Upper non critical Threshold */
    .lower_nonrecover_thr = (0 << 1), /* Lower non-recoverable Threshold */
    .lower_critical_thr = (5 << 1), /* Lower critical Threshold */
    .lower_noncritical_thr = (10 << 1), /* Lower non-critical Threshold */
    .pos_thr_hysteresis = 2, /* positive going Threshold hysteresis value */
    .neg_thr_hysteresis = 2, /* negative going Threshold hysteresis value */
    .reserved1 = 0x00, /* reserved */
    .reserved2 = 0x00, /* reserved */
    .OEM = 0x00, /* OEM reserved */
    .IDtypelen = 0xc0 | STR_SIZE("TEMP OXCO1"), /* 8 bit ASCII, number of bytes */
    .IDstring = "TEMP OXCO1" /* sensor string */
};

const SDR_type_01h_t SDR_TMP100_OXCO2 = {

    .hdr.recID_LSB = 0x00, /* Filled by sdr_insert_entry() */
    .hdr.recID_MSB = 0x00,
    .hdr.SDRversion = 0x51,
    .hdr.rectype = TYPE_01,
    .hdr.reclength = sizeof(SDR_type_01h_t) - sizeof(SDR_entry_hdr_t),

    .ownerID = 0x00, /* i2c address, -> SDR_Init */
    .ownerLUN = 0x00, /* sensor owner LUN */
    .sensornum = 0x00, /* Filled by sdr_insert_entry() */

    /* record body bytes */
    .entityID = 0xC1, /* entity id: AMC Module */
    .entityinstance = 0x00, /* entity instance -> SDR_Init */
    .sensorinit = 0x7f, /* init: event generation + scanning enabled */
    .sensorcap = 0x68, /* capabilities: auto re-arm,*/
    .sensortype = SENSOR_TYPE_TEMPERATURE, /* sensor type */
    .event_reading_type = 0x01, /* sensor reading*/
    .assertion_event_mask = { 0xFF, /* LSB assert event mask: 3 bit value */
                              0x0F }, /* MSB assert event mask */
    .deassertion_event_mask = { 0xFF, /* LSB deassert event mask: 3 bit value */
                                0x0F }, /* MSB deassert event mask */
    .readable_threshold_mask = 0x3F, /* LSB: readabled Threshold mask: all thresholds are readabled:  */
    .settable_threshold_mask = 0x3F, /* MSB: setabled Threshold mask: all thresholds are setabled: */
    .sensor_units_1 = 0x00, /* sensor units 1 :*/
    .sensor_units_2 = 0x01, /* sensor units 2 :*/
    .sensor_units_3 = 0x00, /* sensor units 3 :*/
    .linearization = 0x00, /* Linearization */
    .M = 5, /* M */
    .M_tol = 0x00, /* M - Tolerance */
    .B = 0x00, /* B */
    .B_accuracy = 0x00, /* B - Accuracy */
    .acc_exp_sensor_dir = 0x00, /* Sensor direction */
    .Rexp_Bexp = 0xF0, /* R-Exp , B-Exp */
    .analog_flags = 0x03, /* Analogue characteristics flags */
    .nominal_reading = (20 << 1), /* Nominal reading */
    .normal_max = (50 << 1), /* Normal maximum */
    .normal_min = (10 << 1), /* Normal minimum */
    .sensor_max_reading = 0xFF, /* Sensor Maximum reading */
    .sensor_min_reading = 0x00, /* Sensor Minimum reading */
    .upper_nonrecover_thr = (80 << 1), /* Upper non-recoverable Threshold */
    .upper_critical_thr = (75 << 1), /* Upper critical Threshold */
    .upper_noncritical_thr = (65 << 1), /* Upper non critical Threshold */
    .lower_nonrecover_thr = (0 << 1), /* Lower non-recoverable Threshold */
    .lower_critical_thr = (5 << 1), /* Lower critical Threshold */
    .lower_noncritical_thr = (10 << 1), /* Lower non-critical Threshold */
    .pos_thr_hysteresis = 2, /* positive going Threshold hysteresis value */
    .neg_thr_hysteresis = 2, /* negative going Threshold hysteresis value */
    .reserved1 = 0x00, /* reserved */
    .reserved2 = 0x00, /* reserved */
    .OEM = 0x00, /* OEM reserved */
    .IDtypelen = 0xc0 | STR_SIZE("TEMP OXCO2"), /* 8 bit ASCII, number of bytes */
    .IDstring = "TEMP OXCO2" /* sensor string */
};
#endif

void amc_sdr_init( void )
{

#ifdef MODULE_TMP100
    /* Board Temperature */
    sdr_insert_entry( TYPE_01, (void *) &SDR_TMP100_PS,          &vTaskTMP100_Handle, 0, CHIP_ID_TMP100_PS         );
    sdr_insert_entry( TYPE_01, (void *) &SDR_TMP100_LO_RF,       &vTaskTMP100_Handle, 0, CHIP_ID_TMP100_LO_RF      );
    sdr_insert_entry( TYPE_01, (void *) &SDR_TMP100_DDS_LO,      &vTaskTMP100_Handle, 0, CHIP_ID_TMP100_DDS_LO     );
    sdr_insert_entry( TYPE_01, (void *) &SDR_TMP100_LTC6150,     &vTaskTMP100_Handle, 0, CHIP_ID_TMP100_LTC6150    );
    sdr_insert_entry( TYPE_01, (void *) &SDR_TMP100_REF_RF,      &vTaskTMP100_Handle, 0, CHIP_ID_TMP100_REF_RF     );
    sdr_insert_entry( TYPE_01, (void *) &SDR_TMP100_DDS_REF,     &vTaskTMP100_Handle, 0, CHIP_ID_TMP100_DDS_REF    );
    sdr_insert_entry( TYPE_01, (void *) &SDR_TMP100_OXCO1,       &vTaskTMP100_Handle, 0, CHIP_ID_TMP100_OXCO1      );
    sdr_insert_entry( TYPE_01, (void *) &SDR_TMP100_OXCO2,       &vTaskTMP100_Handle, 0, CHIP_ID_TMP100_OXCO2      );
    /* need support of I2C_BUS_TEMP2_ID bus... */
    /*sdr_insert_entry( TYPE_01, (void *) &SDR_TMP100_CKLA_FANOUT, &vTaskTMP100_Handle, 0, CHIP_ID_TMP100_CKLA_FANOUT);
    sdr_insert_entry( TYPE_01, (void *) &SDR_TMP100_CKLB_FANOUT, &vTaskTMP100_Handle, 0, CHIP_ID_TMP100_CKLB_FANOUT);*/
#endif

/* TODO: Add analog readings of P12V_DIV_17 and P3V3_DIV_17 */
}
