/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 Scott Shawcroft
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef MICROPY_INCLUDED_SHARED_MODULE_ADS129X_ADS129X_H
#define MICROPY_INCLUDED_SHARED_MODULE_ADS129X_ADS129X_H

#include "py/obj.h"

#include "lib/cionic/diff_filter.h"
#include "lib/cionic/emg_iir.h"
#include "lib/cionic/ringbuf.h"

#include "common-hal/busio/SPI.h"
#include "common-hal/digitalio/DigitalInOut.h"

#define ADS129X_DEV_ID          0x92
#define ADS1198_DEV_ID          0xB6
#define ADS129X_SIZE_DATA_CHAN  3
#define ADS1198_SIZE_DATA_CHAN  2
#define ADS1X9X_SIZE_STATUS_REG 3
#define ADS1X9X_NUM_CHAN        8

typedef enum {
    ADS1x9x_RAW,
    ADS1x9x_DIFF_FILTER,
    ADS1x9x_IIR_FILTER,
} ads1x9x_filter_type_e;

typedef struct {
    mp_obj_base_t base;
    busio_spi_obj_t *bus;
    digitalio_digitalinout_obj_t cs;
    digitalio_digitalinout_obj_t rst;
    digitalio_digitalinout_obj_t start;
    digitalio_digitalinout_obj_t drdy;
    digitalio_digitalinout_obj_t pwdn;
    diff_filter_t diff_filter;
    iir_filter_t iir_filter;
    ringbuf_t *rb;
    uint32_t sample_bytes;
    bool started;
    bool lock;
    uint8_t id;
    uint8_t num_chan;
    uint8_t filter;
    float *norms;
    float *loff;
    float all_norms[ADS1X9X_NUM_CHAN]; // all channel norms
    uint8_t chan[ADS1X9X_NUM_CHAN];
} ads1x9x_ADS1x9x_obj_t;

// System Commands
#define CMD_WAKEUP              0x02
#define CMD_STANDBY             0x04
#define CMD_RESET               0x06
#define CMD_START               0x08
#define CMD_STOP                0x0A

// Data Read commands
#define CMD_RDATAC              0x10
#define CMD_SDATAC              0x11
#define CMD_RDATA               0x12

// Register read commands
#define CMD_RREG                0x20
#define CMD_WREG                0x40

// Registers
#define ADS_INFO_REG            0x00
#define ADS_CFG1_REG            0x01
#define ADS_CFG2_REG            0x02
#define ADS_CFG3_REG            0x03
#define ADS_LOFF_REG            0x04
#define ADS_CH1SET_REG          0x05
#define ADS_CH2SET_REG          0x06
#define ADS_CH3SET_REG          0x07
#define ADS_CH4SET_REG          0x08
#define ADS_CH5SET_REG          0x09
#define ADS_CH6SET_REG          0x0a
#define ADS_CH7SET_REG          0x0b
#define ADS_CH8SET_REG          0x0c
#define ADS_RLDP_REG            0x0d
#define ADS_RLDN_REG            0x0e
#define ADS_LOFFP_REG           0x0f
#define ADS_LOFFN_REG           0x10
#define ADS_LOFF_FLIP_REG       0x11
#define ADS_LOFF_STATP_REG      0x12
#define ADS_LOFF_STATN_REG      0x13
#define ADS_GPIO_REG            0x14
#define ADS_PACE_REG            0x15
#define ADS_RESP_REG            0x16
#define ADS_CFG4_REG            0x17
#define ADS_WCT1_REG            0x18
#define ADS_wct2_REG            0x19

void common_hal_ads1x9x_ADS1x9x_construct(ads1x9x_ADS1x9x_obj_t *self, busio_spi_obj_t *bus, const mcu_pin_obj_t *cs, const mcu_pin_obj_t *rst, const mcu_pin_obj_t *drdy, const mcu_pin_obj_t *start, const mcu_pin_obj_t *pwdn);
uint16_t common_hal_ads1x9x_ADS1x9x_sample_size_get(ads1x9x_ADS1x9x_obj_t *self);
void common_hal_ads1x9x_ADS1x9x_filter_set(ads1x9x_ADS1x9x_obj_t *self, uint8_t filt);
void common_hal_ads1x9x_ADS1x9x_reset(ads1x9x_ADS1x9x_obj_t *self);
void common_hal_ads1x9x_ADS1x9x_deinit(ads1x9x_ADS1x9x_obj_t *self);
void common_hal_ads1x9x_ADS1x9x_start(ads1x9x_ADS1x9x_obj_t *self);
void common_hal_ads1x9x_ADS1x9x_stop(ads1x9x_ADS1x9x_obj_t *self);
uint8_t common_hal_ads1x9x_ADS1x9x_read_reg(ads1x9x_ADS1x9x_obj_t *self, uint8_t addr);
void common_hal_ads1x9x_ADS1x9x_write_reg(ads1x9x_ADS1x9x_obj_t *self, uint8_t addr, uint8_t value);
void common_hal_ads1x9x_ADS1x9x_read_data(ads1x9x_ADS1x9x_obj_t *self, uint8_t *data, uint16_t len);
size_t common_hal_ads1x9x_ADS1x9x_read(ads1x9x_ADS1x9x_obj_t *self, mp_buffer_info_t *buf, uint16_t buf_size);

#endif // MICROPY_INCLUDED_SHARED_MODULE_ADS129X_ADS129X_H
