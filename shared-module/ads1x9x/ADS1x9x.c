/*
 * This file is part of the Micro Python project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2017 Scott Shawcroft for Adafruit Industries
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

#include "common-hal/microcontroller/Pin.h"
#include "shared-bindings/busio/SPI.h"
#include "shared-module/ads1x9x/ADS1x9x.h"
#include "shared-bindings/microcontroller/__init__.h"
#include "shared-bindings/time/__init__.h"
#include "shared-bindings/util.h"
#include "shared-bindings/digitalio/DigitalInOut.h"

#include "py/mperrno.h"
#include <string.h>
#include "py/stream.h"

#define ADS1x9x_BAUDRATE    (8000000)
#define MAX_BUF_LEN         (64)

STATIC bool buffer_ready = false;

STATIC void lock_bus(ads1x9x_ADS1x9x_obj_t *self) {
    if (!common_hal_busio_spi_try_lock(self->bus)) {
        mp_raise_OSError(EAGAIN);
        return;
    }
}

STATIC void unlock_bus(ads1x9x_ADS1x9x_obj_t *self) {
    common_hal_busio_spi_unlock(self->bus);
}

STATIC uint8_t sample_size(ads1x9x_ADS1x9x_obj_t *self) {
    return self->num_chan * self->sample_bytes + ADS1X9X_SIZE_STATUS_REG;
}

STATIC void data_ready_cb(void *arg) {
    ads1x9x_ADS1x9x_obj_t *self = (ads1x9x_ADS1x9x_obj_t *)arg;
    uint16_t data_len = sample_size(self);

    if(self->started == false || self->proc){
        return;
    }

    if(self->sample_cnt == self->sample_nb)
    {
        if(self->buf_idx == 0){
            self->buf_idx = 1;
        }else{
            self->buf_idx = 0;
        }
    }

    common_hal_ads1x9x_ADS1x9x_read_data(self, self->buf[self->buf_idx] + ((data_len - ADS1X9X_SIZE_STATUS_REG) * self->sample_cnt), data_len);
    self->sample_cnt++;

    if(self->sample_cnt == self->sample_nb){
        buffer_ready = true;
    }
}

void common_hal_ads1x9x_ADS1x9x_construct(ads1x9x_ADS1x9x_obj_t *self, busio_spi_obj_t *bus, const mcu_pin_obj_t *cs, const mcu_pin_obj_t *rst, const mcu_pin_obj_t *drdy, const mcu_pin_obj_t *start, const mcu_pin_obj_t *pwdn) {    
    self->bus = bus;
    self->started = false;
    self->buf_idx = 0;
    self->sample_cnt = 0;
    self->num_chan = ADS1X9X_NUM_CHAN;
    self->sample_nb = 0;
    self->sample_bytes = 0;
    self->proc = false;
    common_hal_digitalio_digitalinout_construct(&self->cs, cs);
    common_hal_digitalio_digitalinout_switch_to_output(&self->cs, true, DRIVE_MODE_PUSH_PULL);
    common_hal_digitalio_digitalinout_construct(&self->rst, rst);
    common_hal_digitalio_digitalinout_switch_to_output(&self->rst, false, DRIVE_MODE_PUSH_PULL);
    common_hal_digitalio_digitalinout_construct(&self->start, start);
    common_hal_digitalio_digitalinout_switch_to_output(&self->start, false, DRIVE_MODE_PUSH_PULL);
    common_hal_digitalio_digitalinout_construct(&self->pwdn, pwdn);
    common_hal_digitalio_digitalinout_switch_to_output(&self->pwdn, true, DRIVE_MODE_PUSH_PULL);
    common_hal_digitalio_digitalinout_construct(&self->drdy, drdy);
    common_hal_digitalio_digitalinout_set_irq(&self->drdy, EDGE_FALL, PULL_NONE, data_ready_cb, self);

    lock_bus(self);
    common_hal_busio_spi_configure(self->bus, ADS1x9x_BAUDRATE, 1, 0, 8);
    unlock_bus(self);

    common_hal_ads1x9x_ADS1x9x_reset(self);

    self->id = common_hal_ads1x9x_ADS1x9x_read_reg(self, ADS_INFO_REG);
    
    if(self->id == ADS129X_DEV_ID){
        self->sample_bytes = ADS129X_SIZE_DATA_CHAN;
    }else if(self->id == ADS1198_DEV_ID){
        self->sample_bytes = ADS1198_SIZE_DATA_CHAN;
    }else{
        mp_raise_OSError(ENODEV);
        return;
    }
    
    mp_printf(&mp_plat_print, "%s found\n", self->id == ADS129X_DEV_ID ? "ADS129X" : "ADS1198");
}

void common_hal_ads1x9x_ADS1x9x_reset(ads1x9x_ADS1x9x_obj_t *self) {
    common_hal_digitalio_digitalinout_set_value(&self->rst, true);
    common_hal_time_delay_ms(1000);
    common_hal_digitalio_digitalinout_set_value(&self->rst, false);
    common_hal_time_delay_ms(100);
    common_hal_digitalio_digitalinout_set_value(&self->rst, true);
    common_hal_time_delay_ms(200);
}

void common_hal_ads1x9x_ADS1x9x_deinit(ads1x9x_ADS1x9x_obj_t *self) {
    if (!self->bus) {
        return;
    }
    
    self->bus = 0;

    common_hal_digitalio_digitalinout_deinit(&self->cs);
    common_hal_digitalio_digitalinout_deinit(&self->rst);
    common_hal_digitalio_digitalinout_deinit(&self->drdy);
    common_hal_digitalio_digitalinout_deinit(&self->start);
    common_hal_digitalio_digitalinout_deinit(&self->pwdn);
    return;
}

void common_hal_ads1x9x_ADS1x9x_start(ads1x9x_ADS1x9x_obj_t *self, uint32_t sample_nb) {
    uint8_t wval = 0;
    if(self->started){
        mp_raise_OSError(EAGAIN);
        return;
    }
    self->started = true;
    self->sample_nb = sample_nb;
    self->buf[0] = m_malloc(sample_nb * sample_size(self), false);
    if(self->buf[0] == NULL){
        mp_raise_OSError(ENOMEM);
        return;
    }
    self->buf[1] = m_malloc(sample_nb * sample_size(self), false);
    if(self->buf[1] == NULL){
        m_free(self->buf[0]);
        mp_raise_OSError(ENOMEM);
        return;
    }
    self->buf_idx = 0;
    self->sample_cnt = 0;
    lock_bus(self);
    common_hal_digitalio_digitalinout_set_value(&self->cs, false);
    wval = CMD_RDATAC;
    common_hal_busio_spi_write(self->bus, &wval, 1);
    common_hal_digitalio_digitalinout_set_value(&self->cs, true);
    unlock_bus(self);
    common_hal_digitalio_digitalinout_set_value(&self->start, true);
}

void common_hal_ads1x9x_ADS1x9x_stop(ads1x9x_ADS1x9x_obj_t *self) {
    common_hal_digitalio_digitalinout_set_value(&self->start, false);
}

uint8_t common_hal_ads1x9x_ADS1x9x_read_reg(ads1x9x_ADS1x9x_obj_t *self, uint8_t addr) {
    uint8_t value = 0;
    uint8_t wval = 0;
    
    lock_bus(self);
    common_hal_digitalio_digitalinout_set_value(&self->cs, false);
    wval = CMD_SDATAC;
    common_hal_busio_spi_write(self->bus, &wval, 1);
    wval = addr | CMD_RREG;
    common_hal_busio_spi_write(self->bus, &wval, 1);
    wval = 0;
    common_hal_busio_spi_write(self->bus, &wval, 1);
    common_hal_busio_spi_read(self->bus, &value, 1, 0x00);
    common_hal_digitalio_digitalinout_set_value(&self->cs, true);
    unlock_bus(self);

    return value;
}

void common_hal_ads1x9x_ADS1x9x_write_reg(ads1x9x_ADS1x9x_obj_t *self, uint8_t addr, uint8_t value) {
    uint8_t wval = 0;

    lock_bus(self);
    common_hal_digitalio_digitalinout_set_value(&self->cs, false);
    wval = addr | CMD_WREG;
    common_hal_busio_spi_write(self->bus, &wval, 1);
    wval = 0;
    common_hal_busio_spi_write(self->bus, &wval, 1);
    common_hal_busio_spi_write(self->bus, &value, 1);
    common_hal_digitalio_digitalinout_set_value(&self->cs, true);
    unlock_bus(self);
}

void common_hal_ads1x9x_ADS1x9x_read_data(ads1x9x_ADS1x9x_obj_t *self, uint8_t *data, uint16_t len) {
    uint8_t tx_buf[MAX_BUF_LEN] = {0};
    uint8_t rx_buf[MAX_BUF_LEN] = {0};

    lock_bus(self);
    common_hal_digitalio_digitalinout_set_value(&self->cs, false);
    tx_buf[0] = CMD_RDATA;
    // common_hal_busio_spi_write(self->bus, &wval, 1);
    // common_hal_busio_spi_read(self->bus, data, len, 0);
    common_hal_busio_spi_transfer(self->bus, tx_buf, rx_buf, len + 1);
    common_hal_digitalio_digitalinout_set_value(&self->cs, true);
    memcpy(data, &rx_buf[ADS1X9X_SIZE_STATUS_REG + 1], len - ADS1X9X_SIZE_STATUS_REG);
    unlock_bus(self);
}

size_t common_hal_ads1x9x_ADS1x9x_read(ads1x9x_ADS1x9x_obj_t *self, mp_buffer_info_t *buf) {
    uint8_t idx = self->buf_idx;
    uint8_t *ptr = buf->buf;
    uint16_t nb = 0;

    while (buffer_ready == false) {
        mp_handle_pending(true);
        // Allow user to break out of a timeout with a KeyboardInterrupt.
        if (mp_hal_is_interrupted()) {
            return 0;
        }
    }
    self->proc = true;
    nb = self->sample_cnt;
    self->sample_cnt = 0;

    buffer_ready = false;
    // mp_printf(&mp_plat_print, "ads1x9x cpy %d samples from buf[%d]\n", nb, idx);
    memcpy(ptr, self->buf[idx], nb * (sample_size(self) - ADS1X9X_SIZE_STATUS_REG));
    self->proc = false;
    return nb * (sample_size(self) - ADS1X9X_SIZE_STATUS_REG);
}