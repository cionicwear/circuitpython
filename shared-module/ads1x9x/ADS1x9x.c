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
// #include "shared-module/ads1x9x/ads_utils.h"
#include "lib/cionic/utils.h"
#include "shared-bindings/microcontroller/__init__.h"
#include "shared-bindings/time/__init__.h"
#include "shared-bindings/util.h"
#include "shared-bindings/digitalio/DigitalInOut.h"

#include "py/mperrno.h"
#include <string.h>
#include "py/stream.h"

#define ADS1x9x_BAUDRATE    (8000000)
// TS (4 Bytes) + (nb_chan * sizeof(float)) +  ADS1X9X_SIZE_STATUS_REG + spi_cmd (1 Byte)
#define TS_LEN              (sizeof(uint32_t))
#define MAX_BUF_LEN         ((ADS1X9X_NUM_CHAN * sizeof(float)) + ADS1X9X_SIZE_STATUS_REG + 1 + TS_LEN)

typedef struct ads_sample_t{
    uint32_t ts;
    float data[ADS1X9X_NUM_CHAN];
}ads_sample_t;

static ads_sample_t g_ads_sample;

// ads129x datasheet 9.4.1.3.3
// ads121x datasheet p.25 - Data Format
STATIC float ads_gain_norms[2][7] = {
    {12.207403790398877f, // gain 6
    73.244422742393262f, // 1
    36.622211371196631f, // 2
    24.414807580797754f, // 3
    18.311105685598316f, // 4
    9.155552842799158f, // 8
    6.103701895199439f},  // 12

    {0.047683721504655066f, // gain 6
    0.286102329027930368f, // 1
    0.143051164513965184f, // 2
    0.095367443009310132f, // 3
    0.071525582256982592f, // 4
    0.035762791128491296f, // 8
    0.023841860752327533f}  // 12
};

STATIC float ads_loff_currents[2][4] = {
    {4000,8000,12000,16000}, // ads119x datasheet [Lead-Off Control Register]
    {6000,12000,18000,24000} // ads129x datasheet 9.6.1.5
}; // picoA

// STATIC bool buffer_ready = false;

STATIC void lock_bus(ads1x9x_ADS1x9x_obj_t *self) {
    if (!common_hal_busio_spi_try_lock(self->bus)) {
        mp_raise_OSError(EAGAIN);
        return;
    }
}

STATIC void unlock_bus(ads1x9x_ADS1x9x_obj_t *self) {
    common_hal_busio_spi_unlock(self->bus);
}

STATIC void ads129x_config_update(ads1x9x_ADS1x9x_obj_t *self, uint8_t reg, const uint8_t val)
{
    // for (uint8_t reg = startreg; reg < startreg + nregs; reg++, index++) {
    if (reg >= ADS_CH1SET_REG && reg < ADS_CH1SET_REG + ADS1X9X_NUM_CHAN) {
        int ch = reg - ADS_CH1SET_REG;
        self->chan[ch] = val;
        if (val >> 7 == 0) {
            // ads129x datasheet 9.6.1.6
            uint8_t gain_index = (val >> 4) & 0x7;
            mp_printf(&mp_plat_print, "set gain to %d for channel %d\n", gain_index, ch);
            self->all_norms[ch] = self->norms[gain_index];
        }
    }
}

STATIC void ads129x_raw(ads1x9x_ADS1x9x_obj_t *self, uint8_t *in, float *out)
{
    uint8_t i = 0;
    int16_t ads_sample;

    for(i = 0 ; i < ADS1X9X_NUM_CHAN ; i++){
        ads_sample = READ_BE(int16_t, in);
        out[i] = (float)ads_sample * self->all_norms[i];
        in += 2;
    }
}

STATIC void ads129x_diff_filtered(ads1x9x_ADS1x9x_obj_t *self, uint8_t *in, float *out, uint16_t len)
{
    int numchans = len / 2; // data in is 16-bit
    uint32_t ts_out;

    if (diff_filter_process(&self->diff_filter, self->norms, numchans, 0, in, &ts_out, out) != 0) {
        return;
    }

}

STATIC void ads129x_iir_filtered(ads1x9x_ADS1x9x_obj_t *self, uint8_t *in, float *out, uint16_t len)
{
    int numchans = len / 2; // data in is 16-bit
    uint32_t ts_out;

    if (iir_filter_process(&self->iir_filter, self->norms, numchans, 0, in, &ts_out, out) != 0) {
        return;
    }

}

STATIC void data_ready_cb(void *arg) {
    static bool g_full = false;
    ads1x9x_ADS1x9x_obj_t *self = (ads1x9x_ADS1x9x_obj_t *)arg;
    self->lock = true;

    if(self->started == false){
        return;
    }

    g_ads_sample.ts = common_hal_time_monotonic_ns() / 100000;
    common_hal_ads1x9x_ADS1x9x_read_data(self, (uint8_t *)g_ads_sample.data, (self->num_chan * self->sample_bytes) + ADS1X9X_SIZE_STATUS_REG);

    if(cionic_ringbuf_write_sample(self->rb, &g_ads_sample, sizeof(ads_sample_t)) == false){
        if(g_full == false){
            g_full = true;
            mp_printf(&mp_plat_print, "ringbuf full!\n");
        }
    }else{
        g_full = false;
    }
    self->lock = false;
}

STATIC void ads1x9x_set_norms(ads1x9x_ADS1x9x_obj_t *self)
{
    if(self->id == ADS129X_DEV_ID){
        self->norms = ads_gain_norms[0];
        self->loff = ads_loff_currents[0];
    }else{ // ADS1X9X_DEV_ID
        self->norms = ads_gain_norms[1];
        self->loff = ads_loff_currents[1];
    }
}

void common_hal_ads1x9x_ADS1x9x_construct(ads1x9x_ADS1x9x_obj_t *self, busio_spi_obj_t *bus, const mcu_pin_obj_t *cs, const mcu_pin_obj_t *rst, const mcu_pin_obj_t *drdy, const mcu_pin_obj_t *start, const mcu_pin_obj_t *pwdn) {    
    self->bus = bus;
    self->started = false;
    self->num_chan = ADS1X9X_NUM_CHAN;
    self->sample_bytes = 0;
    self->filter = ADS1x9x_IIR_FILTER;
    self->lock = false;

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

    ads1x9x_set_norms(self);
    diff_filter_init(&self->diff_filter);
    iir_filter_init(&self->iir_filter);
    memset(&g_ads_sample, 0, sizeof(ads_sample_t));
    self->rb = cionic_ringbuf_alloc(sizeof(ads_sample_t), 400);
   

    if(self->rb == NULL){
        mp_raise_OSError(ENOMEM);
        return;
    }

    cionic_ringbuf_clear(self->rb);
}

void common_hal_ads1x9x_ADS1x9x_filter_set(ads1x9x_ADS1x9x_obj_t *self, uint8_t filt) {
    self->filter = filt;
    diff_filter_init(&self->diff_filter);
    iir_filter_init(&self->iir_filter);
    mp_printf(&mp_plat_print, "set filter to %d\n", self->filter);
}

uint16_t common_hal_ads1x9x_ADS1x9x_sample_size_get(ads1x9x_ADS1x9x_obj_t *self) {
    return (uint16_t)(self->num_chan * sizeof(float));
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

void common_hal_ads1x9x_ADS1x9x_start(ads1x9x_ADS1x9x_obj_t *self) {
    uint8_t wval = CMD_RDATAC;
    if(self->started){
        mp_raise_OSError(EAGAIN);
        return;
    }
    self->started = true;

    lock_bus(self);
    common_hal_digitalio_digitalinout_set_value(&self->cs, false);
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

    ads129x_config_update(self, addr, value);

    unlock_bus(self);
}

void common_hal_ads1x9x_ADS1x9x_read_data(ads1x9x_ADS1x9x_obj_t *self, uint8_t *data, uint16_t len) {
    uint8_t tx_buf[MAX_BUF_LEN] = {0};
    uint8_t rx_buf[MAX_BUF_LEN] = {0};

    tx_buf[0] = CMD_RDATA;

    lock_bus(self);
    common_hal_digitalio_digitalinout_set_value(&self->cs, false);
    common_hal_busio_spi_transfer(self->bus, tx_buf, rx_buf, len + 1);
    common_hal_digitalio_digitalinout_set_value(&self->cs, true);
    memcpy(rx_buf, &rx_buf[ADS1X9X_SIZE_STATUS_REG + 1], len - ADS1X9X_SIZE_STATUS_REG);

    if(self->filter == ADS1x9x_RAW){
        ads129x_raw(self, rx_buf, (float *)data);
    }else if(self->filter == ADS1x9x_DIFF_FILTER){
        ads129x_diff_filtered(self, rx_buf, (float *)data, len - ADS1X9X_SIZE_STATUS_REG);
    }else if(self->filter == ADS1x9x_IIR_FILTER){
        ads129x_iir_filtered(self, rx_buf, (float *)data, len - ADS1X9X_SIZE_STATUS_REG);
    }
    
    unlock_bus(self);
}

size_t common_hal_ads1x9x_ADS1x9x_read(ads1x9x_ADS1x9x_obj_t *self, mp_buffer_info_t *buf, uint16_t buf_size) {
    uint8_t *ptr = buf->buf;

    while(self->lock){
        mp_handle_pending(true);
        // Allow user to break out of a timeout with a KeyboardInterrupt.
        if (mp_hal_is_interrupted()) {
            return 0;
        }
    }

    return cionic_ringbuf_read_samples(self->rb, ptr, buf_size);
}
