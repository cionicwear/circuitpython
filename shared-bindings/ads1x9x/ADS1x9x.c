/*
 * This file is part of the Micro Python project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Jeff Epler for Adafruit Industries
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

#include "py/obj.h"
#include "py/objproperty.h"
#include "py/runtime.h"
#include "py/objarray.h"
#include "py/stream.h"

#include "shared-bindings/ads1x9x/ADS1x9x.h"
#include "shared-module/ads1x9x/ADS1x9x.h"
#include "common-hal/busio/SPI.h"
#include "shared-bindings/busio/SPI.h"
#include "shared-bindings/microcontroller/Pin.h"
#include "supervisor/flash.h"

//| class ADS1x9x:
//|     """ADS1x9x Interface
//|
//|     Interacts with an ADS1x9x over SPI."""
//|
//|     def __init__(
//|         self, bus: busio.SPI, cs: microcontroller.Pin, rst: microcontroller.Pin, 
//|         drdy: microcontroller.Pin, start: microcontroller.Pin, pwdn: microcontroller.Pin
//|     ) -> None:
//|         """Construct an SPI ADS1x9x object with the given properties
//|
//|         :param busio.SPI spi: The SPI bus
//|         :param microcontroller.Pin cs: The SPI chip select
//|         :param microcontroller.Pin rst: The ADS1x9x reset pin
//|         :param microcontroller.Pin drdy: The ADS1x9x data ready pin
//|         :param microcontroller.Pin start: The ADS1x9x start pin
//|         :param microcontroller.Pin pwdn: The ADS1x9x power down pin
//|
//|         Example usage:
//|
//|         .. code-block:: python
//|
//|             import os
//|
//|             import board
//|             import ads1x9x
//|
//|             ads = ads1x9x.ADS1x9x(board.SPI(), board.ADS_CS, board.ADS_RST, board.ADS_DRDY, board.ADS_START, board.ADS_PWDN)

STATIC mp_obj_t ads1x9x_ads1x9x_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    enum { ARG_spi, ARG_cs, ARG_rst, ARG_drdy, ARG_start, ARG_pwdn, NUM_ARGS };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_spi, MP_ARG_OBJ, {.u_obj = mp_const_none } },
        { MP_QSTR_cs, MP_ARG_OBJ, {.u_obj = mp_const_none } },
        { MP_QSTR_rst, MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_drdy, MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_start, MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_pwdn, MP_ARG_OBJ, {.u_obj = mp_const_none} },
    };
    MP_STATIC_ASSERT(MP_ARRAY_SIZE(allowed_args) == NUM_ARGS);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    busio_spi_obj_t *spi = validate_obj_is_spi_bus(args[ARG_spi].u_obj, MP_QSTR_spi);
    const mcu_pin_obj_t *cs = validate_obj_is_free_pin(args[ARG_cs].u_obj, MP_QSTR_cs);
    const mcu_pin_obj_t *rst = validate_obj_is_free_pin(args[ARG_rst].u_obj, MP_QSTR_rst);
    const mcu_pin_obj_t *drdy = validate_obj_is_free_pin(args[ARG_drdy].u_obj, MP_QSTR_drdy);
    const mcu_pin_obj_t *start = validate_obj_is_free_pin(args[ARG_start].u_obj, MP_QSTR_start);
    const mcu_pin_obj_t *pwdn = validate_obj_is_free_pin(args[ARG_pwdn].u_obj, MP_QSTR_pwdn);

    ads1x9x_ADS1x9x_obj_t *self = m_new_obj(ads1x9x_ADS1x9x_obj_t);
    self->base.type = &ads1x9x_ADS1x9x_type;

    common_hal_ads1x9x_ADS1x9x_construct(self, spi, cs, rst, drdy, start, pwdn);

    return self;
}

//|     def reset(self) -> None:
//|         """Reset the ADS1x9x
//|
//|         :return: None"""
STATIC mp_obj_t ads1x9x_ads1x9x_reset(mp_obj_t self_in) {
    ads1x9x_ADS1x9x_obj_t *self = (ads1x9x_ADS1x9x_obj_t *)self_in;
    common_hal_ads1x9x_ADS1x9x_reset(self);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(ads1x9x_ads1x9x_reset_obj, ads1x9x_ads1x9x_reset);

//|     def sample_size_get(self) -> None:
//|         """Get the ADS1x9x sample size
//|
//|         :return: Sample size"""
STATIC mp_obj_t ads1x9x_ads1x9x_sample_size_get(mp_obj_t self_in) {
    ads1x9x_ADS1x9x_obj_t *self = (ads1x9x_ADS1x9x_obj_t *)self_in;
    return mp_obj_new_int_from_uint(common_hal_ads1x9x_ADS1x9x_sample_size_get(self));
}
MP_DEFINE_CONST_FUN_OBJ_1(ads1x9x_ads1x9x_sample_size_get_obj, ads1x9x_ads1x9x_sample_size_get);

//|     def filter_set(self, filter) -> None:
//|         """Set filter type for ADS1x9x
//|
//|         :param int filter: The filter enum to write
//|         :return: None"""

STATIC mp_obj_t ads1x9x_ads1x9x_filter_set(mp_obj_t self_in, mp_obj_t filter) {
    ads1x9x_ADS1x9x_obj_t *self = (ads1x9x_ADS1x9x_obj_t *)self_in;
    uint32_t filt = mp_obj_get_int(filter);
    common_hal_ads1x9x_ADS1x9x_filter_set(self, (uint8_t)filt);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_2(ads1x9x_ads1x9x_filter_set_obj, ads1x9x_ads1x9x_filter_set);

//|     def read_reg(self, address) -> int:
//|         """Read a ADS1x9x register
//|
//|         :param int address: The register address to read from
//|         :return: register value"""

STATIC mp_obj_t ads1x9x_ads1x9x_read_reg(mp_obj_t self_in, mp_obj_t reg_addr) {
    ads1x9x_ADS1x9x_obj_t *self = (ads1x9x_ADS1x9x_obj_t *)self_in;
    uint32_t addr = mp_obj_get_int(reg_addr);
    return mp_obj_new_int_from_uint(common_hal_ads1x9x_ADS1x9x_read_reg(self, (uint8_t)addr));
}
MP_DEFINE_CONST_FUN_OBJ_2(ads1x9x_ads1x9x_read_reg_obj, ads1x9x_ads1x9x_read_reg);

//|     def write_reg(self, address, value) -> None:
//|         """Write value to a ADS1x9x register
//|
//|         :param int address: The register address to write to
//|         :param int value: The value address to write
//|         :return: None"""

STATIC mp_obj_t ads1x9x_ads1x9x_write_reg(mp_obj_t self_in, mp_obj_t reg_addr, mp_obj_t value) {
    ads1x9x_ADS1x9x_obj_t *self = (ads1x9x_ADS1x9x_obj_t *)self_in;
    uint32_t addr = mp_obj_get_int(reg_addr);
    uint32_t val = mp_obj_get_int(value);
    common_hal_ads1x9x_ADS1x9x_write_reg(self, (uint8_t)addr, (uint8_t)val);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_3(ads1x9x_ads1x9x_write_reg_obj, ads1x9x_ads1x9x_write_reg);


//|     def start(self) -> None:
//|         """Start ADS1x9x sampling
//|
//|         :return: None"""

STATIC mp_obj_t ads1x9x_ads1x9x_start(mp_obj_t self_in) {
    ads1x9x_ADS1x9x_obj_t *self = (ads1x9x_ADS1x9x_obj_t *)self_in;

    common_hal_ads1x9x_ADS1x9x_start(self);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_1(ads1x9x_ads1x9x_start_obj, ads1x9x_ads1x9x_start);

//|     def stop(self) -> None:
//|         """Stop ADS1x9x sampling
//|
//|         :return: None"""

STATIC mp_obj_t ads1x9x_ads1x9x_stop(mp_obj_t self_in) {
    ads1x9x_ADS1x9x_obj_t *self = (ads1x9x_ADS1x9x_obj_t *)self_in;

    common_hal_ads1x9x_ADS1x9x_stop(self);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_1(ads1x9x_ads1x9x_stop_obj, ads1x9x_ads1x9x_stop);

//|     def read(self, buffer) -> int:
//|         """Read ADS1x9x data
//|
//|         :param buffer: Buffer to write data to
//|         :return: size read"""

STATIC mp_obj_t ads1x9x_ads1x9x_read(mp_obj_t self_in, mp_obj_t buf_in, mp_obj_t buf_size) {
    ads1x9x_ADS1x9x_obj_t *self = (ads1x9x_ADS1x9x_obj_t *)self_in;
    mp_buffer_info_t bufinfo;
    uint32_t buf_sz = mp_obj_get_int(buf_size);
    mp_get_buffer_raise(buf_in, &bufinfo, MP_BUFFER_WRITE);

    return mp_obj_new_int_from_uint((uint32_t)common_hal_ads1x9x_ADS1x9x_read(self, &bufinfo, buf_sz));
}

STATIC MP_DEFINE_CONST_FUN_OBJ_3(ads1x9x_ads1x9x_read_obj, ads1x9x_ads1x9x_read);

//|     def deinit(self) -> None:
//|         """Disable permanently.
//|
//|         :return: None"""
STATIC mp_obj_t ads1x9x_ads1x9x_deinit(mp_obj_t self_in) {
    ads1x9x_ADS1x9x_obj_t *self = (ads1x9x_ADS1x9x_obj_t *)self_in;
    common_hal_ads1x9x_ADS1x9x_deinit(self);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(ads1x9x_ads1x9x_deinit_obj, ads1x9x_ads1x9x_deinit);

STATIC mp_obj_t ads1x9x_ads1x9x_set_emg_filter(mp_obj_t self_in, mp_obj_t coeffs_list, mp_obj_t low) {
    ads1x9x_ADS1x9x_obj_t *self = (ads1x9x_ADS1x9x_obj_t *) self_in;
    set_emg_filter(self, coeffs_list, low);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_3(ads1x9x_ads1x9x_set_emg_filter_obj, ads1x9x_ads1x9x_set_emg_filter);

STATIC mp_obj_t ads1x9x_ads1x9x_set_emg_decim_rate(mp_obj_t self_in, mp_obj_t decim_rate) {
    ads1x9x_ADS1x9x_obj_t *self = (ads1x9x_ADS1x9x_obj_t *) self_in;
    set_emg_decim_rate(self, decim_rate);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_2(ads1x9x_ads1x9x_set_emg_decim_rate_obj, ads1x9x_ads1x9x_set_emg_decim_rate);

STATIC const mp_rom_map_elem_t ads1x9x_ads1x9x_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_reset), MP_ROM_PTR(&ads1x9x_ads1x9x_reset_obj) },
    { MP_ROM_QSTR(MP_QSTR_sample_size_get), MP_ROM_PTR(&ads1x9x_ads1x9x_sample_size_get_obj) },
    { MP_ROM_QSTR(MP_QSTR_filter_set), MP_ROM_PTR(&ads1x9x_ads1x9x_filter_set_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_emg_filter), MP_ROM_PTR(&ads1x9x_ads1x9x_set_emg_filter_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_emg_decim_rate), MP_ROM_PTR(&ads1x9x_ads1x9x_set_emg_decim_rate_obj) },
    { MP_ROM_QSTR(MP_QSTR_read_reg), MP_ROM_PTR(&ads1x9x_ads1x9x_read_reg_obj) },
    { MP_ROM_QSTR(MP_QSTR_write_reg), MP_ROM_PTR(&ads1x9x_ads1x9x_write_reg_obj) },
    { MP_ROM_QSTR(MP_QSTR_start), MP_ROM_PTR(&ads1x9x_ads1x9x_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop), MP_ROM_PTR(&ads1x9x_ads1x9x_stop_obj) },
    // { MP_OBJ_NEW_QSTR(MP_QSTR_read), MP_ROM_PTR(&mp_stream_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_read), MP_ROM_PTR(&ads1x9x_ads1x9x_read_obj) },
};
STATIC MP_DEFINE_CONST_DICT(ads1x9x_ads1x9x_locals_dict, ads1x9x_ads1x9x_locals_dict_table);

const mp_obj_type_t ads1x9x_ADS1x9x_type = {
    { &mp_type_type },
    .name = MP_QSTR_ADS1x9x,
    .make_new = ads1x9x_ads1x9x_make_new,
    .locals_dict = (mp_obj_dict_t *)&ads1x9x_ads1x9x_locals_dict,
};
