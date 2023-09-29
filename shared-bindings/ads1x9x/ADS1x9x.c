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

#include "shared-bindings/ads129x/ADS129x.h"
#include "shared-module/ads129x/ADS129x.h"
#include "common-hal/busio/SPI.h"
#include "shared-bindings/busio/SPI.h"
#include "shared-bindings/microcontroller/Pin.h"
#include "supervisor/flash.h"

//| class ADS129x:
//|     """ADS129x Interface
//|
//|     Interacts with an ADS129x over SPI."""
//|
//|     def __init__(
//|         self, bus: busio.SPI, cs: microcontroller.Pin, rst: microcontroller.Pin, 
//|         drdy: microcontroller.Pin, start: microcontroller.Pin, pwdn: microcontroller.Pin
//|     ) -> None:
//|         """Construct an SPI ADS129x object with the given properties
//|
//|         :param busio.SPI spi: The SPI bus
//|         :param microcontroller.Pin cs: The SPI chip select
//|         :param microcontroller.Pin rst: The ADS129x reset pin
//|         :param microcontroller.Pin drdy: The ADS129x data ready pin
//|         :param microcontroller.Pin start: The ADS129x start pin
//|         :param microcontroller.Pin pwdn: The ADS129x power down pin
//|
//|         Example usage:
//|
//|         .. code-block:: python
//|
//|             import os
//|
//|             import board
//|             import ads129x
//|
//|             ads = ads129x.ADS129x(board.SPI(), board.ADS_CS, board.ADS_RST, board.ADS_DRDY, board.ADS_START, board.ADS_PWDN)

STATIC mp_obj_t ads129x_ads129x_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
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

    ads129x_ADS129x_obj_t *self = m_new_obj(ads129x_ADS129x_obj_t);
    self->base.type = &ads129x_ADS129x_type;

    common_hal_ads129x_ADS129x_construct(self, spi, cs, rst, drdy, start, pwdn);

    return self;
}

//|     def reset(self) -> None:
//|         """Reset the ADS129x
//|
//|         :return: None"""
STATIC mp_obj_t ads129x_ads129x_reset(mp_obj_t self_in) {
    ads129x_ADS129x_obj_t *self = (ads129x_ADS129x_obj_t *)self_in;
    common_hal_ads129x_ADS129x_reset(self);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(ads129x_ads129x_reset_obj, ads129x_ads129x_reset);

//|     def read_reg(self, address) -> int:
//|         """Read a ADS129x register
//|
//|         :param int address: The register address to read from
//|         :return: register value"""

STATIC mp_obj_t ads129x_ads129x_read_reg(mp_obj_t self_in, mp_obj_t reg_addr) {
    ads129x_ADS129x_obj_t *self = (ads129x_ADS129x_obj_t *)self_in;
    uint32_t addr = mp_obj_get_int(reg_addr);
    return mp_obj_new_int_from_uint(common_hal_ads129x_ADS129x_read_reg(self, (uint8_t)addr));
}
MP_DEFINE_CONST_FUN_OBJ_2(ads129x_ads129x_read_reg_obj, ads129x_ads129x_read_reg);

//|     def write_reg(self, address, value) -> None:
//|         """Write value to a ADS129x register
//|
//|         :param int address: The register address to write to
//|         :param int value: The value address to write
//|         :return: None"""

STATIC mp_obj_t ads129x_ads129x_write_reg(mp_obj_t self_in, mp_obj_t reg_addr, mp_obj_t value) {
    ads129x_ADS129x_obj_t *self = (ads129x_ADS129x_obj_t *)self_in;
    uint32_t addr = mp_obj_get_int(reg_addr);
    uint32_t val = mp_obj_get_int(value);
    common_hal_ads129x_ADS129x_write_reg(self, (uint8_t)addr, (uint8_t)val);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_3(ads129x_ads129x_write_reg_obj, ads129x_ads129x_write_reg);


//|     def start(self) -> None:
//|         """Start ADS129x sampling
//|
//|         :return: None"""

STATIC mp_obj_t ads129x_ads129x_start(mp_obj_t self_in, mp_obj_t sample_nb) {
    ads129x_ADS129x_obj_t *self = (ads129x_ADS129x_obj_t *)self_in;
    uint32_t smpl_nb = mp_obj_get_int(sample_nb);

    common_hal_ads129x_ADS129x_start(self, smpl_nb);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_2(ads129x_ads129x_start_obj, ads129x_ads129x_start);

//|     def stop(self) -> None:
//|         """Stop ADS129x sampling
//|
//|         :return: None"""

STATIC mp_obj_t ads129x_ads129x_stop(mp_obj_t self_in) {
    ads129x_ADS129x_obj_t *self = (ads129x_ADS129x_obj_t *)self_in;

    common_hal_ads129x_ADS129x_stop(self);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_1(ads129x_ads129x_stop_obj, ads129x_ads129x_stop);

//|     def read(self, buffer) -> int:
//|         """Read ADS129x data
//|
//|         :param buffer: Buffer to write data to
//|         :return: size read"""

STATIC mp_obj_t ads129x_ads129x_read(mp_obj_t self_in, mp_obj_t buf_in) {
    ads129x_ADS129x_obj_t *self = (ads129x_ADS129x_obj_t *)self_in;
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buf_in, &bufinfo, MP_BUFFER_WRITE);

    return mp_obj_new_int_from_uint((uint32_t)common_hal_ads129x_ADS129x_read(self, &bufinfo));
}

STATIC MP_DEFINE_CONST_FUN_OBJ_2(ads129x_ads129x_read_obj, ads129x_ads129x_read);

//|     def deinit(self) -> None:
//|         """Disable permanently.
//|
//|         :return: None"""
STATIC mp_obj_t ads129x_ads129x_deinit(mp_obj_t self_in) {
    ads129x_ADS129x_obj_t *self = (ads129x_ADS129x_obj_t *)self_in;
    common_hal_ads129x_ADS129x_deinit(self);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(ads129x_ads129x_deinit_obj, ads129x_ads129x_deinit);

STATIC const mp_rom_map_elem_t ads129x_ads129x_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_reset), MP_ROM_PTR(&ads129x_ads129x_reset_obj) },
    { MP_ROM_QSTR(MP_QSTR_read_reg), MP_ROM_PTR(&ads129x_ads129x_read_reg_obj) },
    { MP_ROM_QSTR(MP_QSTR_write_reg), MP_ROM_PTR(&ads129x_ads129x_write_reg_obj) },
    { MP_ROM_QSTR(MP_QSTR_start), MP_ROM_PTR(&ads129x_ads129x_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop), MP_ROM_PTR(&ads129x_ads129x_stop_obj) },
    // { MP_OBJ_NEW_QSTR(MP_QSTR_read), MP_ROM_PTR(&mp_stream_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_read), MP_ROM_PTR(&ads129x_ads129x_read_obj) },
};
STATIC MP_DEFINE_CONST_DICT(ads129x_ads129x_locals_dict, ads129x_ads129x_locals_dict_table);

const mp_obj_type_t ads129x_ADS129x_type = {
    { &mp_type_type },
    .name = MP_QSTR_ADS129x,
    .make_new = ads129x_ads129x_make_new,
    .locals_dict = (mp_obj_dict_t *)&ads129x_ads129x_locals_dict,
};
