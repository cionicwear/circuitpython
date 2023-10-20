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

#include "shared-bindings/bno080/BNO080.h"
#include "shared-bindings/bno080/BNO080ReportId.h"
#include "shared-module/bno080/BNO080.h"
#include "common-hal/busio/SPI.h"
#include "shared-bindings/busio/SPI.h"
#include "shared-bindings/microcontroller/Pin.h"
#include "supervisor/flash.h"

//| class BNO080:
//|     """BNO080 Interface
//|
//|     Interacts with an BNO080 over SPI."""
//|
//|     def __init__(
//|         self, bus: busio.SPI, cs: microcontroller.Pin, rst: microcontroller.Pin, 
//|         drdy: microcontroller.Pin, start: microcontroller.Pin, pwdn: microcontroller.Pin
//|     ) -> None:
//|         """Construct an SPI BNO080 object with the given properties
//|
//|         :param busio.SPI spi: The SPI bus
//|         :param microcontroller.Pin cs: The SPI chip select
//|         :param microcontroller.Pin rst: The BNO080 reset pin
//|         :param microcontroller.Pin ps0: The BNO080 PS0/Wake pin
//|         :param microcontroller.Pin bootn: The BNO080 bootn pin
//|         :param microcontroller.Pin irq: The BNO080 interrupt pin
//|
//|         Example usage:
//|
//|         .. code-block:: python
//|
//|             import os
//|
//|             import board
//|             import bno080
//|
//|             bno = bno080.BNO080(board.SPI(), board.BNO_CS, board.BNO_RST, board.BNO_PS0, board.BNO_BOOTN, board.BNO_INT)

STATIC mp_obj_t bno080_bno080_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    enum { ARG_spi, ARG_cs, ARG_rst, ARG_ps0, ARG_bootn, ARG_irq, NUM_ARGS };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_spi, MP_ARG_OBJ, {.u_obj = mp_const_none } },
        { MP_QSTR_cs, MP_ARG_OBJ, {.u_obj = mp_const_none } },
        { MP_QSTR_rst, MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_ps0, MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_bootn, MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_irq, MP_ARG_OBJ, {.u_obj = mp_const_none} },
    };
    MP_STATIC_ASSERT(MP_ARRAY_SIZE(allowed_args) == NUM_ARGS);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    busio_spi_obj_t *spi = validate_obj_is_spi_bus(args[ARG_spi].u_obj, MP_QSTR_spi);
    const mcu_pin_obj_t *cs = validate_obj_is_free_pin(args[ARG_cs].u_obj, MP_QSTR_cs);
    const mcu_pin_obj_t *rst = validate_obj_is_free_pin(args[ARG_rst].u_obj, MP_QSTR_rst);
    const mcu_pin_obj_t *ps0 = validate_obj_is_free_pin(args[ARG_ps0].u_obj, MP_QSTR_ps0);
    const mcu_pin_obj_t *bootn = validate_obj_is_free_pin(args[ARG_bootn].u_obj, MP_QSTR_bootn);
    const mcu_pin_obj_t *irq = validate_obj_is_free_pin(args[ARG_irq].u_obj, MP_QSTR_irq);

    bno080_BNO080_obj_t *self = m_new_obj(bno080_BNO080_obj_t);
    self->base.type = &bno080_BNO080_type;

    common_hal_bno080_BNO080_construct(self, spi, cs, rst, ps0, bootn, irq);

    return self;
}

//|     def reset(self) -> None:
//|         """Reset the BNO080
//|
//|         :return: None"""
STATIC mp_obj_t bno080_BNO080_reset(mp_obj_t self_in) {
    bno080_BNO080_obj_t *self = (bno080_BNO080_obj_t *)self_in;
    common_hal_bno080_BNO080_reset(self);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(bno080_bno080_reset_obj, bno080_BNO080_reset);

//|     def deinit(self) -> None:
//|         """Disable permanently.
//|
//|         :return: None"""
STATIC mp_obj_t bno080_BNO080_deinit(mp_obj_t self_in) {
    bno080_BNO080_obj_t *self = (bno080_BNO080_obj_t *)self_in;
    common_hal_bno080_BNO080_deinit(self);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(bno080_bno080_deinit_obj, bno080_BNO080_deinit);

//|     def deinit(self) -> None:
//|         """Disable permanently.
//|
//|         :return: None"""

STATIC mp_obj_t bno080_BNO080_set_feature(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_feature, ARG_refresh_us, ARG_batch_us, ARG_flags, ARG_sns, ARG_cfg, NUM_ARGS };
    
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_feature,    MP_ARG_INT | MP_ARG_REQUIRED },
        { MP_QSTR_refresh_us, MP_ARG_INT | MP_ARG_REQUIRED },
        { MP_QSTR_batch_us,   MP_ARG_INT | MP_ARG_KW_ONLY, {.u_int = 0} },
        { MP_QSTR_flags,   MP_ARG_INT | MP_ARG_KW_ONLY, {.u_int = 0} },
        { MP_QSTR_sns,   MP_ARG_INT | MP_ARG_KW_ONLY, {.u_int = 0} },
        { MP_QSTR_cfg,   MP_ARG_INT | MP_ARG_KW_ONLY, {.u_int = 0} },
    };

    bno080_BNO080_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
    
    uint8_t feature = args[ARG_feature].u_int;
    uint32_t refresh_us = args[ARG_refresh_us].u_int;
    uint32_t batch_us = args[ARG_batch_us].u_int;
    uint8_t flags = args[ARG_flags].u_int;
    uint16_t sns = args[ARG_sns].u_int;
    uint32_t cfg = args[ARG_cfg].u_int;

    return mp_obj_new_int(common_hal_bno080_BNO080_set_feature(self, feature, refresh_us, batch_us, flags, sns, cfg));
}
MP_DEFINE_CONST_FUN_OBJ_KW(bno080_bno080_set_feature_obj, 1, bno080_BNO080_set_feature);


STATIC mp_obj_t bno080_bno080_read(mp_obj_t self_in, mp_obj_t id) {
    bno080_BNO080_obj_t *self = (bno080_BNO080_obj_t *)self_in;
    uint32_t report_id = mp_obj_get_int(id);

    return common_hal_bno080_BNO080_read(self, report_id);
}

MP_DEFINE_CONST_FUN_OBJ_2(bno080_bno080_read_obj, bno080_bno080_read);

STATIC const mp_rom_map_elem_t bno080_bno080_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_reset), MP_ROM_PTR(&bno080_bno080_reset_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_feature), MP_ROM_PTR(&bno080_bno080_set_feature_obj) },
    { MP_ROM_QSTR(MP_QSTR_read), MP_ROM_PTR(&bno080_bno080_read_obj) },
};
STATIC MP_DEFINE_CONST_DICT(bno080_bno080_locals_dict, bno080_bno080_locals_dict_table);

const mp_obj_type_t bno080_BNO080_type = {
    { &mp_type_type },
    .name = MP_QSTR_BNO080,
    .make_new = bno080_bno080_make_new,
    .locals_dict = (mp_obj_dict_t *)&bno080_bno080_locals_dict,
};
