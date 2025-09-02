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

#include "shared-bindings/bno080i2c/BNO080I2C.h"
#include "shared-bindings/bno080i2c/BNO080I2CReportId.h"
#include "shared-module/bno080i2c/BNO080I2C.h"
#include "common-hal/busio/I2C.h"
#include "shared-bindings/busio/I2C.h"
#include "common-hal/busio/SPI.h"
#include "shared-bindings/busio/SPI.h"
#include "shared-bindings/microcontroller/Pin.h"
#include "supervisor/flash.h"

//| class BNO080I2C:
//|     """
//|     BNO080I2C Interface
//|
//|     Interacts with an BNO080 over I2C.
//|     """
//|
//|     def __init__(self, bus: busio.I2C, addr: int = 0x4B, debug: bool = False) -> None:
//|         """
//|         Construct a BNO080I2C object with the given properties
//|
//|         :param busio.I2C i2c: The I2C bus
//|         :param int addr: The BNO080 I2C address (optional, default=0x4B)
//|         :param bool debug: Enable debugging output (optional, default=False)
//|
//|         Example usage:
//|
//|         .. code-block:: python
//|
//|             import os
//|
//|             import board
//|             import bno080i2c
//|
//|             bno = bno080i2c.BNO080I2C(board.I2C(), board.ADDR, debug=False)
//|         """
STATIC mp_obj_t bno080i2c_bno080i2c_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    enum { ARG_i2c, ARG_addr,
           // ARG_rst, ARG_ps0, ARG_bootn, ARG_irq,
           ARG_debug, NUM_ARGS };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_i2c, MP_ARG_OBJ, {.u_obj = mp_const_none } },
        { MP_QSTR_addr, MP_ARG_INT, {.u_int = 0x4B} },
        // { MP_QSTR_rst, MP_ARG_OBJ, {.u_obj = mp_const_none} },
        // { MP_QSTR_ps0, MP_ARG_OBJ, {.u_obj = mp_const_none} },
        // { MP_QSTR_bootn, MP_ARG_OBJ, {.u_obj = mp_const_none} },
        // { MP_QSTR_irq, MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_debug, MP_ARG_BOOL, {.u_bool = false} },
    };
    MP_STATIC_ASSERT(MP_ARRAY_SIZE(allowed_args) == NUM_ARGS);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // busio_i2c_obj_t *i2c = validate_obj_is_i2c_bus(args[ARG_i2c].u_obj, MP_QSTR_i2c);
    busio_i2c_obj_t *i2c = MP_OBJ_TO_PTR(args[ARG_i2c].u_obj);
    int8_t addr = args[ARG_addr].u_int;
    if (common_hal_busio_i2c_probe(i2c, addr) == false) {
        mp_raise_OSError(ENODEV);
    }
    // const mcu_pin_obj_t *rst = validate_obj_is_free_pin(args[ARG_rst].u_obj, MP_QSTR_rst);
    // const mcu_pin_obj_t *ps0 = validate_obj_is_free_pin(args[ARG_ps0].u_obj, MP_QSTR_ps0);
    // const mcu_pin_obj_t *bootn = validate_obj_is_free_pin(args[ARG_bootn].u_obj, MP_QSTR_bootn);
    // const mcu_pin_obj_t *irq = validate_obj_is_free_pin(args[ARG_irq].u_obj, MP_QSTR_irq);
    bool debug = args[ARG_debug].u_bool;

    if (debug) {
        mp_printf(&mp_plat_print, "Called bno080i2c_BNO080I2C_make_new\n");
    }

    bno080i2c_BNO080I2C_obj_t *self = m_new_obj(bno080i2c_BNO080I2C_obj_t);
    self->base.type = &bno080i2c_BNO080I2C_type;

    common_hal_bno080i2c_BNO080I2C_construct(self, i2c, addr, debug);
    if (debug) {
        mp_printf(&mp_plat_print, "Constructed bno080i2c_BNO080I2C\n");
    }

    return self;
}

//|     def reset(self) -> None:
//|         """Reset the BNO080
//|
//|         :return: None"""
STATIC mp_obj_t bno080i2c_BNO080I2C_reset(mp_obj_t self_in) {
    if (((bno080i2c_BNO080I2C_obj_t *)self_in)->debug) {
        mp_printf(&mp_plat_print, "Called bno080i2c_BNO080I2C_reset\n");
    }

    bno080i2c_BNO080I2C_obj_t *self = (bno080i2c_BNO080I2C_obj_t *)self_in;
    common_hal_bno080i2c_BNO080I2C_soft_reset(self);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(bno080i2c_bno080i2c_reset_obj, bno080i2c_BNO080I2C_reset);

//|     def deinit(self) -> None:
//|         """Disable permanently.
//|
//|         :return: None"""
STATIC mp_obj_t bno080i2c_BNO080I2C_deinit(mp_obj_t self_in) {
    if (((bno080i2c_BNO080I2C_obj_t *)self_in)->debug) {
        mp_printf(&mp_plat_print, "Called bno080i2c_BNO080I2C_deinit\n");
    }

    bno080i2c_BNO080I2C_obj_t *self = (bno080i2c_BNO080I2C_obj_t *)self_in;
    common_hal_bno080i2c_BNO080I2C_deinit(self);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(bno080i2c_bno080i2c_deinit_obj, bno080i2c_BNO080I2C_deinit);

//|     def set_feature(self) -> None:
//|         """Set the feature configuration.
//|
//|         :param feature: The feature to set
//|         :param refresh_us: The refresh rate in microseconds
//|         :param batch_us: The batch rate in microseconds
//|         :param flags: Flags to set
//|         :param sns: Sensor ID
//|         :param cfg: Configuration value
//|
//|         :return: None"""
STATIC mp_obj_t bno080i2c_BNO080I2C_set_feature(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    bno080i2c_BNO080I2C_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    if (self->debug) {
        mp_printf(&mp_plat_print, "Called bno080i2c_BNO080I2C_set_feature\n");
    }

    enum { ARG_feature, ARG_refresh_us, ARG_batch_us, ARG_flags, ARG_sns, ARG_cfg, NUM_ARGS };

    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_feature,    MP_ARG_INT | MP_ARG_REQUIRED },
        { MP_QSTR_refresh_us, MP_ARG_INT | MP_ARG_REQUIRED },
        { MP_QSTR_batch_us,   MP_ARG_INT | MP_ARG_KW_ONLY, {.u_int = 0} },
        { MP_QSTR_flags,   MP_ARG_INT | MP_ARG_KW_ONLY, {.u_int = 0} },
        { MP_QSTR_sns,   MP_ARG_INT | MP_ARG_KW_ONLY, {.u_int = 0} },
        { MP_QSTR_cfg,   MP_ARG_INT | MP_ARG_KW_ONLY, {.u_int = 0} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    uint8_t feature = args[ARG_feature].u_int;
    uint32_t refresh_us = args[ARG_refresh_us].u_int;
    uint32_t batch_us = args[ARG_batch_us].u_int;
    uint8_t flags = args[ARG_flags].u_int;
    uint16_t sns = args[ARG_sns].u_int;
    uint32_t cfg = args[ARG_cfg].u_int;

    return mp_obj_new_int(common_hal_bno080i2c_BNO080I2C_set_feature(self, feature, refresh_us, batch_us, flags, sns, cfg));
}
MP_DEFINE_CONST_FUN_OBJ_KW(bno080i2c_bno080i2c_set_feature_obj, 1, bno080i2c_BNO080I2C_set_feature);


//|     def read(self) -> None:
//|         """Read data from the BNO080
//|
//|         :param id: The report ID to read
//|
//|         :return: The report data"""
//|
STATIC mp_obj_t bno080i2c_bno080i2c_read(mp_obj_t self_in, mp_obj_t id) {
    // print that read is called
    bno080i2c_BNO080I2C_obj_t *self = (bno080i2c_BNO080I2C_obj_t *)self_in;
    if (self->debug) {
        mp_printf(&mp_plat_print, "Called bno080i2c_BNO080I2C_read\n");
    }
    uint32_t report_id = mp_obj_get_int(id);

    return common_hal_bno080i2c_BNO080I2C_read(self, report_id);
}

MP_DEFINE_CONST_FUN_OBJ_2(bno080i2c_bno080i2c_read_obj, bno080i2c_bno080i2c_read);

STATIC const mp_rom_map_elem_t bno080i2c_bno080i2c_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_reset), MP_ROM_PTR(&bno080i2c_bno080i2c_reset_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_feature), MP_ROM_PTR(&bno080i2c_bno080i2c_set_feature_obj) },
    { MP_ROM_QSTR(MP_QSTR_read), MP_ROM_PTR(&bno080i2c_bno080i2c_read_obj) },
};
STATIC MP_DEFINE_CONST_DICT(bno080i2c_bno080i2c_locals_dict, bno080i2c_bno080i2c_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    bno080i2c_BNO080I2C_type,
    MP_QSTR_BNO080I2C,
    MP_TYPE_FLAG_NONE,
    make_new, bno080i2c_bno080i2c_make_new,
    locals_dict, &bno080i2c_bno080i2c_locals_dict
    );
