/*
 * This file is part of the MicroPython project, http://micropython.org/
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


#include <stdint.h>

#include "py/obj.h"
#include "py/runtime.h"

#include "shared-bindings/bno080i2c/__init__.h"
#include "shared-bindings/bno080i2c/BNO080I2CReportId.h"
#include "shared-bindings/bno080i2c/BNO080I2C.h"
//| """Interface to an BNO080 via the I2C bus"""

STATIC const mp_rom_map_elem_t bno080i2c_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_bno080i2c) },
    // { MP_ROM_QSTR(MP_QSTR_BNO080ReportId),   MP_ROM_PTR(&bno080_bno080reportid_type) },
    { MP_ROM_QSTR(MP_QSTR_BNO080I2C), MP_ROM_PTR(&bno080i2c_BNO080I2C_type) },
};

STATIC MP_DEFINE_CONST_DICT(bno080i2c_module_globals, bno080i2c_module_globals_table);

const mp_obj_module_t bno080i2c_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&bno080i2c_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_bno080i2c, bno080i2c_module);
