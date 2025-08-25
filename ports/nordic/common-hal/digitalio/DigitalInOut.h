// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2017 Scott Shawcroft for Adafruit Industries
//
// SPDX-License-Identifier: MIT

#pragma once

#include "common-hal/microcontroller/Pin.h"

typedef void (io_irq_t)(void *arg);

typedef struct {
    mp_obj_base_t base;
    const mcu_pin_obj_t *pin;
    io_irq_t *cb;
    void *cb_arg;
} digitalio_digitalinout_obj_t;
