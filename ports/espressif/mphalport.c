// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2015 Glenn Ruben Bakke
// SPDX-FileCopyrightText: Copyright (c) 2018 Artur Pacholec
//
// SPDX-License-Identifier: MIT

#include "py/mphal.h"
#include "supervisor/cpu.h"

#include "rom/ets_sys.h"

#include "esp_attr.h"

// This is used by ProtoMatter's interrupt so make sure it is available when
// flash isn't.
void IRAM_ATTR mp_hal_delay_us(mp_uint_t delay) {
    ets_delay_us(delay);
}

// This is provided by the esp-idf/components/xtensa/esp32s2/libhal.a binary blob.
#ifndef CONFIG_IDF_TARGET_ARCH_RISCV
extern void xthal_window_spill(void);

mp_uint_t cpu_get_regs_and_sp(mp_uint_t *regs) {
    // xtensa has more registers than an instruction can address. The 16 that
    // can be addressed are called the "window". When a function is called or
    // returns the window rotates. This allows for more efficient function calls
    // because ram doesn't need to be used. It's only used if the window wraps
    // around onto itself. At that point values are "spilled" to empty spots in
    // the stack that were set aside. When the window rotates back around (on
    // function return), the values are restored into the register from ram.

    // So, in order to read the values in the stack scan we must make sure all
    // of the register values we care about have been spilled to RAM. Luckily,
    // there is a HAL call to do it. There is a bit of a race condition here
    // because the register value could change after it's been restored but that
    // is unlikely to happen with a heap pointer while we do a GC.
    xthal_window_spill();
    return (mp_uint_t)__builtin_stack_address();
}
#endif
