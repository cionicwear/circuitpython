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

#ifndef MICROPY_INCLUDED_SHARED_MODULE_BNO080_BNO080_H
#define MICROPY_INCLUDED_SHARED_MODULE_BNO080_BNO080_H

#include "py/obj.h"

#include "lib/cionic/utils.h"

#include "common-hal/busio/SPI.h"
#include "common-hal/digitalio/DigitalInOut.h"

#include "BNO080_reg.h"

#define BNO080_HEADER_SIZE 4
#define BNO080_BASE_DELTA_OFFSET 1
#define BNO080_PACKET_SIZE 32
#define BNO080_SRID_OFFSET 5
#define BNO080_STATUS_DELAY_OFFSET 2
#define BNO080_PRODUCT_ID_RESPONSE 0xF8

#define BNO080_FRS_READ_NO_ERROR 0
#define BNO080_FRS_READ_COMPLETE 3
#define BNO080_FRS_READ_EMPTY 5
#define BNO080_FRS_WRITE_COMPLETE 3
#define BNO080_FRS_WRITE_START -1
#define LE_U16(V) ((V) & 0xFF), (((V) >> 8) & 0xFF)
#define LE_U32(V) ((V) & 0xFF), (((V) >> 8) & 0xFF), (((V) >> 16) & 0xFF), (((V) >> 24) & 0xFF)

#define BNO080_CHANNEL_COMMAND     0x00
#define BNO080_CHANNEL_EXECUTE     0x01
#define BNO080_CHANNEL_CONTROL     0x02
#define BNO080_CHANNEL_REPORT      0x03
#define BNO080_CHANNEL_WAKE        0x04
#define BNO080_CHANNEL_GYRO        0x05

#define BNO080_NUM_CHANNELS        6
#define BNO080_MAX_TX              200
#define BNO080_MAX_RX              200

#define BNO080_FRS_SLOTS           4

#define CALIBRATION_LEN     (5)

#define QUAT_DIMENSION      (4)
#define ACCEL_DIMENSION     (3)
#define GYRO_DIMENSION      (3)
#define MAG_DIMENSION       (3)
#define GRAV_DIMENSION      (3)

enum {
    BNO080_QUAT_FLOAT=0,
    BNO080_ACCEL_FLOAT,
    BNO080_GYRO_FLOAT,
    BNO080_GRAV_FLOAT,
    BNO080_SHTP,
    BNO080_FRS,
};

enum {
    BNO080_ACCURACY_ACCEL=0,
    BNO080_ACCURACY_GYRO,
    BNO080_ACCURACY_MAG,
    BNO080_ACCURACY_COUNT
};

typedef struct bno080_pid_t {
    uint8_t id;
    uint8_t reset_cause;
    uint8_t sw_ver_major;
    uint8_t sw_ver_minor;
    uint32_t sw_part_number;
    uint32_t sw_build_number;
    uint16_t sw_version_patch;
} bno080_pid_t;

typedef struct bno080_frs_t {
    uint16_t id;
    uint16_t offset;
    uint32_t data0;
    uint32_t data1;
} bno080_frs_t;

typedef struct {
    mp_obj_base_t base;
    busio_spi_obj_t *bus;
    bno080_pid_t pid;
    digitalio_digitalinout_obj_t cs;
    digitalio_digitalinout_obj_t rst;
    digitalio_digitalinout_obj_t ps0;
    digitalio_digitalinout_obj_t bootn;
    digitalio_digitalinout_obj_t irq;
    bno080_frs_t frs_saved[BNO080_FRS_SLOTS];
    uint8_t read_seqnums[BNO080_NUM_CHANNELS];
    uint8_t write_seqnums[BNO080_NUM_CHANNELS];
    // float quat[QUAT_DIMENSION];        // most recent quaternion sample
    mp_obj_t fquat[QUAT_DIMENSION];
    mp_obj_t accel[ACCEL_DIMENSION];
    mp_obj_t gyro[GYRO_DIMENSION];
    mp_obj_t mag[MAG_DIMENSION];
    mp_obj_t grav[GRAV_DIMENSION];
    float calibration[CALIBRATION_LEN]; // calibration data
    uint8_t accuracy[BNO080_ACCURACY_COUNT];
    int selected_rotation;
    int frs_read;
    int frs_write;
    int frs_write_offset;
    uint8_t txbuf[BNO080_MAX_TX];
    uint8_t rxbuf[BNO080_MAX_RX];
    int16_t txlen;
    elapsed_t last_timestamp; // 100us since boot
    uint8_t resp;
    bool init_done;
} bno080_BNO080_obj_t;

void common_hal_bno080_BNO080_construct(bno080_BNO080_obj_t *self, busio_spi_obj_t *bus, const mcu_pin_obj_t *cs, const mcu_pin_obj_t *rst, const mcu_pin_obj_t *ps0, const mcu_pin_obj_t *bootn, const mcu_pin_obj_t *irq);
void common_hal_bno080_BNO080_reset(bno080_BNO080_obj_t *self);
int common_hal_bno080_BNO080_set_feature(bno080_BNO080_obj_t *self, uint8_t feature, uint32_t refresh_us, uint32_t batch_us, uint8_t flags, uint16_t sns, uint32_t cfg);
mp_obj_t common_hal_bno080_BNO080_read(bno080_BNO080_obj_t *self, uint8_t report_id);
void common_hal_bno080_BNO080_deinit(bno080_BNO080_obj_t *self);

#endif // MICROPY_INCLUDED_SHARED_MODULE_BNO080_BNO080_H
