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
#include "shared-module/bno080/BNO080.h"
#include "lib/cionic/orientation.h"

#include "shared-bindings/microcontroller/__init__.h"
#include "shared-bindings/time/__init__.h"
#include "shared-bindings/util.h"
#include "shared-bindings/digitalio/DigitalInOut.h"

#include "py/mperrno.h"
#include <string.h>
#include <math.h>
#include "py/stream.h"

#define BNO_BAUDRATE    (1000000)


STATIC void lock_bus(bno080_BNO080_obj_t *self) {
    if (!common_hal_busio_spi_try_lock(self->bus)) {
        mp_raise_OSError(EAGAIN);
        return;
    }
}

STATIC void unlock_bus(bno080_BNO080_obj_t *self) {
    common_hal_busio_spi_unlock(self->bus);
}

STATIC void bno080_post_response(bno080_BNO080_obj_t *self, uint8_t response_id)
{
    self->resp = response_id;
}

STATIC void bno080_wait_for_response(bno080_BNO080_obj_t *self, uint8_t response_id)
{
    while(self->resp != response_id){
        mp_handle_pending(true);
        // Allow user to break out of a timeout with a KeyboardInterrupt.
        if (mp_hal_is_interrupted()) {
            return;
        }
    }

    self->resp = 0;
}

STATIC int bno080_txrx(bno080_BNO080_obj_t *self, uint8_t *txbuf, uint8_t *rxbuf, int txlen, int rxlen)
{
    if (txlen <= 0 && rxlen <= 0) {
        mp_printf(&mp_plat_print, "BNO called with nothing to do\n");
        return rxlen;
    }

    if (rxlen > BNO080_MAX_RX- 4) {
        mp_printf(&mp_plat_print, "BNO requested len %d max %d\n", rxlen, (BNO080_MAX_RX- 4));
        rxlen = BNO080_MAX_RX- 4;
    }

    int len = MAX(txlen, rxlen);

    // prepare the tx buffer
    // should be any outgoing transaction
    // plus zero padding to the size of rx
    //
    uint8_t tx[len];
    bzero(tx, sizeof(tx));
    if (txlen > 0) {
        memcpy(tx, txbuf, txlen);
    }

    common_hal_busio_spi_transfer(self->bus, tx, rxbuf, len);
    
    return rxlen;
}

/**
 * Send a data packet to the sensor
 *
 * @param dev      Sensor interface
 * @param channel  Sensor Channel to send to
 * @param buf      Byte array of data to send to the sensor
 * @param len      Length of data to send to the sensor
 *
 * @returns        0 on success else ERROR
 */
STATIC int bno080_spi_send(bno080_BNO080_obj_t *self, uint8_t channel, const uint8_t *buf, int len)
{
    lock_bus(self);
    if ((self->txlen + len + 4) > (int)sizeof(self->txbuf)) {
        return ENOMEM;
    }

    uint8_t *txbuf = &self->txbuf[self->txlen];

    txbuf[0] = len+4;
    txbuf[1] = 0;
    txbuf[2] = channel;
    txbuf[3] = self->write_seqnums[channel]++;

    memcpy(txbuf+4, buf, len);
    self->txlen += len+4;

    common_hal_digitalio_digitalinout_set_value(&self->ps0, false);
    unlock_bus(self);
    return 0;
}

STATIC void bno080_pid_response(bno080_BNO080_obj_t *self, elapsed_t timestamp, const uint8_t *buf, int len)
{
    self->pid.id = READ_LE(uint8_t, buf);
    self->pid.reset_cause = READ_LE(uint8_t, buf+1);
    self->pid.sw_ver_major = READ_LE(uint8_t, buf+2);
    self->pid.sw_ver_minor = READ_LE(uint8_t, buf+3);
    self->pid.sw_part_number = READ_LE(uint32_t, buf+4);
    self->pid.sw_build_number = READ_LE(uint32_t, buf+8);
    self->pid.sw_version_patch = READ_LE(uint16_t, buf+12);

    mp_printf(&mp_plat_print, "mpid.id %d\n", self->pid.id);
    mp_printf(&mp_plat_print, "mpid.reset_cause %d\n", self->pid.reset_cause);
    mp_printf(&mp_plat_print, "mpid.sw_ver_major.sw_ver_major %d\n", self->pid.sw_ver_major);
    mp_printf(&mp_plat_print, "mpid.sw_ver_minor.sw_ver_minor %d\n", self->pid.sw_ver_minor);
    mp_printf(&mp_plat_print, "mpid.sw_part_number %ld\n", self->pid.sw_part_number);
    mp_printf(&mp_plat_print, "mpid.sw_build_number %ld\n", self->pid.sw_build_number);
    mp_printf(&mp_plat_print, "mpid.sw_version_patch %d\n", self->pid.sw_version_patch);
}
// From 1000-3927 BNO080 Datasheet Figure 1-31: FRS records
const uint16_t bno080_frs_ids[] = {
    0x7979, // Static calibration – AGM
    0x4D4D, // Nominal calibration – AGM
    0x8A8A, // Static calibration – SRA
    0x4E4E, // Nominal calibration - SRA
    0x1F1F, // Dynamic calibration
    0xD3E2, // MotionEngine power management
    0x2D3E, // System orientation
    0x2D41, // Primary accelerometer orientation
    0x2D46, // Gyroscope orientation
    0x2D4C, // Magnetometer orientation
    0x3E2D, // AR/VR stabilization – rotation vector
    0x3E2E, // AR/VR stabilization – game rotation vector
    0xC274, // Significant Motion detector configuration
    0x7D7D, // Shake detector configuration
    0xD7D7, // Maximum fusion period
    0x4B4B, // Serial number
    0x39AF, // Environmental sensor - Pressure calibration
    0x4D20, // Environmental sensor - Temperature calibration
    0x1AC9, // Environmental sensor - Humidity calibration
    0x39B1, // Environmental sensor - Ambient light calibration
    0x4DA2, // Environmental sensor - Proximity calibration
    0xD401, // ALS Calibration
    0xD402, // Proximity Sensor Calibration
    0xED85, // Stability detector configuration
    0x74B4, // User record
    0xD403, // MotionEngine Time Source Selection
    0xA1A2, // Gyro-Integrated Rotation Vector configuration
};

typedef struct frs_write_t {
    uint16_t frs_id;
    uint8_t length;
    const uint32_t *data;
} frs_write_t;

uint32_t bno080_rotation_vector_config[] = { 0xccccccd, 0x410624e, 0x191eb852, 0x0 };

const frs_write_t bno080_frs_writes[] = {
     // { frs_id, offset, data0, data1 }
     { 0x3E2D, ARRAY_SIZE(bno080_rotation_vector_config), bno080_rotation_vector_config },
     { 0x3E2E, ARRAY_SIZE(bno080_rotation_vector_config), bno080_rotation_vector_config }
};

STATIC int bno080_spi_frs(bno080_BNO080_obj_t *self)
{
    // 1. write configurations
    //
    if (self->frs_write < (int)ARRAY_SIZE(bno080_frs_writes)) {
        frs_write_t write = bno080_frs_writes[self->frs_write];
        int offset = self->frs_write_offset;
        if (offset == BNO080_FRS_WRITE_START) {
            // 1a. tell bno we want to write
            //
            const uint8_t command[] = {
                BNO080_FRS_WRITE_REQ,  // Report ID
                0,                     // Reserved
                LE_U16(write.length),  // Length to written
                LE_U16(write.frs_id)   // FRS Type
            };
            self->frs_write_offset = 0;
            return bno080_spi_send(self, BNO080_CHANNEL_CONTROL, command, sizeof(command));
        }
        else if (offset < write.length) {
            // 1b. write data
            //
            const uint8_t command[] = {
                BNO080_FRS_WRITE_DATA,        // Report ID
                0,                            // Reserved
                LE_U16(offset),               // Offset to write to
                LE_U32(write.data[offset]),   // Data 0
                LE_U32(write.data[offset+1])  // Data 1
            };
            self->frs_write_offset += 2;
            return bno080_spi_send(self, BNO080_CHANNEL_CONTROL, command, sizeof(command));
        }
        else {
            // 1c. wait for write to complete
            //
            return 0;
        }
    }

    // 2. read configurations
    //
    if (self->frs_read < (int)ARRAY_SIZE(bno080_frs_ids)) {
        uint16_t frstype = bno080_frs_ids[self->frs_read];
        const uint8_t command[] = {
                BNO080_FRS_READ_REQ,  // Report ID
                0,                    // Reserved
                0, 0,                 // Read Offset
                LE_U16(frstype),      // FRS Type
                0, 0                  // Block Size (0 == entire record)
        };
        return bno080_spi_send(self, BNO080_CHANNEL_CONTROL, command, sizeof(command));
    }

    return 0;
}

STATIC int bno080_frs_save_index(bno080_frs_t *frs)
{
    if (frs->id == 0x3E2D && frs->offset == 0) return 0;
    if (frs->id == 0x3E2D && frs->offset == 2) return 1;
    if (frs->id == 0x3E2E && frs->offset == 0) return 2;
    if (frs->id == 0x3E2E && frs->offset == 2) return 3;

    return ENOMEM;
}

STATIC void bno080_read_frs(bno080_BNO080_obj_t *self, const uint8_t *buf, int len)
{
    uint8_t length_status = READ_LE(uint8_t, buf+1);
    uint8_t status = length_status & 0x0F;

    bno080_frs_t frs;
    frs.id = READ_LE(uint16_t, buf+12);     // frs type
    frs.offset = READ_LE(uint16_t, buf+2);  // offset
    frs.data0 = READ_LE(uint32_t, buf+4);   // data0
    frs.data1 = READ_LE(uint32_t, buf+8);   // data1
    
    int save_idx = bno080_frs_save_index(&frs);
    if (save_idx >= 0) {
        self->frs_saved[save_idx] = frs;
    }

    if (status == BNO080_FRS_READ_COMPLETE ||
        status == BNO080_FRS_READ_EMPTY) {
        // if complete advance to the next FRS operation
        self->frs_read++;
        bno080_spi_frs(self);
    }
}

STATIC void bno080_write_frs(bno080_BNO080_obj_t *self, const uint8_t *buf, int len)
{
    uint8_t status = READ_LE(uint8_t, buf+1);

    if (status == BNO080_FRS_WRITE_COMPLETE) {
        // if complete advance the pointer
        self->frs_write++;
        self->frs_write_offset = BNO080_FRS_WRITE_START;
    }
    bno080_spi_frs(self);
}

STATIC void bno080_feature_response(bno080_BNO080_obj_t *self, elapsed_t timestamp, const uint8_t *buf, int len)
{
    uint8_t feature_id = buf[1];
    uint32_t rate = READ_LE(uint32_t, buf+5);
    char *feature = (char *)"";
    switch(feature_id) {
    case BNO080_SRID_ACCELEROMETER:
        feature = (char *)"ACCELEROMETER";
        break;
    case BNO080_SRID_GYROSCOPE:
        feature = (char *)"GYROSCOPE";
        break;
    case BNO080_SRID_MAGNETIC_FIELD:
        feature = (char *)"MAGNETIC_FIELD";
        break;
    case BNO080_SRID_LINEAR_ACCELERATION:
        feature = (char *)"LINEAR_ACCELERATION";
        break;
    case BNO080_SRID_ROTATION_VECTOR:
        feature = (char *)"ROTATION_VECTOR";
        break;
    case BNO080_SRID_GRAVITY:
        feature = (char *)"GRAVITY";
        break;
    case BNO080_SRID_ARVR_ROTATION_VECTOR:
        feature = (char *)"ARVR_ROTATION_VECTOR";
        break;
    case BNO080_SRID_ARVR_GAME_ROTATION_VECTOR:
        feature = (char *)"ARVR_GAME_ROTATION_VECTOR";
        break;
    case BNO080_SRID_GYRO_INT_ROTATION_VECTOR:
        feature = (char *)"GYRO_INT_ROTATION_VECTOR";
        break;
    case BNO080_SRID_GAME_ROTATION_VECTOR:
        feature = (char *)"GAME_ROTATION VECTOR";
        break;
    case BNO080_SRID_UNCAL_GYROSCOPE:
    case BNO080_SRID_GEOMAGNETIC_ROTATION_VECTOR:
    case BNO080_SRID_TAP_DETECTOR:
    case BNO080_SRID_STEP_COUNTER:
    case BNO080_SRID_SIGNIFICANT_MOTION:
    case BNO080_SRID_STABILITY_CLASSIFIER:
    case BNO080_SRID_RAW_ACCELEROMETER:
    case BNO080_SRID_RAW_GYROSCOPE:
    case BNO080_SRID_RAW_MAGNETOMETER:
    case BNO080_SRID_SAR:
    case BNO080_SRID_STEP_DETECTOR:
    case BNO080_SRID_SHAKE_DETECTOR:
    case BNO080_SRID_FLIP_DETECTOR:
    case BNO080_SRID_PICKUP_DETECTOR:
    case BNO080_SRID_STABILITY_DETECTOR:
    case BNO080_SRID_PERSONAL_ACTIVITY_CLASSIFIER:
    case BNO080_SRID_SLEEP_DETECTOR:
    case BNO080_SRID_TILT_DETECTOR:
    case BNO080_SRID_POCKET_DETECTOR:
    case BNO080_SRID_CIRCLE_DETECTOR:
    case BNO080_SRID_HEART_RATE_MONITOR:
    default:
        feature = (char *)"UNKNOWN";
        break;
    }

    mp_printf(&mp_plat_print, "Feature %s : %ld\n", feature, rate);
}

STATIC void bno080_command_response(bno080_BNO080_obj_t *self, elapsed_t timestamp, const uint8_t *buf, int len)
{
    uint8_t command_id = buf[2];
    char *command = (char *)"";
    uint8_t status = buf[5];
    
    switch(command_id) {
    case BNO080_COMMAND_ERRORS:
        command = (char *)"ERRORS";
        break;
    case BNO080_COMMAND_COUNTER:
        command = (char *)"COUNTER";
        break;
    case BNO080_COMMAND_TARE:
        command = (char *)"TARE";
        break;
    case BNO080_COMMAND_INITIALIZE:
        command = (char *)"INITIALIZE";
        break;
    case BNO080_COMMAND_INIT_STARTUP:
        command = (char *)"INIT_STARTUP";
        self->init_done = true;
        break;
    case BNO080_COMMAND_DCD_SAVE:
        command = (char *)"DCD_SAVE";
        break;
    case BNO080_COMMAND_ME_CAL:
        command = (char *)"ME_CAL";
        break;
    case BNO080_COMMAND_DCD_PERIODIC:
        command = (char *)"DCD_PERIODIC";
        break;
    case BNO080_COMMAND_OSCILLATOR:
        command = (char *)"OSCILLATOR";
        break;
    case BNO080_COMMAND_DCD_CLEAR:
        command = (char *)"DCD_CLEAR";
        break;
    default:
        command = (char *)"UNKNOWN";
        break;
    }
    
    mp_printf(&mp_plat_print, "command response %s = %d\n", command, status);   
}

STATIC void bno080_control(bno080_BNO080_obj_t *self, elapsed_t timestamp, const uint8_t *buf, int len)
{
    uint8_t control_id = buf[0];
    switch (control_id) {
    case BNO080_FRS_READ_RESP:
        bno080_read_frs(self, buf, len);
        break;
    case BNO080_FRS_WRITE_RESP:
        bno080_write_frs(self, buf, len);
        break;
    case BNO080_GET_FEATURE_RESPONSE:
        bno080_feature_response(self, timestamp, buf, len);
        break;
    case BNO080_COMMAND_RESP:
        bno080_command_response(self, timestamp, buf, len);
        break;
    case BNO080_PRODUCT_ID_RESPONSE:
        bno080_pid_response(self, timestamp, buf, len);
        break;
    default:
        mp_printf(&mp_plat_print, "unknown control %d\n", control_id);
        break;
    }

    bno080_post_response(self, control_id);
}

STATIC void bno080_report_rotation(bno080_BNO080_obj_t *self, elapsed_t timestamp, const uint8_t *pkt, int len)
{
    /**
     * 6.5.42.2 Input Report
     *
     * 0 Report ID = 0x23
     * 1 Sequence number
     * 2 Status
     * 3 Delay
     * 4 Unit quaternion i component LSB
     * 5 Unit quaternion i component MSB
     * 6 Unit quaternion j component LSB
     * 7 Unit quaternion j component MSB
     * 8 Unit quaternion k component LSB
     * 9 Unit quaternion k component MSB
     * 10 Unit quaternion real component LSB
     * 11 Unit quaternion real component MSB
     * 12 Accuracy estimate LSB
     * 13 Accuracy estimate MSB
     */
    uint8_t qp = 14;  /// per section 6.5.19 Q Point = 14
    // https://en.wikipedia.org/wiki/Q_(number_format)
    float scale = pow(2.0, -qp);

    self->fquat[0] = mp_obj_new_float(READ_LE(int16_t, &pkt[4])*scale);  // i
    self->fquat[1] = mp_obj_new_float(READ_LE(int16_t, &pkt[6])*scale);  // j
    self->fquat[2] = mp_obj_new_float(READ_LE(int16_t, &pkt[8])*scale);  // k
    self->fquat[3] = mp_obj_new_float(READ_LE(int16_t, &pkt[10])*scale);  // real
}

STATIC void bno080_report(bno080_BNO080_obj_t *self, elapsed_t timestamp, uint8_t accuracy, const uint8_t *buf, int len)
{
    // currently all reports must start with base timestamp reference
    // ASSERT(buf[0] == BNO080_BASE_TIMESTAMP);
    if(buf[0] != BNO080_BASE_TIMESTAMP){
        mp_printf(&mp_plat_print, "no timestamp found\n");
        return;
    }

    uint8_t report_id = buf[BNO080_SRID_OFFSET];
    switch (report_id) {
    // rotation vectors all with Q point 14
    case BNO080_SRID_ARVR_GAME_ROTATION_VECTOR:
    case BNO080_SRID_ARVR_ROTATION_VECTOR:
    case BNO080_SRID_GEOMAGNETIC_ROTATION_VECTOR:
    case BNO080_SRID_GAME_ROTATION_VECTOR:
    case BNO080_SRID_ROTATION_VECTOR:
        bno080_report_rotation(self, timestamp, buf+BNO080_SRID_OFFSET, len-BNO080_SRID_OFFSET);
        break;
    case BNO080_SRID_ACCELEROMETER:
        // bno080_check_accuracy(bno, BNO080_ACCURACY_ACCEL, accuracy);
        // bno080_report_accel(bno, timestamp, buf+BNO080_SRID_OFFSET, len-BNO080_SRID_OFFSET, accuracy);
        break;
    case BNO080_SRID_GYROSCOPE:
        // bno080_check_accuracy(bno, BNO080_ACCURACY_GYRO, accuracy);
        // bno080_report_gyroscope(bno, timestamp, buf+BNO080_SRID_OFFSET, len-BNO080_SRID_OFFSET, accuracy);
        break;
    case BNO080_SRID_MAGNETIC_FIELD:
        // bno080_check_accuracy(bno, BNO080_ACCURACY_MAG, accuracy);
        // bno080_report_magnetic_field(bno, timestamp, buf+BNO080_SRID_OFFSET, len-BNO080_SRID_OFFSET, accuracy);
        break;
    case BNO080_SRID_GRAVITY:
        // bno080_report_grav(bno, timestamp, buf+BNO080_SRID_OFFSET, len-BNO080_SRID_OFFSET, accuracy);
        break;
    // IMU sensor values currently recorded raw
    case BNO080_SRID_LINEAR_ACCELERATION:
    case BNO080_SRID_UNCAL_GYROSCOPE:
        break;
    default:
        // TRACE_BUF(buf+BNO080_SRID_OFFSET, len-BNO080_SRID_OFFSET, "unknown report");
        break;
    };

}

STATIC int bno080_on_read(bno080_BNO080_obj_t *self, elapsed_t timestamp, uint8_t *buf, int len)
{
    uint8_t channel = buf[2];

    switch (channel) {
        case BNO080_CHANNEL_REPORT:

            // currently all sensor reports must start with base timestamp reference
            // if we implement batching this will no longer be true
            // and we will need to handle the channel report seperate from sensor report
            if(buf[BNO080_HEADER_SIZE] != BNO080_BASE_TIMESTAMP){
                mp_printf(&mp_plat_print, "BNO080_HEADER_SIZE != BNO080_BASE_TIMESTAMP\n");
                return EINVAL;
            }

            // 7.2.1 Base Timestamp Reference (0xFB)
            // relative to transport-defined reference point. Signed. Units are 100 microsecond ticks.
            // For example, if HINT occurs at some time t and the Base Timestamp Reference record has
            // a value for delta of 10, the timestamps in a given batch will be relative to t – 1 ms.
            int32_t base_delta = READ_LE(int32_t, &buf[BNO080_HEADER_SIZE+BNO080_BASE_DELTA_OFFSET]);
            timestamp -= base_delta;

            // 6.5.1 Common Fields
            // read 16 bits of status and delay
            // Status
            // Bits 1:0 – indicate the status of a sensor. 0 – Unreliable 1 – Accuracy low 2 – Accuracy medium 3 – Accuracy high
            // Bits 7:2 – Delay upper bits: 6 most-significant bits of report delay. See below.
            // Delay LSB
            // 8 least-significant bits of report delay. Units are 100 us.
            uint16_t status_delay = READ_LE(uint16_t, &buf[BNO080_HEADER_SIZE+BNO080_SRID_OFFSET+BNO080_STATUS_DELAY_OFFSET]);
            //    uint8_t status = status_delay >> 14;
            uint16_t report_delay = status_delay & 0x3fff;
            
            uint8_t accuracy = READ_LE(uint8_t, &buf[BNO080_HEADER_SIZE+BNO080_SRID_OFFSET+BNO080_STATUS_DELAY_OFFSET]);
            
            timestamp += report_delay;

            bno080_report(self, timestamp, accuracy, buf+BNO080_HEADER_SIZE, len-BNO080_HEADER_SIZE);
            break;

        case BNO080_CHANNEL_EXECUTE:
            // ASSERT(buf[4] == 0x1); // reset complete
            if (buf[4] != 1) {
                mp_printf(&mp_plat_print, "error buf[4] = 0x%x\n", buf[4]);
            }
            break;

        case BNO080_CHANNEL_COMMAND:
        case BNO080_CHANNEL_CONTROL:
            bno080_control(self, timestamp, buf+BNO080_HEADER_SIZE, len-BNO080_HEADER_SIZE);
            break;
        case BNO080_CHANNEL_WAKE:
        case BNO080_CHANNEL_GYRO:
        default:
            break;
    };

    return 0;
}
STATIC int bno080_txrx_spi(bno080_BNO080_obj_t *self, uint8_t **outbuf)
{
    lock_bus(self);                           // select
    common_hal_digitalio_digitalinout_set_value(&self->cs, false);
    common_hal_digitalio_digitalinout_set_value(&self->ps0, true);

    int rxlen = 0;              // read from incoming header
    int txlen = self->txbuf[0];  // size of outgoing transaction

    // transact headers - 4 bytes each
    uint8_t *hobuf = self->txbuf;
    uint8_t *hibuf = self->rxbuf;
    int holen = (txlen>=4) ? 4 : 0;
    int hilen = 4;
    hilen = bno080_txrx(self, hobuf, hibuf, holen, hilen);

    // figure out the size of the receive
    rxlen = READ_LE(uint16_t, hibuf);
    if (rxlen == 0xffff) {  // nothing to receive
        rxlen = 0;
    } else if (rxlen & 0x8000) { // msb == continuation
        rxlen &= 0x7fff;
    }

    // transact payloads
    uint8_t *pobuf = self->txbuf + 4;
    uint8_t *pibuf = self->rxbuf + 4;

    // let these possibly be negative, for correct return value
    int polen = txlen - 4;
    int pilen = rxlen - 4;

    pilen = bno080_txrx(self, pobuf, pibuf, polen, pilen);

    *outbuf = self->rxbuf;

    // shift the transaction queue
    if (txlen > 0) {
        self->txlen -= txlen;
        memmove(self->txbuf, &self->txbuf[txlen], self->txlen);
        memset(&self->txbuf[self->txlen], 0, txlen);
    }

    // deselect
    common_hal_digitalio_digitalinout_set_value(&self->cs, true);
    unlock_bus(self);

    return hilen+pilen;
}

STATIC int bno080_spi_sample(bno080_BNO080_obj_t *self)
{
    // save timestamp before transfer
    elapsed_t timestamp = self->last_timestamp;

    // the resultant buffer points to bno->rxbuf and is not locked after this call
    uint8_t *buf = NULL;
    int len = bno080_txrx_spi(self, &buf);

    if (len == 0) { // SHTP 2.3.1  no cargo
        return 0;
    } else if (len < 0) {
        mp_printf(&mp_plat_print, "Wrong length rx: %d\n", len);
        return len;
    } else if (len < BNO080_HEADER_SIZE) {
        mp_printf(&mp_plat_print, "Wrong length rx: %d\n", len);
        return -1;
    }

    uint8_t channel = buf[2];
    uint8_t seqnum = buf[3];
    uint8_t expectedseq = self->read_seqnums[channel]+1;
    if (seqnum != expectedseq) {
        // DISABLED ONLY FOR FES BUILD - PLEASE REENABLE
        // LOG(ERROR, "[channel %d] expected seq %d, got %d", channel, expectedseq, seqnum);
    }
    self->read_seqnums[channel] = seqnum;

    return bno080_on_read(self, timestamp, buf, len);
}

STATIC int bno080_read_pid(bno080_BNO080_obj_t *self)
{
    const uint8_t command[] = {
        BNO080_PRODUCT_ID_REQUEST,  
        0,                          // Reserved
    };
    
    bno080_spi_send(self, BNO080_CHANNEL_CONTROL, command, sizeof(command));

    bno080_wait_for_response(self, BNO080_PRODUCT_ID_RESPONSE);
    return 0;
}

STATIC void bno080_isr_recv(void *arg) {
    bno080_BNO080_obj_t *self = (bno080_BNO080_obj_t *)arg;

    bno080_spi_sample(self);
}

void common_hal_bno080_BNO080_construct(bno080_BNO080_obj_t *self, busio_spi_obj_t *bus, const mcu_pin_obj_t *cs, const mcu_pin_obj_t *rst, const mcu_pin_obj_t *ps0, const mcu_pin_obj_t *bootn, const mcu_pin_obj_t *irq) {    
    self->bus = bus;
    self->resp = 0;
    self->init_done = false;
    common_hal_digitalio_digitalinout_construct(&self->cs, cs);
    common_hal_digitalio_digitalinout_switch_to_output(&self->cs, true, DRIVE_MODE_PUSH_PULL);
    common_hal_digitalio_digitalinout_construct(&self->rst, rst);
    common_hal_digitalio_digitalinout_switch_to_output(&self->rst, true, DRIVE_MODE_PUSH_PULL);
    common_hal_digitalio_digitalinout_construct(&self->ps0, ps0);
    common_hal_digitalio_digitalinout_switch_to_output(&self->ps0, true, DRIVE_MODE_PUSH_PULL);
    common_hal_digitalio_digitalinout_construct(&self->bootn, bootn);
    common_hal_digitalio_digitalinout_switch_to_output(&self->bootn, true, DRIVE_MODE_PUSH_PULL);
    common_hal_digitalio_digitalinout_construct(&self->irq, irq);
    common_hal_digitalio_digitalinout_set_irq(&self->irq, EDGE_FALL, PULL_UP, bno080_isr_recv, self);

    lock_bus(self);
    common_hal_busio_spi_configure(self->bus, BNO_BAUDRATE, 1, 1, 8);
    unlock_bus(self);

    common_hal_bno080_BNO080_reset(self);

    while(!self->init_done){
        mp_handle_pending(true);
        // Allow user to break out of a timeout with a KeyboardInterrupt.
        if (mp_hal_is_interrupted()) {
            return;
        }
    }

    bno080_read_pid(self);

    if(self->pid.id != BNO080_PRODUCT_ID_RESPONSE){
        mp_raise_OSError(ENODEV);
        return;
    }
    
    mp_printf(&mp_plat_print, "BNO id=%x found\n", self->pid.id);
    return;
}

void common_hal_bno080_BNO080_reset(bno080_BNO080_obj_t *self) {
    common_hal_digitalio_digitalinout_set_value(&self->ps0, true);
    common_hal_digitalio_digitalinout_set_value(&self->rst, true);

    common_hal_digitalio_digitalinout_set_value(&self->rst, false);
    common_hal_digitalio_digitalinout_set_value(&self->rst, true);

    // clear seqnums
    memset(self->read_seqnums, 0xff, sizeof(self->read_seqnums));
    memset(self->write_seqnums, 0x00, sizeof(self->write_seqnums));
}

STATIC void bno080_unary_rotation(bno080_BNO080_obj_t *self, uint8_t feature)
{
    switch(feature) {
        case BNO080_SRID_ARVR_GAME_ROTATION_VECTOR:
        case BNO080_SRID_ARVR_ROTATION_VECTOR:
        case BNO080_SRID_GEOMAGNETIC_ROTATION_VECTOR:
        case BNO080_SRID_GAME_ROTATION_VECTOR:
        case BNO080_SRID_ROTATION_VECTOR:
            if (self->selected_rotation != 0 && self->selected_rotation != feature) {
                uint8_t disable = self->selected_rotation;
                common_hal_bno080_BNO080_set_feature(self, disable, 0, 0, 0, 0, 0);
            }
            self->selected_rotation = feature;
            break;
        default:
            break;
    }
}

int common_hal_bno080_BNO080_set_feature(bno080_BNO080_obj_t *self, uint8_t feature, uint32_t refresh_us, uint32_t batch_us, uint8_t flags, uint16_t sns, uint32_t cfg) {
    int rc = 0;
    bno080_unary_rotation(self, feature);


    const uint8_t command[17] = {
        BNO080_SET_FEATURE_COMMAND,
        feature,
        flags,                 // flags
        (sns >> 0)  & 0xFF,    // sensitivity LSB
        (sns >> 8)  & 0xFF,    // sensitivity MSB
        (refresh_us >> 0)   & 0xFF,    // us LSB
        (refresh_us >> 8)   & 0xFF,    // us
        (refresh_us >> 16)  & 0xFF,    // us
        (refresh_us >> 24)  & 0xFF,    // us MSB
        (batch_us >> 0)     & 0xFF,    // batch interval LSB
        (batch_us >> 8)     & 0xFF,    // batch interval
        (batch_us >> 16)    & 0xFF,    // batch interval
        (batch_us >> 24)    & 0xFF,    // batch interval MSB
        (cfg >> 0)  & 0xFF,    // config LSB
        (cfg >> 8)  & 0xFF,    // config
        (cfg >> 16) & 0xFF,    // config
        (cfg >> 24) & 0xFF     // config MSB
    };

    mp_printf(&mp_plat_print, "setting feature [%d] rate [%d]\n", feature, refresh_us);
    rc = bno080_spi_send(self, BNO080_CHANNEL_CONTROL, command, sizeof(command));
    
    if(rc){
        mp_raise_OSError(rc);
    }

    return rc;
}

mp_obj_t common_hal_bno080_BNO080_read(bno080_BNO080_obj_t *self, uint8_t report_id) {
    // mp_obj_t fquat[QUAT_DIMENSION];
    // int rc = 0;

    switch(report_id){
        case BNO080_SRID_ARVR_GAME_ROTATION_VECTOR:
        case BNO080_SRID_ARVR_ROTATION_VECTOR:
        case BNO080_SRID_GEOMAGNETIC_ROTATION_VECTOR:
        case BNO080_SRID_GAME_ROTATION_VECTOR:
        case BNO080_SRID_ROTATION_VECTOR:
            return mp_obj_new_list(QUAT_DIMENSION, self->fquat);
        case BNO080_SRID_ACCELEROMETER:
            // bno080_check_accuracy(bno, BNO080_ACCURACY_ACCEL, accuracy);
            // bno080_report_accel(bno, timestamp, buf+BNO080_SRID_OFFSET, len-BNO080_SRID_OFFSET, accuracy);
            break;
        case BNO080_SRID_GYROSCOPE:
            // bno080_check_accuracy(bno, BNO080_ACCURACY_GYRO, accuracy);
            // bno080_report_gyroscope(bno, timestamp, buf+BNO080_SRID_OFFSET, len-BNO080_SRID_OFFSET, accuracy);
            break;
        case BNO080_SRID_MAGNETIC_FIELD:
            // bno080_check_accuracy(bno, BNO080_ACCURACY_MAG, accuracy);
            // bno080_report_magnetic_field(bno, timestamp, buf+BNO080_SRID_OFFSET, len-BNO080_SRID_OFFSET, accuracy);
            break;
        case BNO080_SRID_GRAVITY:
            break;
    }

    return NULL;
}

void common_hal_bno080_BNO080_deinit(bno080_BNO080_obj_t *self) {
    if (!self->bus) {
        return;
    }
    
    self->bus = 0;

    common_hal_digitalio_digitalinout_deinit(&self->cs);
    common_hal_digitalio_digitalinout_deinit(&self->rst);
    common_hal_digitalio_digitalinout_deinit(&self->ps0);
    common_hal_digitalio_digitalinout_deinit(&self->bootn);
    common_hal_digitalio_digitalinout_deinit(&self->irq);
    return;
}
