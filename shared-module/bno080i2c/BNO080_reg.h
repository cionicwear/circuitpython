/**
 * @file BNO080_reg.h
 *
 * defines register values for the bno080
 * https://www.hillcrestlabs.com/downloads/sh-2-reference-manual
 */



/**
 * REPORT ID LIST - Figure 34

SH-2 Control W 0xFE Get Feature Request
SH-2 Control W 0xFD Set Feature Command
SH-2 Control W 0xFC Get Feature Response
Wakeup/Normal R 0xFB Base Timestamp
Wakeup/Normal R 0xFA Timestamp Rebase
SH-2 Control W 0xF9 Product ID Request
SH-2 Control R 0xF8 Product ID Response
SH-2 Control W 0xF7 FRS Write Request
SH-2 Control W 0xF6 FRS Write Data
SH-2 Control W 0xF5 FRS Write Response
SH-2 Control W 0xF4 FRS Read Request
SH-2 Control R 0xF3 FRS Read Response
SH-2 Control W 0xF2 Command Request
SH-2 Control R 0xF1 Command Response
Wakeup/Normal R 0x01 Accelerometer
Wakeup/Normal R 0x02 Gyroscope
Wakeup/Normal R 0x03 Magnetic Field
Wakeup/Normal R 0x04 Linear Acceleration
Wakeup/Normal R 0x05 Rotation Vector
Wakeup/Normal R 0x06 Gravity
Wakeup/Normal R 0x07 Uncalibrated Gyroscope
Wakeup/Normal R 0x08 Game Rotation Vector
Wakeup/Normal R 0x09 Geomagnetic Rotation Vector
Wakeup/Normal R 0x0A Pressure
Wakeup/Normal R 0x0B Ambient Light
Wakeup/Normal R 0x0C Humidity
Wakeup/Normal R 0x0D Proximity
Wakeup/Normal R 0x0E Temperature
Wakeup/Normal R 0x0F Uncalibrated Magnetic Field
Wakeup/Normal R 0x10 Tap Detector
Wakeup/Normal R 0x11 Step Counter
Wakeup/Normal R 0x12 Significant Motion
Wakeup/Normal R 0x13 Stability Classifier
Wakeup/Normal R 0x14 Raw Accelerometer
Wakeup/Normal R 0x15 Raw Gyroscope
Wakeup/Normal R 0x16 Raw Magnetometer
Wakeup/Normal R 0x17 SAR
Wakeup/Normal R 0x18 Step Detector
Wakeup/Normal R 0x19 Shake Detector
Wakeup/Normal R 0x1A Flip Detector
Wakeup/Normal R 0x1B Pickup Detector
Wakeup/Normal R 0x1C Stability Detector
Wakeup/Normal R 0x1E Personal Activity Classifier
Wakeup/Normal R 0x1F Sleep Detector
Wakeup/Normal R 0x20 Tilt Detector
Wakeup/Normal R 0x21 Pocket Detector
Wakeup/Normal R 0x22 Circle Detector
Wakeup/Normal R 0x23 Heart Rate Monitor
Wakeup/Normal R 0x28 ARVR-Stabilized Rotation Vector
Wakeup/Normal R 0x29 ARVR-Stabilized Game Rotation Vector
*/

#define BNO080_GET_FEATURE_REQUEST                          0xFE
#define BNO080_SET_FEATURE_COMMAND                          0xFD
#define BNO080_GET_FEATURE_RESPONSE                         0xFC
#define BNO080_BASE_TIMESTAMP                               0xFB
#define BNO080_TIMESTAMP_REBASE                             0xFA
#define BNO080_PRODUCT_ID_REQUEST                           0xF9
#define BNO080_PRODUCT_ID_RESPONSE                          0xF8
#define BNO080_FRS_WRITE_REQ                                0xF7
#define BNO080_FRS_WRITE_DATA                               0xF6
#define BNO080_FRS_WRITE_RESP                               0xF5
#define BNO080_FRS_READ_REQ                                 0xF4
#define BNO080_FRS_READ_RESP                                0xF3
#define BNO080_COMMAND_REQ                                  0xF2
#define BNO080_COMMAND_RESP                                 0xF1

#define BNO080_SRID_ACCELEROMETER                0x01
#define BNO080_SRID_GYROSCOPE                    0x02
#define BNO080_SRID_MAGNETIC_FIELD               0x03
#define BNO080_SRID_LINEAR_ACCELERATION          0x04
#define BNO080_SRID_ROTATION_VECTOR              0x05
#define BNO080_SRID_GRAVITY                      0x06
#define BNO080_SRID_UNCAL_GYROSCOPE              0x07
#define BNO080_SRID_GAME_ROTATION_VECTOR         0x08
#define BNO080_SRID_GEOMAGNETIC_ROTATION_VECTOR  0x09
#define BNO080_SRID_TAP_DETECTOR                 0x10
#define BNO080_SRID_STEP_COUNTER                 0x11
#define BNO080_SRID_SIGNIFICANT_MOTION           0x12
#define BNO080_SRID_STABILITY_CLASSIFIER         0x13
#define BNO080_SRID_RAW_ACCELEROMETER            0x14
#define BNO080_SRID_RAW_GYROSCOPE                0x15
#define BNO080_SRID_RAW_MAGNETOMETER             0x16
#define BNO080_SRID_SAR                          0x17
#define BNO080_SRID_STEP_DETECTOR                0x18
#define BNO080_SRID_SHAKE_DETECTOR               0x19
#define BNO080_SRID_FLIP_DETECTOR                0x1A
#define BNO080_SRID_PICKUP_DETECTOR              0x1B
#define BNO080_SRID_STABILITY_DETECTOR           0x1C
#define BNO080_SRID_PERSONAL_ACTIVITY_CLASSIFIER 0x1E
#define BNO080_SRID_SLEEP_DETECTOR               0x1F
#define BNO080_SRID_TILT_DETECTOR                0x20
#define BNO080_SRID_POCKET_DETECTOR              0x21
#define BNO080_SRID_CIRCLE_DETECTOR              0x22
#define BNO080_SRID_HEART_RATE_MONITOR           0x23
#define BNO080_SRID_ARVR_ROTATION_VECTOR         0x28
#define BNO080_SRID_ARVR_GAME_ROTATION_VECTOR    0x29
#define BNO080_SRID_GYRO_INT_ROTATION_VECTOR     0x2A

/**
 * Sensor Meta Data - Figure 29
 *
0xE301 Raw accelerometer
0xE302 Accelerometer
0xE303 Linear acceleration
0xE304 Gravity
0xE305 Raw gyroscope
0xE306 Gyroscope calibrated
0xE307 Gyroscope uncalibrated
0xE308 Raw magnetometer
0xE309 Magnetic field calibrated
0xE30A Magnetic field uncalibrated
0xE30B Rotation vector
0xE30C Game rotation vector
0xE30D Geomagnetic rotation vector
0xE30E Pressure
0xE30F Ambient light
0xE310 Humidity
0xE311 Proximity
0xE312 Temperature
0xE313 Tap detector
0xE314 Step detector
0xE315 Step counter
0xE316 Significant motion
0xE317 Stability classifier
0xE318 Shake detector
0xE319 Flip detector
0xE31A Pickup detector
0xE31B Stability detector
0xE31C Personal Activity classifier
0xE31D Sleep detector
0xE31E Tilt detector
0xE31F Pocket detector
0xE320 Circle detector
0xE321 Heart Rate Monitor
0xE322 ARVR Stabilized Rotation Vector
0xE323 ARVR Stabilized Game Rotation Vector
0xE324 Gyro-integrated Rotation Vector
*/


/**
 * Counter Commands - Figure 44
1 Errors Command and Response to access error queue. See section 6.4.1
2 Counter Command and Response to access counters. See section 6.4.3
3 Tare Command and Response to operate on tare. See section 6.4.4
4 Initialize Reinitialize sensor hub components. See section 6.4.5
5 Reserved ID 5 is not currently in use. It is reserved for future use.
6 DCD Command to save DCD. See section 6.4.6
7 ME CAL Command and Response to configure ME Calibration. See section 6.4.7
8 Reserved Deprecated.
9 DCD Save Command to configure periodic saving of DCD. See section 6.4.6
10 Oscillator Command to retrieve the oscillator type used in the clock system.
11 Clear DCD and Reset Command to clear the in-memory DCD state and perform a chip reset.
 */

#define BNO080_COMMAND_ERRORS            0x01
#define BNO080_COMMAND_COUNTER           0x02
#define BNO080_COMMAND_TARE              0x03
#define BNO080_COMMAND_INITIALIZE        0x04
#define BNO080_COMMAND_INIT_STARTUP      0x84
#define BNO080_COMMAND_DCD_SAVE          0x06
#define BNO080_COMMAND_ME_CAL            0x07
#define BNO080_COMMAND_DCD_PERIODIC      0x09
#define BNO080_COMMAND_OSCILLATOR        0x0A
#define BNO080_COMMAND_DCD_CLEAR         0x0B

/*
#define BNO080_CALIBRATE_ACCEL 0
#define BNO080_CALIBRATE_GYRO 1
#define BNO080_CALIBRATE_MAG 2
#define BNO080_CALIBRATE_PLANAR_ACCEL 3
#define BNO080_CALIBRATE_ACCEL_GYRO_MAG 4
#define BNO080_CALIBRATE_STOP 5

#define BNO080_MAX_PACKET_SIZE 128 // Packets can be up to 32k but we don't have that much RAM.
#define BNO080_MAX_METADATA_SIZE 9 // This is in words. There can be many but we mostly only care about the first 9 (Qs, range, etc)
*/

