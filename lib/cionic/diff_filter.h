#ifndef __DIFF_FILTER_H__
#define __DIFF_FILTER_H__

// #include "cmsis-dsp/arm_math.h"
// #else
#include <math.h>
#include "utils.h"
// #endif

#define DIFF_FILTER_ORDER 2
#define DIFF_NUM_CHANNELS 8
#define DIFF_BLE_SAMPLES 200

// RMS window = RMS_NUM_SAMPLES * RMS_NUM_SQUARES
// this is in order to preserve resources
//
#define RMS_NUM_SAMPLES   50
#define RMS_NUM_SQUARES   4

typedef struct diff_filter_t {
    float input[DIFF_NUM_CHANNELS][DIFF_FILTER_ORDER+1];   // buffer for diff filter most recent sample at zero
    float rms_buffer[DIFF_NUM_CHANNELS][RMS_NUM_SAMPLES];  // buffer for sos most recent sample at rms_samples
    float rms_squares[DIFF_NUM_CHANNELS][RMS_NUM_SQUARES]; // buffer for rms calculation most recent sample last
    float rms[DIFF_NUM_CHANNELS];  // most recent rms calculation
    uint8_t rms_samples;           // tracks number of samples before calculating sos
} diff_filter_t;


// initialize once to reset the feature state
void diff_filter_init(diff_filter_t *filter);

// process a single sample of emg data and get results
int diff_filter_process(diff_filter_t *diff, float *norms, int numchans,
                        elapsed_t ts_in, const uint8_t *buffer,
                        elapsed_t *ts_out, float *uv_out);
#endif
