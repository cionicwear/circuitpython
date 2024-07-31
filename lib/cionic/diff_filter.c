#include "shared-bindings/util.h"
#include "diff_filter.h"
#include <math.h>

// diff_filter_run
// --
// given an array of floats <input>, return the <order>-order diff of position zero
//
// samples ordered most recent at [0]
//
// first order diff
// out[i] = a[i] - a[i+1]
//
// second order diff
// inter[i] = a[i] - a[i+1]
// out[i] = inter[i] - iter[i+1]

static float
diff_filter_run(float *input, int order) {
    if (order == 0) {
        return input[0];
    }
    float interim[order];
    for (int i = 0; i < order; i++) {
        interim[i] = input[i] - input[i+1];
    }
    return diff_filter_run(interim, order-1);
}

// diff_filter_process
// --
// called on every EMG sample
//
// for each channel
//   shift input buffer and append sample
//   calculate N-order diff on that sample
//   append to rms_buffer
//
// once rms_buffer is full for each channel
//   calculate sum_of_squares for the rms_buffer
//   shift rms_squares buffer and append sum_of_squares
//   calculate RMS value from rm_squares buffer
//   and send out over BLE
//
// process a single sample of emg data and get results
int
diff_filter_process(diff_filter_t *diff, float *norms, int numchans,
                    elapsed_t ts_in, const uint8_t *buffer,
                    elapsed_t *ts_out, float *uv_out)
{
    // 1. calculate N order diff and prepend to rms_bufffer
    for (int i = 0; i < numchans; i++) {
        // @todo replace with memmove
        for (int j = DIFF_FILTER_ORDER; j > 0; j--) {
            diff->input[i][j] = diff->input[i][j-1];
        }
        int16_t tf = READ_BE(int16_t, buffer);
        float uv = (float)tf * norms[i];
        diff->input[i][0] = uv;
        buffer += 2;
        // calculate N-order diff
        float output = diff_filter_run(diff->input[i], DIFF_FILTER_ORDER);
        // append to rms_buffer
        diff->rms_buffer[i][diff->rms_samples] = output;
        // output channel rms
        uv_out[i] = diff->rms[i];
    }    

    // 2. if rms_buffer filled compute sos for window and update rms
    if (++diff->rms_samples >= RMS_NUM_SAMPLES) {
        for (int i = 0; i < numchans; i++) {
            float square = 0;
            for (int j = 0; j < RMS_NUM_SAMPLES; j++) {
                float val = diff->rms_buffer[i][j];
                float sq = val*val;
                square += sq;
            }
            // compute rms
            // start with newly compute square
            // move each rms window down
            // adding to the sum of squares
            float sos = square;
            for (int k = 1; k < RMS_NUM_SQUARES; k++) {
                sos += diff->rms_squares[i][k];
                diff->rms_squares[i][k-1] = diff->rms_squares[i][k];
            }
            diff->rms_squares[i][RMS_NUM_SQUARES-1] = square;
            diff->rms[i] = sqrt(sos/(RMS_NUM_SQUARES*RMS_NUM_SAMPLES));
            // update output rms
            uv_out[i] = diff->rms[i];
        }
        diff->rms_samples = 0;
    }

    // output at same timestamp as input
    *ts_out = ts_in;
    return 0;
}

void
diff_filter_init(diff_filter_t *filter)
{
    bzero(filter, sizeof(diff_filter_t));
}

