#include "shared-bindings/util.h"
#include "emg_iir.h"
#include <math.h>

// generated coefficients
float emg_highpass_filter_sos[NO_OF_BQS][NO_OF_COEFFS_PER_BQ] = {
    {0.20822075418596409,-0.41644150837192817,0.20822075418596409,1.0,-0.8402869216513267,0.18834516088404457},
    {1.0,-2.0,1.0,1.0,-0.9428090415820631,0.33333333333333326},
    {1.0,-2.0,1.0,1.0,-1.1954339628907378,0.6905989232414969},
};

float emg_lowpass_filter_sos[NO_OF_BQS][NO_OF_COEFFS_PER_BQ] = {
    {0.0003319067151156544,0.0006638134302313088,0.0003319067151156544,1.0,-1.0360442299054848,0.2777110296235904},
    {1.0,2.0,1.0,1.0,-1.1470415772827258,0.4145995242545582},
    {1.0,2.0,1.0,1.0,-1.4083885078365557,0.7369080185138834},
};

// takes as input a bank of biquad coefficients ( NO_OF_BQS of them ) with the
// order of the coefficients as shown below. conveniently, this is the same
// order of the SOS ( second order section ) coefficients as produced by the
// scipy package:
// b0, b1, b2, a0, a1, a2
// therefore
// a0*y[n] = b0*x[0] + b1*x[1] + b2*x[2] -a1*y[1] -a2*y[2]
// or 
// y[n] = b0*x[0] + b1*x[1] + b2*x[2] -a1*y[1] -a2*y[2]
// since a0 is normalized to 1.0.
float 
emg_iir(emg_filter_state_t *state, float val)
{
    float y;
    for(int i=0; i<NO_OF_BQS; i++)
    {
        // update the input delay line
        state->x[i][2] = state->x[i][1];
        state->x[i][1] = state->x[i][0];
        state->x[i][0] = val; 
       
        y = 0;
        y += state->x[i][0]*state->coeffs[i][0];
        y += state->x[i][1]*state->coeffs[i][1];
        y += state->x[i][2]*state->coeffs[i][2];
    
        // skip one coeff here
        y -= state->y[i][0]*state->coeffs[i][4];
        y -= state->y[i][1]*state->coeffs[i][5];

        // update the output delay line
        state->y[i][1] = state->y[i][0];
        state->y[i][0] = y; 
        val = y;
    }
    return y;
}

void
emg_iir_init(emg_filter_state_t *state)
{
    for(int i=0; i<NO_OF_BQS; i++)
    {
        // update the input delay line
        state->x[i][2] = 0;
        state->x[i][1] = 0;
        state->x[i][0] = 0; 
    
        // update the output delay line
        state->y[i][1] = 0;
        state->y[i][0] = 0; 
    }
}

// the DC blocking is showing promise
#define NO_EMG_RMS_DC_BLOCKING
// the diff is not working that great
#define NO_EMG_RMS_DIFF
double 
emg_mwa_rms(emg_mwa_state_t *state, float val)
{
    // subtract the oldest value
    state->sum -= state->mw[state->write_ptr];    
    // update the delay line
    state->mw[state->write_ptr] = val*val;    
    state->write_ptr = (state->write_ptr+1) % EMG_RMS_MA_SIZE;

    // the latest sum
    state->sum += val*val;
    double result = state->sum;
    result = sqrt(result/EMG_RMS_MA_SIZE);
#ifdef EMG_RMS_DC_BLOCKING
    result = result + state->dc; 
    state->dc = state->dc - result*state->alpha;
#endif
#ifdef EMG_RMS_DIFF
    double temp = result;
    result = result - state->past_sum;
    state->past_sum = temp;
#endif
    return result;
}

void
iir_filter_init(iir_filter_t *filter)
{
    filter->emg_rms_sub_sample_counter = EMG_RMS_SUBSAMPLING_FACTOR;
    for( int i=0; i<IIR_NUM_CHANNELS; i++) {
        emg_iir_init(&filter->emg_lowpass_iir_state[i]);
        emg_iir_init(&filter->emg_highpass_iir_state[i]);
        filter->emg_lowpass_iir_state[i].coeffs = emg_lowpass_filter_sos; 
        filter->emg_highpass_iir_state[i].coeffs = emg_highpass_filter_sos; 
        filter->emg_mwa_state[i].sum = 0;
        filter->emg_mwa_state[i].past_sum = 0;
        filter->emg_mwa_state[i].alpha = EMG_RMS_MA_DC_BLOCK_ALPHA;
        filter->emg_mwa_state[i].write_ptr = 0; 
    }
}

#define IIR_SCALE 8388608 // 2^23 scale down to prevent float overflows in processing

int
iir_filter_process(iir_filter_t *filter, float *norms, int numchans,
                   elapsed_t ts_in, const uint8_t *buffer,
                   elapsed_t *ts_out, float *uv_out)
{
    uint8_t *local_buf_ptr = (uint8_t *) buffer;

    filter->emg_rms_sub_sample_counter--;
    for (int i = 0; i < numchans; i++) {
        int16_t tf = READ_BE(int16_t, local_buf_ptr);
        float uv = (float) tf/IIR_SCALE;
        local_buf_ptr += 2;
        uv = emg_iir(&filter->emg_lowpass_iir_state[i], uv);
        if (filter->emg_rms_sub_sample_counter <= 0)
        {
            float emg_bp_filtered = emg_iir(&filter->emg_highpass_iir_state[i], uv);
            uv = emg_mwa_rms(&filter->emg_mwa_state[i], emg_bp_filtered);
            uv_out[i] = uv * IIR_SCALE * norms[i];  // scale up and normalize to uv
        }
    }

    if (filter->emg_rms_sub_sample_counter <= 0)
    {
        *ts_out = ts_in;
        filter->emg_rms_sub_sample_counter = EMG_RMS_SUBSAMPLING_FACTOR;
        return 0;
    }

    return -1;
}
