#ifndef __EMG_IIR_H__
#define __EMG_IIR_H__

#include <math.h>
#include "utils.h"

// are assumed to be the same
#define NO_OF_BQS (3)
#define NO_OF_COEFFS_PER_BQ (6)

#define EMG_RMS_FS (2000)
#define EMG_RMS_FILTER_ORDER (6)
#define EMG_RMS_HIGHPASS_FC (50)
#define EMG_RMS_LOWPASS_FC (199)
#define EMG_RMS_SUBSAMPLING_FACTOR (5)  
#define EMG_RMS_MA_SIZE (300/(EMG_RMS_SUBSAMPLING_FACTOR))
#define EMG_RMS_MA_DC_BLOCK_FC (0.001)
#define EMG_RMS_MA_DC_BLOCK_ALPHA (2*M_PI*EMG_RMS_MA_DC_BLOCK_FC/EMG_RMS_FS)

#define IIR_NUM_CHANNELS 8

typedef struct emg_filter_state {
    float x[NO_OF_BQS][3];
    float y[NO_OF_BQS][2];
    float (*coeffs)[NO_OF_COEFFS_PER_BQ];
} emg_filter_state_t;

typedef struct emg_mwa_state {
    float mw[EMG_RMS_MA_SIZE];
    float sum;
    float past_sum; 
    float dc; 
    float alpha;
    int write_ptr;
} emg_mwa_state_t;

typedef struct iir_filter_t {
    int emg_rms_sub_sample_counter;
    emg_filter_state_t emg_lowpass_iir_state[IIR_NUM_CHANNELS]; 
    emg_filter_state_t emg_highpass_iir_state[IIR_NUM_CHANNELS]; 
    emg_mwa_state_t emg_mwa_state[IIR_NUM_CHANNELS];
} iir_filter_t;

float emg_iir(emg_filter_state_t *, float);
void emg_iir_init(emg_filter_state_t *);
double emg_mwa_rms(emg_mwa_state_t *, float);

// 
void iir_filter_init(iir_filter_t *filter);
int iir_filter_process(iir_filter_t *filter, float *norms, int numchans,
                        elapsed_t ts_in, const uint8_t *buffer,
                        elapsed_t *ts_out, float *uv_out);


#endif //_EMG_IIR_H_
