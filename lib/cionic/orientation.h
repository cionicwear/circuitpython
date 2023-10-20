#ifndef __ORIENTATION_H__
#define __ORIENTATION_H__

#define DEG2RAD(d) ((float)d * 0.0174533)
#define RAD2DEG(r) ((int)(r*57.2958))

// quat_a * quat_b output to quat_out
//
void orientation_multiply(float *quat_a, float *quat_b, float *quat_out);

// perform quaterion invers on quat_in to produce quat_out
//
void orientation_inverse(float *quat_in, float *quat_out);

// calculate difference between quat_a and quat_b and output to quat out
//
void orientation_difference(float *quat_a, float *quat_b, float *quat_out);

void orientation_forward(float rotation, float *out);

// apply calibration to quat_in to produce quat_out
//
void orientation_normalize(float *calibration, float *quat_in, float *quat_out);

// convert a quaternion to euler
//
void orientation_quaternion_to_euler(float *quaternion, float *euler);

// convert euler to quaternion 
//
void euler_to_orientation_quaternion(float *euler, float *quaternion);

// clear calibration vector
int orientation_cal_clear(float *calibration);

// calibrate upright
int orientation_cal_upright(float *quat, float *calibration);

// calibrate forward
int orientation_cal_forward(float *quat, float *calibration);

#endif
