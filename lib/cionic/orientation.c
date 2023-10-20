#include "shared-bindings/util.h"
#include "orientation.h"
#include <math.h>

// quaternion is a 4 float array [x,y,z,w]

void
orientation_multiply(float *a, float *b, float *out) 
{
    // val rw = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
    out[3] = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];
    // val rx = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y
    out[0] = a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];
    // val ry = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x
    out[1] = a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0];
    // val rz = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
    out[2] = a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3];
}

void
orientation_inverse(float *a, float *out)
{
    // val d = w*w + x*x + y*y + z*z
    // return Quaternion(-x / d, -y / d, -z / d, w / d)
    float d = a[3] * a[3] + a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
    out[0] = -a[0] / d;
    out[1] = -a[1] / d;
    out[2] = -a[2] / d;
    out[3] = a[3] / d;
}

void
orientation_difference(float *quat_a, float *quat_b, float *quat_out)
{
    float inverse[4];
    orientation_inverse(quat_a, inverse);
    orientation_multiply(inverse, quat_b, quat_out);
}


void
orientation_forward(float rotation, float *out)
{
    float half = rotation/2;
    out[0] = 0.0;
    out[1] = 0.0;
    out[2] = sin(half);
    out[3] = cos(half);
}

// apply calibration to quat_in to produce quat_out
//
void 
orientation_normalize(float *calibration, float *quat_in, float *quat_out)
{
    float uprighted[5];
    orientation_multiply(quat_in, calibration, uprighted);
    
    float forward[5];
    orientation_forward(calibration[4], forward);

    float forwarded[5];
    orientation_multiply(uprighted, forward, forwarded);

    float norm[5];
    orientation_forward(-calibration[4], norm);
    
    orientation_multiply(norm, forwarded, quat_out);
} 


void 
orientation_quaternion_to_euler(float *q, float *e)
{
    // roll (x-axis rotation)
    float sinr = 2.0 * (q[3] * q[0] + q[1] * q[2]);
    float cosr = 1.0 - 2.0 * (q[0] * q[0] + q[1] * q[1]);
    e[0] = atan2(sinr, cosr);

    // pitch (y-axis rotation)
    float sinp = 2.0 * (q[3] * q[1] - q[2] * q[0]);
    if (fabs(sinp) < 1) {
        e[1] = asin(sinp);
    }
    else {
        e[1] = copysign(M_PI_2, sinp);
    }
    
    // yaw (z-axis rotation)
    float siny =  2.0 * (q[3] * q[2] + q[0] * q[1]);
    float cosy = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]);
    e[2] = atan2(siny, cosy);
}

/*
// a function that could be useful in the future
def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

*/

void 
euler_to_orientation_quaternion(float *e, float *q)
{
    float roll, pitch, yaw;
    roll = e[0];
    pitch = e[1];
    yaw = e[2];
    
    q[0] = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2);
    q[1] = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2);
    q[2] = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2);
    q[3] = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2);
}


// clear calibration vector
int
orientation_cal_clear(float *calibration)
{
    calibration[0] = 0; // i
    calibration[1] = 0; // j
    calibration[2] = 0; // k
    calibration[3] = 1; // real
    calibration[4] = 0; // rotation
    return 0;
}

// calibrate upright
int orientation_cal_upright(float *quat, float *calibration)
{
    orientation_inverse(quat, calibration);
    return 0;
}

// calibrate forward
int orientation_cal_forward(float *quat, float *calibration)
{
    float calibrated[4];
    float euler[3];

    // apply rotation greedily until find zero pitch
    for (float i=-180.0; i <= 180.0; i += 1.0) {
        calibration[4] = i * 0.0174533; // convert degrees to radians
        orientation_normalize(calibration, quat, calibrated);
        orientation_quaternion_to_euler(calibrated, euler);
        if (RAD2DEG(euler[0]) > 0 && RAD2DEG(euler[1]) == 0) {
            return 0;
        }
    }
    calibration[4] = 0;
    return -1;
}

