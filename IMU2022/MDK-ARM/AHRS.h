#ifndef _AHRS_H_
#define _AHRS_H_
#include "main.h"

typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

void AHRS_task();
void AHRS_init(fp32 quat[4], fp32 accel[3], fp32 mag[3]);
void get_angle(fp32 q[4], fp32 *yaw, fp32 *pitch, fp32 *roll);
void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3], fp32 mag[3]);

// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
//----------------------------------------------------------------------------------------------------
// Variable declaration

extern volatile float twoKp;			// 2 * proportional gain (Kp)
extern volatile float twoKi;			// 2 * integral gain (Ki)
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations

void MahonyAHRSupdate(float q[4], float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az);


//=====================================================================================================
// End of file
//=====================================================================================================

#endif