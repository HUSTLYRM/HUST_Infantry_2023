#ifndef _KALMAN_FILTER_H_
#define _KALMAN_FILTER_H_
#include "stm32f0xx_hal.h"


float Kalman_Filter_pitch_roll(float Accel,float Gyro,float*angle);
float Kalman_Filter_yaw(float Accel,float Gyro,float *angle);	



#endif