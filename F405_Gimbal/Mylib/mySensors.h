#ifndef MY_SENSORS_H
#define MY_SENSORS_H

#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "mySensors.h"
typedef struct Gyro
{
    float a_x;
    float a_y;
    float a_z;
    float a_t;

    float w_x;
    float w_y;
    float w_z;
    float w_t;

    float x_roll;
    float x_pitch;
    float x_yaw;
    float x_t;

    float yaw_base;
} Gyro;

typedef struct Motor
{
    float speed;
    float get_speed;
    float pos;
    float pos_out;
    float get_pose;
    float torque;
    float get_torque;
} Motor;

extern Gyro gyro;
extern Motor motors[4];

void decode_gyro(uint8_t *buf);

#endif // !_SENSORS_H
