/**
 ******************************************************************************
 * @file	 TD.h
 * @author  Wang Hongxi
 * @version V1.1.3
 * @date    2021/7/3
 * @brief
 ******************************************************************************
 * @attention
 * copied from https://github.com/WangHongxi2001/RoboMaster-C-Board-INS-Example
 ******************************************************************************
 */

#ifndef _TD_H
#define _TD_H

#include "main.h"
#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "counter.h"
#include "user_lib.h"
#include "arm_math.h"
#include <math.h>

/*************************** Tracking Differentiator ***************************/
typedef struct
{
    float Input;

    float h0;
    float r;

    float x;
    float dx;
    float ddx;

    float last_dx;
    float last_ddx;

    uint32_t DWT_CNT;
    float dt;
} TD_t;

void TD_Init(TD_t *td, float r, float h0);
float TD_Calculate(TD_t *td, float input);
void TD_Clear(TD_t *td, float x);

#endif // !_TD_H
