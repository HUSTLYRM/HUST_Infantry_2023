#ifndef __FIR_H__
#define __FIR_H__

void Fir(float *Input,float *Output);
float LowPass_SetChassis(float old,float In);
float LowPass_SetSteer(float old,float In);
float LowPass_SetWheel(float In,float past);
float LowPass_SetK(float old,float In);
float LowPass_SetK1(float old,float In);
float LowPass_SetK5(float old,float In);
#define ORDER        5                       //½×Êý
#endif
