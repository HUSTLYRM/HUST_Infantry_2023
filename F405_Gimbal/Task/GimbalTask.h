#ifndef __GIMBALTASK_H
#define __GIMBALTASK_H

#include "main.h"
#include "stm32f4xx.h"
//#define gimbal_pitch_max 24
#define gimbal_pitch_mid -30
enum controlchoose{           //云台电机模式和陀螺仪模式切换用
    GyroMode = 0,
    MotorMode
};
//void Gimbal_DropShot_Cal(Remote rc,Mouse mouse);
void Gimbal_Armor_Cal(Remote rc,Mouse mouse);
void Gimbal_Buff_Cal(Remote rc,Mouse mouse);
void Gimbal_SI_Cal(float Gimbal_yaw,float Gimbal_pitch);
void Gimbal_Powerdown_Cal(void);
void MotorGimbal_Act_Cal(Remote rc,Mouse mouse);
void GyroGimbal_Act_Cal(Remote rc,Mouse mouse);

void get_F(void);
void T_change(void);

	
void Gimbal_CurrentPid_Cal(void);

void PidGimbalMotor_Init(void);


void Gimbal_task(void *pvParameters);
#endif
