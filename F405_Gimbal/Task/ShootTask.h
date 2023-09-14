#ifndef __SHOOTTASK_H
#define __SHOOTTASK_H

#include "main.h"
#include "DataReceivetask.h"

#define PluckThreholdPos        5000       
#define ShootInterval_Min_time 50 //最快连射的速度是50ms一发(连射情况)

#define QueenSampling_number 20 //采样QueenSampling_number个转速来积分
#define QueenSampling_rate 3 //control task每执行QueenSampling_rate次采样一次摩擦轮转速来积分。减少计算量


typedef struct{
	struct{
		float ShootCount_IntervalTime;//子弹射击估计时上一颗子弹打出至今的时间
		float ShootContinue_IntervalTime;//连射时上一颗子弹打出至今的时间
		short HeatUpdateFlag;//热量信息更新
		short CurShootNumber;//还未处理热量的子弹数
		char IsShootAble;//可以打弹
		float HeatMax17, HeatCool17;
		float CurHeat17, LastHeat17;
	}HeatControl;
	
	char ReverseRotation;
	char ShootContinue;
}ShootTask_typedef;

void FrictionSpeedChoose(void);


void ShootCount_Cal(void);
void Shoot_Check_Cal(void);
void Shoot_Fire_Cal(void);
void Shoot_Powerdown_Cal(void);
void Shoot_Tx2_Cal(void);
void Shoot_Test_Cal(void);
void HeatControl(float dt);
void FrictionWheel_CurrentPid_Cal(void);
void BodanMotor_CurrentPid_Cal(void);
void Pid_BodanMotor_Init(void);
void Pid_Friction_Init(void);
void Shoot_task(void *pvParameters);

#endif
