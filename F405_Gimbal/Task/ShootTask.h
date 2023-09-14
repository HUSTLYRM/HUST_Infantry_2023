#ifndef __SHOOTTASK_H
#define __SHOOTTASK_H

#include "main.h"
#include "DataReceivetask.h"

#define PluckThreholdPos        5000       
#define ShootInterval_Min_time 50 //���������ٶ���50msһ��(�������)

#define QueenSampling_number 20 //����QueenSampling_number��ת��������
#define QueenSampling_rate 3 //control taskÿִ��QueenSampling_rate�β���һ��Ħ����ת�������֡����ټ�����


typedef struct{
	struct{
		float ShootCount_IntervalTime;//�ӵ��������ʱ��һ���ӵ���������ʱ��
		float ShootContinue_IntervalTime;//����ʱ��һ���ӵ���������ʱ��
		short HeatUpdateFlag;//������Ϣ����
		short CurShootNumber;//��δ�����������ӵ���
		char IsShootAble;//���Դ�
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
