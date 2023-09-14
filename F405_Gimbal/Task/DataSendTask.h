#ifndef __DATASENDTASK_H__
#define __DATASENDTASK_H__

#include "stdint.h"
typedef struct{
	char SuperPowerLimit;	//0Ϊ�������ݹرգ���Ϊ0����ʹ�ó�������
	char Chassis_Flag;			//0��������ģʽ  1����С����  2������ģʽ  3��ʾ����
	char AutoFire_Flag;					//0��ʾ�ֶ�����1Ϊ�Զ�����
	char Laser_Flag;				//0��ʾ����رգ�1Ϊ��
	short Pitch_100;				//pitch�Ƕ�,����100֮��
  short Yaw_100;				    //yaw�Ƕ�,����100֮��
	char Gimbal_Flag;				//0Ϊ����ģʽ  1Ϊ����
	char Graphic_Init_Flag;	//0Ϊ�����ʼ��ģʽ��1Ϊ��ʼ������
	char Freq_state;			  //0��ʾ������Ƶ 1��ʾ����Ƶ
	/*�������*/
	char Send_Pack1;	
  char Fric_Flag;	
}F405_typedef;

//void BodanCan1Send(short a);
void F405Can1Send(F405_typedef *F405_Send);
//void GimbalCan2Send(short X,short Y);
void ChassisCan1Send(short *carSpeedx,short *carSpeedy,short *carSpeedw);
void YawCan1Send(short tempX);
void PitchCan2Send(short tempX);
void BodanCan1Send(short tempX);
void FrictionCan2Send(short tempX,short tempY);

void USART6_SendtoPC(void);
void Tx2_Off_CheckAndSet(uint8_t* Buff);
void TX2_task(void *pvParameters);
//void FrictionBodanCan2Send(short X,short Y,short Z);
#define Tx2_Off 0xff				//�ػ�ģʽ
#define Tx2_Small_Buff 0x10	//С��ģʽ
#define Tx2_Big_Buff 0x20		//���ģʽ
#define Tx2_Armor 0x30			//����ģʽ

#endif
