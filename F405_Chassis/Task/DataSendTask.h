#ifndef __DATASENDTASK_H
#define __DATASENDTASK_H

#include "stdint.h"

typedef	__packed struct{
		short HeatMax17;//�������
		short shooterHeat17;//��ǰ����
		unsigned char RobotRed;//������
		unsigned char HeatCool17;//��ǰ��ȴ����ֵδ����256��������unsiged cahr����
		unsigned char BulletSpeedLevel;//���ٵȼ�
		unsigned char RobotLevel;//�����˵ȼ�
}JudgeReceive_Info_Typedef;
//#pragma pack(1) 
typedef struct F105{
	short ChassisSpeedw; //1KHZ���̷���
	JudgeReceive_Info_Typedef Sendmessage;//�������͵����� 10HZ ����ϵͳ����
	//no use
	uint8_t which_balance;
}F105_Typedef;

void ChassisCan1Send(short a,short b,short c,short d);
void Can2Send0(F105_Typedef *F105_Send);
void Can2Send1(JudgeReceive_Info_Typedef *F105_Send);
void Can2Send2(uint8_t BuffState);
void SteerCan1Send(short a,short b,short c,short d);
void USART2SEND(void);

#endif 

