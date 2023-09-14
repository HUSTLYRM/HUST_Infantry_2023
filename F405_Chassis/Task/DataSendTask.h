#ifndef __DATASENDTASK_H
#define __DATASENDTASK_H

#include "stdint.h"

typedef	__packed struct{
		short HeatMax17;//最大热量
		short shooterHeat17;//当前热量
		unsigned char RobotRed;//红蓝方
		unsigned char HeatCool17;//当前冷却，数值未超过256，可以用unsiged cahr发送
		unsigned char BulletSpeedLevel;//射速等级
		unsigned char RobotLevel;//机器人等级
}JudgeReceive_Info_Typedef;
//#pragma pack(1) 
typedef struct F105{
	short ChassisSpeedw; //1KHZ底盘反馈
	JudgeReceive_Info_Typedef Sendmessage;//真正发送的数据 10HZ 裁判系统速率
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

