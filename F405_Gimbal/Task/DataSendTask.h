#ifndef __DATASENDTASK_H__
#define __DATASENDTASK_H__

#include "stdint.h"
typedef struct{
	char SuperPowerLimit;	//0为超级电容关闭，不为0则开启使用超级电容
	char Chassis_Flag;			//0代表正常模式  1代表小陀螺  2代表单挑模式  3表示掉电
	char AutoFire_Flag;					//0表示手动开火，1为自动开火
	char Laser_Flag;				//0表示激光关闭，1为打开
	short Pitch_100;				//pitch角度,乘了100之后发
  short Yaw_100;				    //yaw角度,乘了100之后发
	char Gimbal_Flag;				//0为正常模式  1为辅瞄
	char Graphic_Init_Flag;	//0为进入初始化模式，1为初始化结束
	char Freq_state;			  //0表示正常射频 1表示高射频
	/*打包数据*/
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
#define Tx2_Off 0xff				//关机模式
#define Tx2_Small_Buff 0x10	//小符模式
#define Tx2_Big_Buff 0x20		//大符模式
#define Tx2_Armor 0x30			//辅瞄模式

#endif
