#ifndef __DATARECEIVETASK_H__
#define __DATARECEIVETASK_H__
#include "usart1.h"
#include "stm32f4xx.h"

/*ң�����ṹ��*/
typedef __packed struct
{
		unsigned short ch0;
		unsigned short ch1;
		unsigned short ch2;
		unsigned short ch3;
		unsigned short s1;
		unsigned short s2;
}Remote;
/*���ṹ��*/
typedef __packed 	struct
{
		short x;
		short y;
		short z;
		unsigned char press_l;
		unsigned char press_r;
}Mouse;
/*���̽ṹ��*/
typedef __packed struct
{
		unsigned short w,s,a,d,q,e,r,f,g,z,x,c,v,b,shift,ctrl;
}Key;
/*ң����ṹ���ۺ�*/
typedef __packed struct
{
	Remote rc;
	Mouse mouse;
	Key key;
  char RCrecvd,RCDisconnectCnt;//RCrecvdΪ���ݽ��ձ�־λ
}RC_Ctl_t;
/*�����ǽ��սṹ��*/
typedef struct GYRO{
		float GX;
		float GY;				//�������ٶ�
		float GZ;
		float PITCH;		//�����ǽǶ�
		float YAW;
		float ROLL;
}
Gyro_Typedef;
/*���ʰ���սṹ��*/

typedef	__packed struct{
		short HeatMax17;//�������
		short shooterHeat17;//��ǰ����
		unsigned char RobotRed;//������
		unsigned char HeatCool17;//��ǰ��ȴ����ֵδ����256��������unsiged cahr����
		unsigned char BulletSpeedLevel;//���ٵȼ�
		unsigned char RobotLevel;//�����˵ȼ�
}JudgeReceive_Info_Typedef;

typedef struct F105{
	short ChassisSpeedw;
	JudgeReceive_Info_Typedef JudgeReceive_info;
	
}F105_Typedef;

typedef struct
{
	float RCPitch;
	float RCYaw;
	float RCdistance;
	short ReceiveFromTx2BullectCnt;
	short FrictionWheel_speed;
	short DisConnect;
}PC_Receive_t;

typedef struct{
	short Angle;
	short RealSpeed;  
  short Current;	
}BodanMotorReceive_Typedef;

typedef struct{
unsigned int RC_DisConnect;//ң�������߼��
unsigned int F105_DisConect;//���ʰ���߼��
unsigned int PitchMotor_DisConnect;
unsigned int YawMotor_DisConnect;
unsigned int Gyro_DisConnect;
unsigned int Friction_DisConnect[2];
unsigned int Pluck_DisConnect;
unsigned int PC_DisConnect;
}Disconnect;

void Can1Receive1(CanRxMsg rx_message1);
void Can1Receive0(CanRxMsg rx_message1);
void Can2Receive0(CanRxMsg rx_message0);
void Can2Receive1(CanRxMsg *rx_message1);

void RemoteReceive(volatile unsigned char rx_buffer[]);

void RC_Rst(void);
void F105_Rst(void);

void RCReceive_task(void);
void PCReceive_task(void);

#define ARMOR_NO_AIM	0xff	//û���ҵ�Ŀ��
#define ARMOR_AIMED		0x30	//��Ŀ��

enum ARMOR_ID
{
    ARMOR_AIM_LOST = 0,
    ARMOR_ID_1,
    ARMOR_ID_2,
    ARMOR_ID_3,
    ARMOR_ID_4,
    ARMOR_ID_5,
    ARMOR_ID_Sentry,
};

#endif
