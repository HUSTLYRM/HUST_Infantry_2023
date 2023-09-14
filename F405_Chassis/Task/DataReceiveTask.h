#ifndef __DATARECEIVETASK_H
#define __DATARECEIVETASK_H
#include "main.h"

#define Chassis_Powerdown_Mode 					0
#define Chassis_Act_Mode  							1
#define Chassis_SelfProtect_Mode 				2
#define Chassis_Solo_Mode  							3
#define Chassis_Jump_Mode               4
#define Chassis_Test_Mode               5

#define Gimbal_Powerdown_Mode 					7
#define Gimbal_Act_Mode 								3
#define Gimbal_Armor_Mode  							0
#define Gimbal_BigBuf_Mode 						  2
#define Gimbal_DropShot_Mode 						4
#define Gimbal_SI_Mode 									5
#define Gimbal_Jump_Mode                6
#define Gimbal_AntiSP_Mode              7
#define Gimbal_SmlBuf_Mode              1

typedef struct{
	char SuperPowerLimit;	  //0Ϊ�������ݹرգ���Ϊ0����ʹ�ó�������
	char Chassis_Flag;			//ģʽ����
	char AutoFire_Flag;					//0��ʾ�ֶ�����1Ϊ�Զ�����
	char Laser_Flag;				//0��ʾ����رգ�1Ϊ��
	short Pitch_100;				//pitch�Ƕ�,����100֮��
    short Yaw_100;			    	//yaw�Ƕ�,����100֮��
	char Gimbal_Flag;				//ģʽ����
	char Graphic_Init_Flag;	//0Ϊ�����ʼ��ģʽ��1Ϊ��ʼ������
	char Freq_state;			  //��Ƶ״̬��0��ʾ������Ƶ��1��ʾ����Ƶ
    char Enemy_ID;
	/*�������*/
	char Send_Pack1;	
	char Fric_Flag;
}F405_typedef;

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


typedef struct 
{
  short carSpeedx;
	short carSpeedy;
	short carSpeedw;
	
	short Last_carSpeedx;
	short Last_carSpeedy;
	short Last_carSpeedw;
	
	short ABSLastcarSpeedx;
	short ABSLastcarSpeedy;
	short ABSLastcarSpeedw;
} ChassisSpeed_t;

typedef struct
{
  char HeatUpdate_NoUpdate;
	char SpeedUpdate_NoUpdate;
	
	//0x0201
	uint8_t robot_id;
	uint8_t RobotLevel;
	uint16_t remainHP;
	uint16_t maxHP;
	uint16_t HeatCool17;		//17mmǹ��ÿ����ȴֵ
	uint16_t HeatMax17;			//17mmǹ����������
	uint16_t BulletSpeedMax17;	//17mmǹ�������ٶ�
	uint16_t MaxPower;			//���̹�����������

	//0x0202
	uint16_t realChassisOutV;
	uint16_t realChassisOutA;
	float realChassispower;
	uint16_t remainEnergy;       //ʣ������
	short shooterHeat17;
	
	//0x0207
	uint8_t bulletFreq;		//���Ƶ��
	uint8_t ShootCpltFlag; //�����һ���ӵ���־λ
	
	//0x0208
  uint16_t num_17mm;
  uint16_t num_coin;
	
	//flag
	short HeatUpdateFlag;	
	
	//not in use
	uint8_t cardType;
	uint8_t CardIdx;

	float bulletSpeed;		//��ǰ����
	float LastbulletSpeed;
    
	uint8_t game_progress;
	
	uint16_t enemy_3_hp;
	uint16_t enemy_4_hp;
	uint16_t enemy_5_hp;
	
	uint16_t remain_time;
	
	char airforceStatus;//��Ѫԭ��
}
JudgeReceive_t;


typedef struct{
	short Angle;
	short RealSpeed;  
	short Current;	
	char temp;
}RM820RReceive_Typedef;
typedef struct
{
	short Angle;
	short RealSpeed;  
	short Current;	
	char temp;
}Motor6020Receive_Typedef;

void Can1Receive0(CanRxMsg rx_message0);
void Can1Receive1(CanRxMsg rx_message0);
void Can2Receive1(CanRxMsg *rx_message);

void JudgeBuffReceive(unsigned char ReceiveBuffer[],uint16_t DataLen);

void F405_Rst(void);
void JudgeReceive_task(void);


#endif
