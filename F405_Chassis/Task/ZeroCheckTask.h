#ifndef __ZEROCHECKTASK_H
#define __ZEROCHECKTASK_H

#define Position 1
#define Speed    2

typedef struct 
{
	float Circle;           //转过圈数
	float CountCycle;       //转过一圈的总计数周期
	float LastValue;        //检测过零量上一次的值	
	float ActualValue;      //检测过零量当前值
	float PreError;         //检测量判断差值
}
ZeroCheck_Typedef;

/*电机数据结构体*/
typedef struct 
{
	float Motor;						//Pitch/Yaw轴电机角度
	float MotorTransAngle;	//Pitch/Yaw轴电机转角(电机角度-初始角度)
	float Circle;
	short RealSpeed;
}
Motor6020Data_Typedef;


typedef struct 
{
	float Gyro;							//Pitch/Yaw轴陀螺仪角度
	float Circle;
}
ChassisYawData_Typedef;
/*电机结构体*/
typedef struct 
{
	Motor6020Data_Typedef Pitch;
	Motor6020Data_Typedef Yaw;
}
Gimbal_Typedef;

float ZeroCheck(ZeroCheck_Typedef *Zero,float value,short Zerocheck_mode);
void ZeroCheck_cal(void);
void ZeroCheck_Init(void);

#endif

