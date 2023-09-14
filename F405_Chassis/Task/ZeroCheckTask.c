/**********************************************************************************************************
 * @文件     ZeroCheckTask.c
 * @说明     过零检测
 * @版本  	 V1.0
 * @作者     黄志雄
 * @日期     2020.1
**********************************************************************************************************/
#include "main.h"
/*--------------------------结构体----------------------------*/
ZeroCheck_Typedef ZeroCheck_Yaw_Steering[4];
/*--------------------------外部引用变量----------------------------*/
extern chassis_type chassis;
/**********************************************************************************************************
*函 数 名: ZeroCheck
*功能说明: 位置式和速度式过零检测
					 Zero->ActualValue 表示检测量当前值
					 Zero->LastValue 表示检测量上一次值
					 Zero->CountCycle 表示检测量过零时越变值，即计数周期
					 Zero->PreError 表示检测量差值
					 使用此函数前要申明对应检测量结构体的 Zero->CountCycle与Zero->LastValue
*形    参: ZeroCheck_Typedef *Zero  过零检测结构体
  *        float value  待检测量
*返 回 值: 取决于Zerocheck_mode，分别输出过零检测后位置值或速度值
**********************************************************************************************************/
float ZeroCheck(ZeroCheck_Typedef *Zero,float value,short Zerocheck_mode)
{
	Zero->ActualValue=value;
	
	Zero->PreError=Zero->ActualValue-Zero->LastValue;
	Zero->LastValue=Zero->ActualValue;
	
	if(Zero->PreError>0.7f*Zero->CountCycle)
	{
		Zero->PreError=Zero->PreError-Zero->CountCycle;
		Zero->Circle++;
	}
	if(Zero->PreError<-0.7f*Zero->CountCycle)
	{
		Zero->PreError=Zero->PreError+Zero->CountCycle;
		Zero->Circle--;
	}
	
	if(Zerocheck_mode==Position)
		return Zero->ActualValue - Zero->Circle*Zero->CountCycle;
	else if(Zerocheck_mode==Speed)
	  return Zero->PreError;
	else 
		return 0;
}



/**********************************************************************************************************
*函 数 名: ZeroCheck_cal
*功能说明: 过零检测执行函数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ZeroCheck_cal(void)
{

	short i=0;
	for (i=0;i<4;i++)
	{
		chassis.SteerMotorsDecode[i].Motor = ZeroCheck(&(ZeroCheck_Yaw_Steering[i]),chassis.SteerMotorsCanReceive[i].Angle,Position);
		chassis.SteerMotorsDecode[i].MotorTransAngle = (chassis.SteerMotorsDecode[i].Motor-chassis.SteeringAnglePosition_init[i])/8192.0f*360.0f;
		chassis.SteerMotorsDecode[i].Circle = ZeroCheck_Yaw_Steering[i].Circle;
		chassis.SteerMotorsDecode[i].RealSpeed = chassis.SteerMotorsCanReceive[i].RealSpeed;
	}	

}


/**********************************************************************************************************
*函 数 名: ZeroCheck_Init
*功能说明: 过零检测结构体参数初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ZeroCheck_Init(void)
{
	short i=0;
	for (i=0;i<4;i++)
	{
		ZeroCheck_Yaw_Steering[i].CountCycle = 8192;
		chassis.SteerMotorsDecode[i].Motor = chassis.SteeringAnglePosition_init[i];
		ZeroCheck_Yaw_Steering[i].LastValue = chassis.SteerMotorsCanReceive[i].Angle;
	}	
}


