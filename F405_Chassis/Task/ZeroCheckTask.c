/**********************************************************************************************************
 * @�ļ�     ZeroCheckTask.c
 * @˵��     ������
 * @�汾  	 V1.0
 * @����     ��־��
 * @����     2020.1
**********************************************************************************************************/
#include "main.h"
/*--------------------------�ṹ��----------------------------*/
ZeroCheck_Typedef ZeroCheck_Yaw_Steering[4];
/*--------------------------�ⲿ���ñ���----------------------------*/
extern chassis_type chassis;
/**********************************************************************************************************
*�� �� ��: ZeroCheck
*����˵��: λ��ʽ���ٶ�ʽ������
					 Zero->ActualValue ��ʾ�������ǰֵ
					 Zero->LastValue ��ʾ�������һ��ֵ
					 Zero->CountCycle ��ʾ���������ʱԽ��ֵ������������
					 Zero->PreError ��ʾ�������ֵ
					 ʹ�ô˺���ǰҪ������Ӧ������ṹ��� Zero->CountCycle��Zero->LastValue
*��    ��: ZeroCheck_Typedef *Zero  ������ṹ��
  *        float value  �������
*�� �� ֵ: ȡ����Zerocheck_mode���ֱ�����������λ��ֵ���ٶ�ֵ
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
*�� �� ��: ZeroCheck_cal
*����˵��: ������ִ�к���
*��    ��: ��
*�� �� ֵ: ��
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
*�� �� ��: ZeroCheck_Init
*����˵��: ������ṹ�������ʼ��
*��    ��: ��
*�� �� ֵ: ��
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


