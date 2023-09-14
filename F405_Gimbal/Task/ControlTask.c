/**********************************************************************************************************
 * @�ļ�     ControlTask.c
 * @˵��     ���
 * @�汾  	 V1.0
 * @����     ���κ�
 * @����     2020.1
 **********************************************************************************************************/
#include "main.h"
/*--------------�ڲ�����-------------------*/

/*---------------�ṹ��--------------------*/

/*--------------�ⲿ����-------------------*/
extern Status_t Status;
extern Gimbal_Typedef Gimbal;
extern ChassisSpeed_t chassis;
extern RobotInit_Struct Infantry;
extern F105_Typedef F105;
extern F405_typedef F405;
/**********************************************************************************************************
 *�� �� ��: Chassic_Control
 *����˵��: ���̷��Ϳ���
 *��    ��: pvParameters
 *�� �� ֵ: ��
 **********************************************************************************************************/
extern short BodanMotorCurrent,FrictionCurrent[2];
void Shoot_Motor_Control(void){
	ShootCount_Cal();
	BodanMotor_CurrentPid_Cal();
	FrictionWheel_CurrentPid_Cal();
	BodanCan1Send(BodanMotorCurrent);
	FrictionCan2Send(FrictionCurrent[0],FrictionCurrent[1]);
}
/**********************************************************************************************************
 *�� �� ��: Chassic_Control
 *����˵��: ���̷��Ϳ���
 *��    ��: pvParameters
 *�� �� ֵ: ��
 **********************************************************************************************************/
extern Pid_Typedef pidChassisPosition,pidChassisPosition_Speed;
extern Pid_Typedef SOLO_pidChassisPosition;
extern FeedForward_Typedef FF_w;
extern float ResetPos;
void Chassic_Control(void){
	switch (Status.ChassisMode)
	{
		case Chassis_Act_Mode:
			if(Status.GimbalMode == Gimbal_DropShot_Mode || Status.GimbalMode == Gimbal_BigBuf_Mode || Status.GimbalMode == Gimbal_SmlBuf_Mode)//����ģʽ
			{	
				chassis.carSpeedw = 0;
			}else{
				//����PID����+ǰ��
				pidChassisPosition.ActualValue = Gimbal.Yaw.Motor;
				chassis.carSpeedw = -(-PID_Calc(&pidChassisPosition) +  FeedForward_Calc(&FF_w));
			}	
				
			break;

		case Chassis_SelfProtect_Mode:
				SOLO_pidChassisPosition.ActualValue = ResetPos;
				pidChassisPosition_Speed.SetPoint = -PID_Calc(&SOLO_pidChassisPosition);
				pidChassisPosition_Speed.ActualValue = F105.ChassisSpeedw;
				chassis.carSpeedw = PID_Calc(&pidChassisPosition_Speed);
			break;

		case Chassis_Solo_Mode:
			//Chassis_Solo_Cal(RC_Ctl.rc,RC_Ctl.key);
			break;

		case Chassis_Jump_Mode:
			//Chassis_Jump_Cal(RC_Ctl.rc,RC_Ctl.key);
			break;

		case Chassis_Powerdown_Mode:
			Chassis_Powerdown_Cal();
			break;
	
		default:
			Chassis_Powerdown_Cal();
			break;
	}
	F405.Chassis_Flag = Status.ChassisMode;

	ChassisCan1Send(&chassis.carSpeedx,&chassis.carSpeedy,&chassis.carSpeedw);
}

/**********************************************************************************************************
 *�� �� ��: Gimbal_motor_Control
 *����˵��: ��̨����������
 *��    ��: pvParameters
 *�� �� ֵ: ��
 **********************************************************************************************************/
extern char  pitch_lose_flag;
extern short YawCurrent, PitchCurrent;

void Gimbal_Motor_Control(void){
	
	/******************************pid����*******************************************/
	Gimbal_CurrentPid_Cal();
	/******************************���ݷ���*******************************************/
	if (!pitch_lose_flag) //����ת
	{
		// GimbalCan2Send(PitchCurrent,YawCurrent);
		YawCan1Send(YawCurrent);
		PitchCan2Send(PitchCurrent);
		//PitchCan2Send(0);
	}
	else
	{
		// GimbalCan2Send(0,YawCurrent);
		YawCan1Send(YawCurrent);
		PitchCan2Send(0);
	}
}
/**********************************************************************************************************
 *�� �� ��: ModeChoose_task
 *����˵��: ģʽѡ������
 *��    ��: pvParameters
 *�� �� ֵ: ��
 **********************************************************************************************************/
uint32_t Control_high_water;
uint32_t time_last=0;
float dt_1=0;
void Control_task(void *pvParameters)
{
	portTickType xLastWakeTime;
	const portTickType xFrequency = 2;
	vTaskDelay(300);
	
	while (1)
	{
		xLastWakeTime = xTaskGetTickCount();
		dt_1 = GetDeltaT(&time_last);
		ZeroCheck_cal();
		Gimbal_Motor_Control();
		Shoot_Motor_Control();
//		YawCan1Send(0);
//		PitchCan2Send(0);
		
		IWDG_Feed();
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

#if INCLUDE_uxTaskGetStackHighWaterMark
		Control_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
	}
}
