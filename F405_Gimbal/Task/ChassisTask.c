/**********************************************************************************************************
 * @�ļ�     ChassisTask.c
 * @˵��     ���̿���
 * @�汾  	 V1.0
 * @����     ��־��
 * @����     2019.10
**********************************************************************************************************/
/**********************************************************************************************************
 * @�ļ�     ChassisTask.c
 * @˵��     ���̿���
 * @�汾  	 V2.0
 * @����     ����
 * @����     2022.4
     ǰx��y
**********************************************************************************************************/
#include "main.h"
/*----------------------------------�ڲ�����---------------------------*/
short ChassisAct_Init_Flag=0;
float Theta,SinTheTa,CosTheTa,TanTheTa,Theta0,Speed_Theta;
float Theta_chassis;
char  SelfProtect_Cross_Flag;
float ResetPos;

const short FollowMaxSpeedw = 2000;			//�������ת��
const short RotateMaxSpeedw = 6000;			//С�������ת��

float LastChassisSpeed = 0;
float LPF_FOLLOW = 0.05;
float LPF_FOLLOW_POS = 0.05;
/*----------------------------------�ṹ��-----------------------------*/
ChassisSpeed_t chassis;
Pid_Typedef pidChassisPosition,pidChassisPosition_Speed;
Pid_Typedef SOLO_pidChassisPosition;
FeedForward_Typedef FF_w;
/*----------------------------------�ⲿ����---------------------------*/
extern RC_Ctl_t RC_Ctl;
extern Status_t Status;
extern Gimbal_Typedef Gimbal;
extern RobotInit_Struct Infantry;
extern F105_Typedef F105;
extern F405_typedef F405;
extern short Turning_flag;
extern float YawMotorReceive;
extern short YawMotorSpeed;
extern uint8_t ControlMode;     //��ʼ����ģʽΪ�����ǿ���
/**********************************************************************************************************
*�� �� ��: Chassis_Powerdown_Cal
*����˵��: ����ģʽ
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Chassis_Powerdown_Cal()
{
	if(ChassisAct_Init_Flag!=Chassis_Powerdown_Mode) 
		ChassisAct_Init_Flag=Chassis_Powerdown_Mode;
	
  chassis.carSpeedx=0;chassis.carSpeedy=0;chassis.carSpeedw=0;
  Theta_chassis = ChassisPostionAngle_TranSform(Infantry.Yaw_init)/360.0f*6.28318f;
}

/**********************************************************************************************************
*�� �� ��: Chassis_Act_Cal
*����˵��: ����ģʽ
*��    ��: rc  key
*�� �� ֵ: ��
**********************************************************************************************************/
short test_w;
void Chassis_Act_Cal(Remote rc,Key key) 
{	
	if(ChassisAct_Init_Flag!=Chassis_Act_Mode)
	{
		chassis.carSpeedw = 0;
    ChassisAct_Init_Flag=Chassis_Act_Mode;
	}
	int speed=2500;
	Theta=ChassisPostionAngle_TranSform(Infantry.Yaw_init)/360.0f*6.28318f;  //����
    Theta_chassis = ChassisPostionAngle_TranSform(Infantry.Yaw_init)/360.0f*6.28318f;
	
	CosTheTa=cos(Theta);
	SinTheTa=sin(Theta);
	TanTheTa=tan(Theta);
	

/***************************ң��������********************************/
#if Robot_ID == 46  //����
	if(Status.ControlMode==Control_RC_Mode)
	{
		if((-1024+RC_Ctl.rc.ch1)>300)
		{
			chassis.carSpeedx= -speed; 
		}
		else if((-1024+RC_Ctl.rc.ch1)<-300)
		{
			chassis.carSpeedx = speed; 
		}
		else
			chassis.carSpeedx = 0; 
	
	  if((-1024+RC_Ctl.rc.ch0)>300)
		{
			chassis.carSpeedy = -speed; 
		}
		else if((-1024+RC_Ctl.rc.ch0)<-300)
		{
			chassis.carSpeedy = speed; 
		}
		else
		{
			chassis.carSpeedy = 0;
		}	
	}
#elif Robot_ID == 5 //ȫ����
	if(Status.ControlMode==Control_RC_Mode)
	{
		if((-1024+RC_Ctl.rc.ch1)>300)  //��ǰ
		{		
			if((-3.1416f/4)>Theta && (-3.1416f*3/4)<Theta)  //��Ϊͷ
			{
				chassis.carSpeedy = -2000;
			}
			else if((3.1416f/4)<Theta && (3.1416f*3/4)>Theta)  //��Ϊͷ
			{
				chassis.carSpeedy = 2000;
			}
			else if((-3.1416f*3/4)>Theta || (3.1416f*3/4)<Theta)  //��Ϊͷ
			{
				chassis.carSpeedx = 2000;
			}
			else  //ǰΪͷ
			{
				chassis.carSpeedx = -2000;
			}
		}
		else if((-1024+RC_Ctl.rc.ch1)<-300)  //���
		{
			if((-3.1416f/4)>Theta && (-3.1416f*3/4)<Theta)  //��Ϊͷ
			{
				chassis.carSpeedy = 2000;
			}
			else if((3.1416f/4)<Theta && (3.1416f*3/4)>Theta)  //��Ϊͷ
			{
				chassis.carSpeedy = -2000;
			}
			else if((-3.1416f*3/4)>Theta || (3.1416f*3/4)<Theta)  //��Ϊͷ
			{
				chassis.carSpeedx = -2000;
			}
			else  //ǰΪͷ
			{
				chassis.carSpeedx = 2000;
			}
		}
		else
		{
			if(((-3.1416f/4)>Theta && (-3.1416f*3/4)<Theta) || ((3.1416f/4)<Theta && (3.1416f*3/4)>Theta)) //�����Ϊͷ
				chassis.carSpeedy = 0;
			else  //ǰ���Ϊͷ
			  chassis.carSpeedx = 0;
		}
		
		if((-1024+RC_Ctl.rc.ch0)>300)  //����
		{
			if((-3.1416f/4)>Theta && (-3.1416f*3/4)<Theta)  //��Ϊͷ
			{
				chassis.carSpeedx = 2000;
			}
			else if((3.1416f/4)<Theta && (3.1416f*3/4)>Theta)  //��Ϊͷ
			{
				chassis.carSpeedx = -2000;
			}
			else if((-3.1416f*3/4)>Theta || (3.1416f*3/4)<Theta)  //��Ϊͷ
			{
				chassis.carSpeedy = 2000;
			}
			else  //ǰΪͷ
			{
				chassis.carSpeedy = -2000;
			}
		}
		else if((-1024+RC_Ctl.rc.ch0)<-300)  //����
		{
			if((-3.1416f/4)>Theta && (-3.1416f*3/4)<Theta)  //��Ϊͷ
			{
				chassis.carSpeedx = -2000;
			}
			else if((3.1416f/4)<Theta && (3.1416f*3/4)>Theta)  //��Ϊͷ
			{
				chassis.carSpeedx = 2000;
			}
			else if((-3.1416f*3/4)>Theta || (3.1416f*3/4)<Theta)  //��Ϊͷ
			{
				chassis.carSpeedy = -2000;
			}
			else  //ǰΪͷ
			{
				chassis.carSpeedy = 2000;
			}
		}
		else
		{
			if(((-3.1416f/4)>Theta && (-3.1416f*3/4)<Theta) || ((3.1416f/4)<Theta && (3.1416f*3/4)>Theta)) //�����Ϊͷ
				chassis.carSpeedx = 0;
			else  //ǰ���Ϊͷ
			  chassis.carSpeedy = 0;
		}
	}
		
#else
	if(Status.ControlMode==Control_RC_Mode)
	{	
	    if((-1024+RC_Ctl.rc.ch1)>300)
			{
				chassis.carSpeedx= speed; 
				if(Theta>3.1416f/2||Theta<-3.1416f/2)
				{
					chassis.carSpeedx= -speed; 
				}
			}
			else if((-1024+RC_Ctl.rc.ch1)<-300)
			{
				chassis.carSpeedx= -speed; 
				if(Theta>3.1416f/2||Theta<-3.1416f/2)
				{
				chassis.carSpeedx= speed; 
				}
			}
			else
			chassis.carSpeedx= 0; 
			
			
		  if((-1024+RC_Ctl.rc.ch0)>300)
			{
				chassis.carSpeedy= speed; 
				if(Theta>3.1416f/2||Theta<-3.1416f/2)
				{
				chassis.carSpeedy= -speed; 
				}
			}
			else if((-1024+RC_Ctl.rc.ch0)<-300)
			{
				chassis.carSpeedy= -speed; 
				if(Theta>3.1416f/2||Theta<-3.1416f/2)
				{
					chassis.carSpeedy= speed; 
				}
			}
			else
			chassis.carSpeedy= 0; 
		}
#endif
	
/***************************���̿���********************************/
#if Robot_ID == 46  //����	
	if(Status.ControlMode==Control_MouseKey_Mode)  
	{
        chassis.carSpeedy = (key.a-key.d)*speed; //����
		chassis.carSpeedx = -(key.w-key.s)*speed; //����
	}
#elif Robot_ID == 5
	if(Status.ControlMode==Control_MouseKey_Mode)  
	{
		if(((-3.1416f/4)>Theta && (-3.1416f*3/4)<Theta) || ((3.1416f/4)<Theta && (3.1416f*3/4)>Theta)) //�����Ϊͷ
		{
			chassis.carSpeedx = + ( (key.a-key.d)*2000*SinTheTa - (key.s-key.w)*2000*CosTheTa );
			chassis.carSpeedy = - ( (key.s-key.w)*2000*SinTheTa + (key.a-key.d)*2000*CosTheTa );
		}
		else  //ǰ���Ϊͷ
		{
			chassis.carSpeedx = - ( - ( (key.a-key.d)*2000*SinTheTa + (key.s-key.w)*2000*CosTheTa ));
		chassis.carSpeedy = - ( + ( (key.s-key.w)*2000*SinTheTa - (key.a-key.d)*2000*CosTheTa ));
			//chassis.carSpeedx = + ( (key.a-key.d)*2000*SinTheTa - (key.s-key.w)*2000*CosTheTa );
			//chassis.carSpeedy = - ( (key.s-key.w)*2000*SinTheTa + (key.a-key.d)*2000*CosTheTa );
		}
	//	  chassis.carSpeedx = -((key.a-key.d)*2000*SinTheTa+(key.s-key.w)*2000*CosTheTa); //����
	//		chassis.carSpeedy = ((key.s-key.w)*2000*SinTheTa-(key.a-key.d)*2000*CosTheTa); //����
	}
#else
	if(Status.ControlMode==Control_MouseKey_Mode)  
	{
        chassis.carSpeedx = -(-(key.a-key.d)*speed*SinTheTa+(key.s-key.w)*speed*CosTheTa); //����
		chassis.carSpeedy = (-(key.s-key.w)*speed*SinTheTa-(key.a-key.d)*speed*CosTheTa); //����
	}

#endif	
	
	if(Status.GimbalMode == Gimbal_DropShot_Mode || Status.GimbalMode == Gimbal_BigBuf_Mode || Status.GimbalMode == Gimbal_SmlBuf_Mode)//����ģʽ
	{	
		chassis.carSpeedw = 0;
	}
	else
	{
		ResetPos = (Theta)/6.28318f*8192;
		/*���̸��� ȫ���ֺͶ���*/	
	#if Robot_ID == 5 || Robot_ID == 46
					
		if((-3.1416f/4)>Theta && (-3.1416f*3/4)<Theta)  //��Ϊͷ
		{
		pidChassisPosition.SetPoint = Gimbal.Yaw.Motor-ResetPos-2048;
		}
		else if((3.1416f/4)<Theta && (3.1416f*3/4)>Theta)  //��Ϊͷ
		{
		pidChassisPosition.SetPoint = Gimbal.Yaw.Motor-ResetPos+2048;
		}
		else if((-3.1416f*3/4)>Theta)  //��
		{
		pidChassisPosition.SetPoint = Gimbal.Yaw.Motor-ResetPos-4096;
		}
		else if((3.1416f*3/4)<Theta)  //��
		{
		pidChassisPosition.SetPoint = Gimbal.Yaw.Motor-ResetPos+4096;
		}
		else 
		{
		pidChassisPosition.SetPoint = Gimbal.Yaw.Motor-ResetPos;
		}
	#else
		if((-3.1416f/2)>Theta)
		{
		pidChassisPosition.SetPoint = +(Gimbal.Yaw.Motor-ResetPos-4096);
		}
		else if((3.1416f/2)<Theta)
		{
		pidChassisPosition.SetPoint = +(Gimbal.Yaw.Motor-ResetPos+4096);
		}
		else 
		{
		pidChassisPosition.SetPoint = +(Gimbal.Yaw.Motor-ResetPos);
		}
		
	#endif	
	
	
	
		if(ControlMode == GyroMode)
		{
			//����PID����+ǰ��
			pidChassisPosition.ActualValue =(LPF_FOLLOW_POS)*pidChassisPosition.ActualValue + (1-LPF_FOLLOW_POS)*Gimbal.Yaw.Motor;
			//    chassis.carSpeedw = -(-PID_Calc(&pidChassisPosition) +  FeedForward_Calc(&FF_w));
			//�¼��ٶȻ�
			pidChassisPosition_Speed.SetPoint = -(-PID_Calc(&pidChassisPosition) );
			pidChassisPosition_Speed.ActualValue = Gimbal.Yaw.AngularSpeed - YawMotorSpeed*360.0f/60.0f /*�˴���rpmתΪ�Ƕ�ÿ��*/;
			pidChassisPosition_Speed.ActualValue = (LPF_FOLLOW)*pidChassisPosition_Speed.ActualValue + (1-LPF_FOLLOW)*LastChassisSpeed;
			LastChassisSpeed = pidChassisPosition_Speed.ActualValue;
			chassis.carSpeedw = PID_Calc(&pidChassisPosition_Speed) + FeedForward_Calc(&FF_w);
		}

	}
	
	
		
	if(F405.SuperPowerLimit==CAP){
		if(ABS(chassis.carSpeedy)>1500){
			chassis.carSpeedy = LIMIT_MAX_MIN(1000*chassis.carSpeedy,8000,-8000);
		}
		if(ABS(chassis.carSpeedx)>1500){
			chassis.carSpeedx = LIMIT_MAX_MIN(1000*chassis.carSpeedx,8000,-8000);
		}
		if(ABS(chassis.carSpeedy)>1500 || ABS(chassis.carSpeedx)>1500){
			chassis.carSpeedw*=1.5;
		}
	}
}



/**********************************************************************************************************
*�� �� ��: Chassis_SelfProtect_Cal
*����˵��: ����ģʽ
*��    ��: rc  key
*�� �� ֵ: ��
**********************************************************************************************************/
float SP_Theta,CosSP_Theta,SinSP_Theta,TanSP_Theta;
float bias_SP1 = 28.0f/360.0f*6.28318f;
float bias_SP2 = -33.0f/360.0f*6.28318f;
void Chassis_SelfProtect_Cal(Remote rc,Key key)
{
	if(ChassisAct_Init_Flag!=Chassis_SelfProtect_Mode)
	  ChassisAct_Init_Flag=Chassis_SelfProtect_Mode;
    
    if(Turning_flag%2 == 1)
	SP_Theta=ChassisPostionAngle_TranSform(Infantry.Yaw_init)/360.0f*6.28318f+bias_SP1;
    else
    SP_Theta=ChassisPostionAngle_TranSform(Infantry.Yaw_init)/360.0f*6.28318f+bias_SP2;
    
//    SP_Theta=ChassisPostionAngle_TranSform(Infantry.Yaw_init)/360.0f*6.28318f;
    Theta_chassis = ChassisPostionAngle_TranSform(Infantry.Yaw_init)/360.0f*6.28318f;

	CosSP_Theta=arm_cos_f32(SP_Theta);
	SinSP_Theta=arm_sin_f32(SP_Theta);
	
/***************************ң��������********************************/
#if Robot_ID == 46  //����
	int speed = 2500;
	if(Status.ControlMode==Control_RC_Mode)
	{	
		if((-1024+RC_Ctl.rc.ch1)>300)
		{
			chassis.carSpeedx= -speed; 
		}
		else if((-1024+RC_Ctl.rc.ch1)<-300)
		{
			chassis.carSpeedx= speed; 
		}
		else
			chassis.carSpeedx= 0; 
	
	  if((-1024+RC_Ctl.rc.ch0)>300)
		{
			chassis.carSpeedy= -speed; 
		}
		else if((-1024+RC_Ctl.rc.ch0)<-300)
		{
			chassis.carSpeedy= speed; 
		}
		else
			chassis.carSpeedy= 0; 
	}
#else
	if(Status.ControlMode==Control_RC_Mode)
	  {
			
		   if((-1024+RC_Ctl.rc.ch1)>300)
			{
				chassis.carSpeedx= 1000; 
				if(SP_Theta>3.1416f/2||SP_Theta<-3.1416f/2)
				{
					chassis.carSpeedx=-1000; 
				}
			}
			else if((-1024+RC_Ctl.rc.ch1)<-300)
			{
				chassis.carSpeedx= -1000; 
				if(SP_Theta>3.1416f/2||SP_Theta<-3.1416f/2)
				{
				chassis.carSpeedx= 1000; 
				}
			}
			else
			chassis.carSpeedx= 0; 
			
			
		  if((-1024+RC_Ctl.rc.ch0)>300)
			{
				chassis.carSpeedy= 1000; 
				if(SP_Theta>3.1416f/2||SP_Theta<-3.1416f/2)
				{
				chassis.carSpeedy= -1000; 
				}
			}
			else if((-1024+RC_Ctl.rc.ch0)<-300)
			{
				chassis.carSpeedy= -1000; 
				if(SP_Theta>3.1416f/2||SP_Theta<-3.1416f/2)
				{
				chassis.carSpeedy= 1000; 
				}
			}
			else
			chassis.carSpeedy= 0; 
	}
#endif

/***************************���̿���********************************/
#if Robot_ID == 46  //����	
	if(Status.ControlMode==Control_MouseKey_Mode)  
	{
        chassis.carSpeedy = (key.a-key.d)*speed; //����
		chassis.carSpeedx = -(key.w-key.s)*speed; //����
		
		if((key.a==1&&key.s==1)||(key.a==1&&key.w==1)||(key.d==1&&key.s==1)||(key.d==1&&key.w==1))
		{
		  SelfProtect_Cross_Flag=1;
		}
		else
		{
		 	SelfProtect_Cross_Flag=0;
		}
	}
#else
	if(Status.ControlMode==Control_MouseKey_Mode)  
	{
        chassis.carSpeedx = +((key.a-key.d)*1000*SinSP_Theta-(key.s-key.w)*1000*CosSP_Theta);
        chassis.carSpeedy = -((key.s-key.w)*1000*SinSP_Theta+(key.a-key.d)*1000*CosSP_Theta);
        
		if((key.a==1&&key.s==1)||(key.a==1&&key.w==1)||(key.d==1&&key.s==1)||(key.d==1&&key.w==1))
		{
		  SelfProtect_Cross_Flag=1;
		}
		else
		{
		 	SelfProtect_Cross_Flag=0;
		}
	}
#endif
	

	chassis.carSpeedw = -8000;				//����ֱ������
	
	if(Turning_flag%2 == 1)
		chassis.carSpeedw *= -1;
}

/**********************************************************************************************************
*�� �� ��: Chassis_Solo_Cal
*����˵��: ����ģʽ��ײ�Ƕ��ű���
*��    ��: rc  key
*�� �� ֵ: ��
**********************************************************************************************************/
short SOLO_bias;
short SOLO_bias_max = 450;//550;
short H_pid = 100;
short L_pid = 20;
float H_P = 0.8f;

void Chassis_Solo_Cal(Remote rc,Key key)
{
	if(ChassisAct_Init_Flag!=Chassis_Solo_Mode)
	{
		SOLO_bias = SOLO_bias_max;
	  ChassisAct_Init_Flag=Chassis_Solo_Mode;
	}
	
	Theta=ChassisPostionAngle_TranSform(Infantry.Yaw_init)/360.0f*6.28318f;
    Theta_chassis = ChassisPostionAngle_TranSform(Infantry.Yaw_init)/360.0f*6.28318f;
	CosTheTa=cos(Theta);
	SinTheTa=sin(Theta);
	TanTheTa=tan(Theta);
	
if(Status.ControlMode==Control_RC_Mode)
  {//��Ծ����
   if((-1024+RC_Ctl.rc.ch1)>300)
			{
				chassis.carSpeedx= 2000; 
				if(Theta>3.1416f/2||Theta<-3.1416f/2)
				{
					chassis.carSpeedx= -2000; 
				}
			}
			else if((-1024+RC_Ctl.rc.ch1)<-300)
			{
				chassis.carSpeedx= -2000; 
				if(Theta>3.1416f/2||Theta<-3.1416f/2)
				{
				chassis.carSpeedx= 2000; 
				}
			}
			else
			chassis.carSpeedx= 0; 
			
			
		  if((-1024+RC_Ctl.rc.ch0)>300)
			{
				chassis.carSpeedy= 2000; 
				if(Theta>3.1416f/2||Theta<-3.1416f/2)
				{
				chassis.carSpeedy= -2000; 
				}
			}
			else if((-1024+RC_Ctl.rc.ch0)<-300)
			{
				chassis.carSpeedy= -2000; 
				if(Theta>3.1416f/2||Theta<-3.1416f/2)
				{
				chassis.carSpeedy= 2000; 
				}
			}
			else
			chassis.carSpeedy= 0; 
	}
	if(Status.ControlMode==Control_MouseKey_Mode)  
	{
	  chassis.carSpeedx = -((key.a-key.d)*2000*SinTheTa+(key.s-key.w)*2000*CosTheTa);
		chassis.carSpeedy = ((key.s-key.w)*2000*SinTheTa-(key.a-key.d)*2000*CosTheTa);
	}
	/*���̸���*/
		if(ABS(chassis.carSpeedx) <=300 && ABS(chassis.carSpeedy) < 300)		//��ֹʱŤ��
		{
			ResetPos = (ChassisPostionAngle_TranSform(Infantry.Solo_Yaw_init))/360*8192;		//�����Խǲ�ֵ����
	//		SOLO_pidChassisPosition.SetPoint = Gimbal.Yaw.Motor-ResetPos+SOLO_bias;					//����ģʽ�õ�����pid
		  if(ABS(ResetPos - SOLO_bias) < 50)
			{
				SOLO_bias = -SOLO_bias;
	  	}
			SOLO_pidChassisPosition.SetPoint = SOLO_bias;					//����ģʽ�õ�����pid
			SOLO_pidChassisPosition.ActualValue = ResetPos;
			pidChassisPosition_Speed.SetPoint = -PID_Calc(&SOLO_pidChassisPosition);
			pidChassisPosition_Speed.ActualValue = F105.ChassisSpeedw;
			chassis.carSpeedw = PID_Calc(&pidChassisPosition_Speed);

		}
		else		//��ʼ�˶���ֹͣŤ��
		{
			ResetPos = (ChassisPostionAngle_TranSform(Infantry.Solo_Yaw_init))/360*8192;		//�����Խǲ�ֵ����
			
			SOLO_pidChassisPosition.SetPoint = 0;
			SOLO_pidChassisPosition.ActualValue = ResetPos;
			pidChassisPosition_Speed.SetPoint = -PID_Calc(&SOLO_pidChassisPosition);
			
			pidChassisPosition_Speed.ActualValue = F105.ChassisSpeedw;
			chassis.carSpeedw = PID_Calc(&pidChassisPosition_Speed);

		}
	
	//�ٶȻ�
//	pidChassisPosition_Speed.SetPoint = - chassis.carSpeedw;	
//	chassis.carSpeedw = PID_Calc(&pidChassisPosition_Speed, 0);
	
//	if(ABS(chassis.carSpeedw) < 200)		//���ƾ�̬��Ư
//		chassis.carSpeedw = 0;	
//	pidChassisPosition_Speed.SetPoint = -PID_Calc(&pidChassisPosition, Gimbal.Yaw.Motor);
//	chassis.carSpeedw = LIMIT_MAX_MIN(PID_Calc(&SOLO_pidChassisPosition, -0.002f*F105.ChassisSpeedw), 400, -400); 
}

/**********************************************************************************************************
*�� �� ��: Chassis_Jump_Cal
*����˵��: �޸���ģʽ
*��    ��: rc  key
*�� �� ֵ: ��
**********************************************************************************************************/
void Chassis_Jump_Cal(Remote rc,Key key) 
{
	if(ChassisAct_Init_Flag!=Chassis_Jump_Mode)
	{
    ChassisAct_Init_Flag=Chassis_Jump_Mode;
	}
	Theta=ChassisPostionAngle_TranSform(Infantry.Yaw_init)/360.0f*6.28318f;
    Theta_chassis = ChassisPostionAngle_TranSform(Infantry.Yaw_init)/360.0f*6.28318f;
	
	CosTheTa=cos(Theta);
	SinTheTa=sin(Theta);
	TanTheTa=tan(Theta);
	
if(Status.ControlMode==Control_RC_Mode)
  {
#if Robot_ID == 5
		if((-1024+RC_Ctl.rc.ch1)>300)  //��ǰ
		{		
			if((-3.1416f/4)>Theta && (-3.1416f*3/4)<Theta)  //��Ϊͷ
			{
				chassis.carSpeedy = -2000;
			}
			else if((3.1416f/4)<Theta && (3.1416f*3/4)>Theta)  //��Ϊͷ
			{
				chassis.carSpeedy = 2000;
			}
			else if((-3.1416f*3/4)>Theta || (3.1416f*3/4)<Theta)  //��Ϊͷ
			{
				chassis.carSpeedx = 2000;
			}
			else  //ǰΪͷ
			{
				chassis.carSpeedx = -2000;
			}
		}
		else if((-1024+RC_Ctl.rc.ch1)<-300)  //���
		{
			if((-3.1416f/4)>Theta && (-3.1416f*3/4)<Theta)  //��Ϊͷ
			{
				chassis.carSpeedy = 2000;
			}
			else if((3.1416f/4)<Theta && (3.1416f*3/4)>Theta)  //��Ϊͷ
			{
				chassis.carSpeedy = -2000;
			}
			else if((-3.1416f*3/4)>Theta || (3.1416f*3/4)<Theta)  //��Ϊͷ
			{
				chassis.carSpeedx = -2000;
			}
			else  //ǰΪͷ
			{
				chassis.carSpeedx = 2000;
			}
		}
		else
		{
			if(((-3.1416f/4)>Theta && (-3.1416f*3/4)<Theta) || ((3.1416f/4)<Theta && (3.1416f*3/4)>Theta)) //�����Ϊͷ
				chassis.carSpeedy = 0;
			else  //ǰ���Ϊͷ
			  chassis.carSpeedx = 0;
		}
		
		if((-1024+RC_Ctl.rc.ch0)>300)  //����
		{
			if((-3.1416f/4)>Theta && (-3.1416f*3/4)<Theta)  //��Ϊͷ
			{
				chassis.carSpeedx = 2000;
			}
			else if((3.1416f/4)<Theta && (3.1416f*3/4)>Theta)  //��Ϊͷ
			{
				chassis.carSpeedx = -2000;
			}
			else if((-3.1416f*3/4)>Theta || (3.1416f*3/4)<Theta)  //��Ϊͷ
			{
				chassis.carSpeedy = 2000;
			}
			else  //ǰΪͷ
			{
				chassis.carSpeedy = -2000;
			}
		}
		else if((-1024+RC_Ctl.rc.ch0)<-300)  //����
		{
			if((-3.1416f/4)>Theta && (-3.1416f*3/4)<Theta)  //��Ϊͷ
			{
				chassis.carSpeedx = -2000;
			}
			else if((3.1416f/4)<Theta && (3.1416f*3/4)>Theta)  //��Ϊͷ
			{
				chassis.carSpeedx = 2000;
			}
			else if((-3.1416f*3/4)>Theta || (3.1416f*3/4)<Theta)  //��Ϊͷ
			{
				chassis.carSpeedy = -2000;
			}
			else  //ǰΪͷ
			{
				chassis.carSpeedy = 2000;
			}
		}
		else
		{
			if(((-3.1416f/4)>Theta && (-3.1416f*3/4)<Theta) || ((3.1416f/4)<Theta && (3.1416f*3/4)>Theta)) //�����Ϊͷ
				chassis.carSpeedx = 0;
			else  //ǰ���Ϊͷ
			  chassis.carSpeedy = 0;
		}
#else
     if((-1024+RC_Ctl.rc.ch1)>300)
			{
				chassis.carSpeedx= 2000; 
				if(Theta>3.1416f/2||Theta<-3.1416f/2)
				{
					chassis.carSpeedx= -2000; 
				}
			}
			else if((-1024+RC_Ctl.rc.ch1)<-300)
			{
				chassis.carSpeedx= -2000; 
				if(Theta>3.1416f/2||Theta<-3.1416f/2)
				{
				chassis.carSpeedx= 2000; 
				}
			}
			else
			chassis.carSpeedx= 0; 
			
			
		  if((-1024+RC_Ctl.rc.ch0)>300)
			{
				chassis.carSpeedy= 2000; 
				if(Theta>3.1416f/2||Theta<-3.1416f/2)
				{
				chassis.carSpeedy= -2000; 
				}
			}
			else if((-1024+RC_Ctl.rc.ch0)<-300)
			{
				chassis.carSpeedy= -2000; 
				if(Theta>3.1416f/2||Theta<-3.1416f/2)
				{
				chassis.carSpeedy= 2000; 
				}
			}
			else
			chassis.carSpeedy= 0; 
#endif
	}
	if(Status.ControlMode==Control_MouseKey_Mode)  
	{
#if Robot_ID == 5
	if(((-3.1416f/4)>Theta && (-3.1416f*3/4)<Theta) || ((3.1416f/4)<Theta && (3.1416f*3/4)>Theta)) //�����Ϊͷ
	{
		chassis.carSpeedx = + ( (key.a-key.d)*2000*SinTheTa - (key.s-key.w)*2000*CosTheTa );
		chassis.carSpeedy = - ( (key.s-key.w)*2000*SinTheTa + (key.a-key.d)*2000*CosTheTa );
	}
	else  //ǰ���Ϊͷ
	{
		chassis.carSpeedx = - ( - ( (key.a-key.d)*2000*SinTheTa + (key.s-key.w)*2000*CosTheTa ));
  	chassis.carSpeedy = - ( + ( (key.s-key.w)*2000*SinTheTa - (key.a-key.d)*2000*CosTheTa ));
		//chassis.carSpeedx = + ( (key.a-key.d)*2000*SinTheTa - (key.s-key.w)*2000*CosTheTa );
		//chassis.carSpeedy = - ( (key.s-key.w)*2000*SinTheTa + (key.a-key.d)*2000*CosTheTa );
	}
#else
	  chassis.carSpeedx = -((key.a-key.d)*2000*SinTheTa+(key.s-key.w)*2000*CosTheTa);
		chassis.carSpeedy = ((key.s-key.w)*2000*SinTheTa-(key.a-key.d)*2000*CosTheTa);
#endif	
	}
	
	//ͷ��ǰ
	ResetPos = (Theta)/6.28318f*8192;
 				/*���̸���*/
		if(0>Theta)
	{
	pidChassisPosition.SetPoint = +(Gimbal.Yaw.Motor-ResetPos-4096);
	}
	else 
	{
	pidChassisPosition.SetPoint = +(Gimbal.Yaw.Motor-ResetPos+4096);
	}
	
//����PID����+ǰ��
	pidChassisPosition.ActualValue = Gimbal.Yaw.Motor;
  chassis.carSpeedw = -PID_Calc(&pidChassisPosition) +  FeedForward_Calc(&FF_w);


}

/**********************************************************************************************************
*�� �� ��: ChassisPostionAngle_TranSform
*����˵��: �Ƕ�ת������
           �ɳ�ʼ�ǶȺ�Zero_CheckYawPosition()������-180~180�ĽǶ�
           �ù�������ֵ�ͳ�ʼ����Yaw����λ�ò��ȡ�ǶȲ�
           ���ǹ������Yaw�Ƿ���ڳ�ʼ����ֵ�����ڳ�ʼ���ĽǶ�ֵʱTheTaΪ��������Ϊ����
           ���ҹ涨TheTaֵ�ķ�ΧΪ���ʼֵ�Ľ�С�Ƕ�
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
float Bias_Angle;
float ChassisPostionAngle_TranSform(short InitPos)
{
  int32_t bias;
	bias=YawMotorReceive-InitPos;
	
	if(bias>=4096)
	bias-=8192;
	else if(bias<-4096)
	bias+=8192;
	
	Bias_Angle=bias/8192.0*360.0;
	return Bias_Angle;
}

/**********************************************************************************************************
*�� �� ��: Chassis_CurrentPid_Cal
*����˵��: ����̷���xyw���ٶ�
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Chassis_CurrentPid_Cal(void)
{
	switch (Status.ChassisMode)
	{
		case Chassis_Act_Mode:
			Chassis_Act_Cal(RC_Ctl.rc,RC_Ctl.key);
			break;

		case Chassis_SelfProtect_Mode:
			 Chassis_SelfProtect_Cal(RC_Ctl.rc,RC_Ctl.key);
			break;

		case Chassis_Solo_Mode:
			Chassis_Solo_Cal(RC_Ctl.rc,RC_Ctl.key);
			break;

		case Chassis_Jump_Mode:
			Chassis_Jump_Cal(RC_Ctl.rc,RC_Ctl.key);
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
*�� �� ��: Pid_ChassisPosition
*����˵��: ��������̨��תpid������ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Pid_ChassisPosition_Init(void)                
{
#if Robot_ID == 3
/********************************************* 3�ų� ***********************************************************/	
		pidChassisPosition.P = 5.0f;				//  λ�û�					3�ų�
	  pidChassisPosition.I = 0.00f;					
	  pidChassisPosition.D = 0.0f;				
	  pidChassisPosition.IMax = 300.0f;
	  pidChassisPosition.OutMax = 16000.0f;
	  pidChassisPosition.DeadZone=0.0f;
		pidChassisPosition.RC_DF = 0.5f;

	
		FF_w.K1 = 20000.0f;
		FF_w.K2 = 0.0f;
		FF_w.OutMax = 16000.0f;
		

#elif Robot_ID == 4	
	/********************************************* 4�ų� ***********************************************************/	
		pidChassisPosition.P = 5.0f;				//2.0  λ�û�					3�ų�
	  pidChassisPosition.I = 0.00f;					
	  pidChassisPosition.D = 0.0f;				
	  pidChassisPosition.IMax = 300.0f;
	  pidChassisPosition.OutMax = 16000.0f;
	  pidChassisPosition.DeadZone=0.0f;
		pidChassisPosition.RC_DF = 0.5f;
	
		FF_w.K1 = 20000.0f;
		FF_w.K2 = 0.0f;
		FF_w.OutMax = 16000.0f;
		
#elif Robot_ID == 14
	/********************************************* 14 �ų� ***********************************************************/	
		pidChassisPosition.P = 5.0f;				//  λ�û�					3�ų�
	  pidChassisPosition.I = 0.00f;					
	  pidChassisPosition.D = 1.0f;				
	  pidChassisPosition.IMax = 300.0f;
	  pidChassisPosition.OutMax = 16000.0f;
	  pidChassisPosition.DeadZone=0.0f;
		pidChassisPosition.RC_DF = 0.5f;
	
		FF_w.K1 = 20000.0f;
		FF_w.K2 = 0.0f;
		FF_w.OutMax = 16000.0f;
		
#elif Robot_ID == 5
	/********************************************* 5 �ų� ***********************************************************/	
		pidChassisPosition.P = 4.5f;				//  λ�û�					3�ų�
	  pidChassisPosition.I = 0.00f;					
	  pidChassisPosition.D = 1.0f;	
	  pidChassisPosition.IMax = 300.0f;
	  pidChassisPosition.OutMax = 16000.0f;
	  pidChassisPosition.DeadZone=0.0f;
		pidChassisPosition.RC_DF = 0.5f;
	
		FF_w.K1 = 15000.0f;
		FF_w.K2 = 0.0f;
		FF_w.OutMax = 16000.0f;
		
#elif Robot_ID == 44
	/********************************************* 44 �ų� ***********************************************************/	
        pidChassisPosition.P = 1.6;				//  λ�û�					44�ų�
        pidChassisPosition.I = 0.00f;					
        pidChassisPosition.D = 11.5f;	
        pidChassisPosition.IMax = 300.0f;
        pidChassisPosition.OutMax = 16000.0f;
        pidChassisPosition.DeadZone=0.0f;
        pidChassisPosition.RC_DF = 0.07f;

        FF_w.K1 = 7500.0f;
        FF_w.K2 = 30000.0f;
        FF_w.OutMax = 16000.0f;
        
        pidChassisPosition_Speed.P = 3.7f;          //�ٶȻ�
        pidChassisPosition_Speed.I = 0.0f;
        pidChassisPosition_Speed.D = 0.0f;
        pidChassisPosition_Speed.IMax = 300.0f;
        pidChassisPosition_Speed.OutMax = 30000.0f;
        pidChassisPosition_Speed.DeadZone = 100.0f;
        pidChassisPosition_Speed.RC_DF = 0.07;
 
#elif Robot_ID == 45
	/********************************************* 45 �ų� ***********************************************************/	
        pidChassisPosition.P = 1.8;				//  λ�û�					44�ų�
        pidChassisPosition.I = 0.00f;					
        pidChassisPosition.D = 12.5f;	
        pidChassisPosition.IMax = 300.0f;
        pidChassisPosition.OutMax = 16000.0f;
        pidChassisPosition.DeadZone=0.0f;
        pidChassisPosition.RC_DF = 0.07f;

        FF_w.K1 = 7000.0f;
        FF_w.K2 = 32000.0f;
        FF_w.OutMax = 16000.0f;
        
        pidChassisPosition_Speed.P = 4.0f;          //�ٶȻ�
        pidChassisPosition_Speed.I = 0.0f;
        pidChassisPosition_Speed.D = 0.0f;
        pidChassisPosition_Speed.IMax = 300.0f;
        pidChassisPosition_Speed.OutMax = 30000.0f;
        pidChassisPosition_Speed.DeadZone = 100.0f;
        pidChassisPosition_Speed.RC_DF = 0.07;
 #elif Robot_ID == 46
	/********************************************* 45 �ų� ***********************************************************/	
        pidChassisPosition.P = 0.5;				//  λ�û�					44�ų�
        pidChassisPosition.I = 0.00f;					
        pidChassisPosition.D = 0.0f;	
        pidChassisPosition.IMax = 300.0f;
        pidChassisPosition.OutMax = 16000.0f;
        pidChassisPosition.DeadZone=0.0f;
        pidChassisPosition.RC_DF = 0.07f;

        FF_w.K1 = 0.0f;
        FF_w.K2 = 0.0f;
        FF_w.OutMax = 0.0f;
        
        pidChassisPosition_Speed.P = 1.0f;          //�ٶȻ�
        pidChassisPosition_Speed.I = 0.0f;
        pidChassisPosition_Speed.D = 0.0f;
        pidChassisPosition_Speed.IMax = 300.0f;
        pidChassisPosition_Speed.OutMax = 8000.0f;
        pidChassisPosition_Speed.DeadZone = 100.0f;
        pidChassisPosition_Speed.RC_DF = 0.07;
#elif Robot_ID == 47
	/********************************************* 45 �ų� ***********************************************************/	
        pidChassisPosition.P = 1.8;				//  λ�û�					44�ų�
        pidChassisPosition.I = 0.00f;					
        pidChassisPosition.D = 12.5f;	
        pidChassisPosition.IMax = 300.0f;
        pidChassisPosition.OutMax = 16000.0f;
        pidChassisPosition.DeadZone=0.0f;
        pidChassisPosition.RC_DF = 0.07f;

        FF_w.K1 = 7000.0f;
        FF_w.K2 = 32000.0f;
        FF_w.OutMax = 16000.0f;
        
        pidChassisPosition_Speed.P = 4.0f;          //�ٶȻ�
        pidChassisPosition_Speed.I = 0.0f;
        pidChassisPosition_Speed.D = 0.0f;
        pidChassisPosition_Speed.IMax = 300.0f;
        pidChassisPosition_Speed.OutMax = 30000.0f;
        pidChassisPosition_Speed.DeadZone = 100.0f;
        pidChassisPosition_Speed.RC_DF = 0.07;
#endif 
}

/**********************************************************************************************************
*�� �� ��: Chassis_task
*����˵��: ��������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
uint32_t Chassis_high_water;
void Chassis_task(void *pvParameters)
{	
	 vTaskDelay(200);
	 while (1) {
		Chassis_CurrentPid_Cal(); 
		IWDG_Feed();
		vTaskDelay(5); 
#if INCLUDE_uxTaskGetStackHighWaterMark
        Chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}
