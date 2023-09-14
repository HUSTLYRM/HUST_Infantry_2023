/**********************************************************************************************************
 * @文件     ChassisTask.c
 * @说明     底盘控制
 * @版本  	 V1.0
 * @作者     黄志雄
 * @日期     2019.10
**********************************************************************************************************/
/**********************************************************************************************************
 * @文件     ChassisTask.c
 * @说明     底盘控制
 * @版本  	 V2.0
 * @作者     戴军
 * @日期     2022.4
     前x右y
**********************************************************************************************************/
#include "main.h"
/*----------------------------------内部变量---------------------------*/
short ChassisAct_Init_Flag=0;
float Theta,SinTheTa,CosTheTa,TanTheTa,Theta0,Speed_Theta;
float Theta_chassis;
char  SelfProtect_Cross_Flag;
float ResetPos;

const short FollowMaxSpeedw = 2000;			//跟随最高转速
const short RotateMaxSpeedw = 6000;			//小陀螺最高转速

float LastChassisSpeed = 0;
float LPF_FOLLOW = 0.05;
float LPF_FOLLOW_POS = 0.05;
/*----------------------------------结构体-----------------------------*/
ChassisSpeed_t chassis;
Pid_Typedef pidChassisPosition,pidChassisPosition_Speed;
Pid_Typedef SOLO_pidChassisPosition;
FeedForward_Typedef FF_w;
/*----------------------------------外部变量---------------------------*/
extern RC_Ctl_t RC_Ctl;
extern Status_t Status;
extern Gimbal_Typedef Gimbal;
extern RobotInit_Struct Infantry;
extern F105_Typedef F105;
extern F405_typedef F405;
extern short Turning_flag;
extern float YawMotorReceive;
extern short YawMotorSpeed;
extern uint8_t ControlMode;     //初始控制模式为陀螺仪控制
/**********************************************************************************************************
*函 数 名: Chassis_Powerdown_Cal
*功能说明: 锁车模式
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Chassis_Powerdown_Cal()
{
	if(ChassisAct_Init_Flag!=Chassis_Powerdown_Mode) 
		ChassisAct_Init_Flag=Chassis_Powerdown_Mode;
	
  chassis.carSpeedx=0;chassis.carSpeedy=0;chassis.carSpeedw=0;
  Theta_chassis = ChassisPostionAngle_TranSform(Infantry.Yaw_init)/360.0f*6.28318f;
}

/**********************************************************************************************************
*函 数 名: Chassis_Act_Cal
*功能说明: 正常模式
*形    参: rc  key
*返 回 值: 无
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
	Theta=ChassisPostionAngle_TranSform(Infantry.Yaw_init)/360.0f*6.28318f;  //弧度
    Theta_chassis = ChassisPostionAngle_TranSform(Infantry.Yaw_init)/360.0f*6.28318f;
	
	CosTheTa=cos(Theta);
	SinTheTa=sin(Theta);
	TanTheTa=tan(Theta);
	

/***************************遥控器控制********************************/
#if Robot_ID == 46  //舵轮
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
#elif Robot_ID == 5 //全向轮
	if(Status.ControlMode==Control_RC_Mode)
	{
		if((-1024+RC_Ctl.rc.ch1)>300)  //向前
		{		
			if((-3.1416f/4)>Theta && (-3.1416f*3/4)<Theta)  //左为头
			{
				chassis.carSpeedy = -2000;
			}
			else if((3.1416f/4)<Theta && (3.1416f*3/4)>Theta)  //右为头
			{
				chassis.carSpeedy = 2000;
			}
			else if((-3.1416f*3/4)>Theta || (3.1416f*3/4)<Theta)  //后为头
			{
				chassis.carSpeedx = 2000;
			}
			else  //前为头
			{
				chassis.carSpeedx = -2000;
			}
		}
		else if((-1024+RC_Ctl.rc.ch1)<-300)  //向后
		{
			if((-3.1416f/4)>Theta && (-3.1416f*3/4)<Theta)  //左为头
			{
				chassis.carSpeedy = 2000;
			}
			else if((3.1416f/4)<Theta && (3.1416f*3/4)>Theta)  //右为头
			{
				chassis.carSpeedy = -2000;
			}
			else if((-3.1416f*3/4)>Theta || (3.1416f*3/4)<Theta)  //后为头
			{
				chassis.carSpeedx = -2000;
			}
			else  //前为头
			{
				chassis.carSpeedx = 2000;
			}
		}
		else
		{
			if(((-3.1416f/4)>Theta && (-3.1416f*3/4)<Theta) || ((3.1416f/4)<Theta && (3.1416f*3/4)>Theta)) //左或右为头
				chassis.carSpeedy = 0;
			else  //前或后为头
			  chassis.carSpeedx = 0;
		}
		
		if((-1024+RC_Ctl.rc.ch0)>300)  //向右
		{
			if((-3.1416f/4)>Theta && (-3.1416f*3/4)<Theta)  //左为头
			{
				chassis.carSpeedx = 2000;
			}
			else if((3.1416f/4)<Theta && (3.1416f*3/4)>Theta)  //右为头
			{
				chassis.carSpeedx = -2000;
			}
			else if((-3.1416f*3/4)>Theta || (3.1416f*3/4)<Theta)  //后为头
			{
				chassis.carSpeedy = 2000;
			}
			else  //前为头
			{
				chassis.carSpeedy = -2000;
			}
		}
		else if((-1024+RC_Ctl.rc.ch0)<-300)  //向左
		{
			if((-3.1416f/4)>Theta && (-3.1416f*3/4)<Theta)  //左为头
			{
				chassis.carSpeedx = -2000;
			}
			else if((3.1416f/4)<Theta && (3.1416f*3/4)>Theta)  //右为头
			{
				chassis.carSpeedx = 2000;
			}
			else if((-3.1416f*3/4)>Theta || (3.1416f*3/4)<Theta)  //后为头
			{
				chassis.carSpeedy = -2000;
			}
			else  //前为头
			{
				chassis.carSpeedy = 2000;
			}
		}
		else
		{
			if(((-3.1416f/4)>Theta && (-3.1416f*3/4)<Theta) || ((3.1416f/4)<Theta && (3.1416f*3/4)>Theta)) //左或右为头
				chassis.carSpeedx = 0;
			else  //前或后为头
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
	
/***************************键盘控制********************************/
#if Robot_ID == 46  //舵轮	
	if(Status.ControlMode==Control_MouseKey_Mode)  
	{
        chassis.carSpeedy = (key.a-key.d)*speed; //后正
		chassis.carSpeedx = -(key.w-key.s)*speed; //左正
	}
#elif Robot_ID == 5
	if(Status.ControlMode==Control_MouseKey_Mode)  
	{
		if(((-3.1416f/4)>Theta && (-3.1416f*3/4)<Theta) || ((3.1416f/4)<Theta && (3.1416f*3/4)>Theta)) //左或右为头
		{
			chassis.carSpeedx = + ( (key.a-key.d)*2000*SinTheTa - (key.s-key.w)*2000*CosTheTa );
			chassis.carSpeedy = - ( (key.s-key.w)*2000*SinTheTa + (key.a-key.d)*2000*CosTheTa );
		}
		else  //前或后为头
		{
			chassis.carSpeedx = - ( - ( (key.a-key.d)*2000*SinTheTa + (key.s-key.w)*2000*CosTheTa ));
		chassis.carSpeedy = - ( + ( (key.s-key.w)*2000*SinTheTa - (key.a-key.d)*2000*CosTheTa ));
			//chassis.carSpeedx = + ( (key.a-key.d)*2000*SinTheTa - (key.s-key.w)*2000*CosTheTa );
			//chassis.carSpeedy = - ( (key.s-key.w)*2000*SinTheTa + (key.a-key.d)*2000*CosTheTa );
		}
	//	  chassis.carSpeedx = -((key.a-key.d)*2000*SinTheTa+(key.s-key.w)*2000*CosTheTa); //后正
	//		chassis.carSpeedy = ((key.s-key.w)*2000*SinTheTa-(key.a-key.d)*2000*CosTheTa); //左正
	}
#else
	if(Status.ControlMode==Control_MouseKey_Mode)  
	{
        chassis.carSpeedx = -(-(key.a-key.d)*speed*SinTheTa+(key.s-key.w)*speed*CosTheTa); //后正
		chassis.carSpeedy = (-(key.s-key.w)*speed*SinTheTa-(key.a-key.d)*speed*CosTheTa); //左正
	}

#endif	
	
	if(Status.GimbalMode == Gimbal_DropShot_Mode || Status.GimbalMode == Gimbal_BigBuf_Mode || Status.GimbalMode == Gimbal_SmlBuf_Mode)//吊射模式
	{	
		chassis.carSpeedw = 0;
	}
	else
	{
		ResetPos = (Theta)/6.28318f*8192;
		/*底盘跟随 全向轮和舵轮*/	
	#if Robot_ID == 5 || Robot_ID == 46
					
		if((-3.1416f/4)>Theta && (-3.1416f*3/4)<Theta)  //右为头
		{
		pidChassisPosition.SetPoint = Gimbal.Yaw.Motor-ResetPos-2048;
		}
		else if((3.1416f/4)<Theta && (3.1416f*3/4)>Theta)  //左为头
		{
		pidChassisPosition.SetPoint = Gimbal.Yaw.Motor-ResetPos+2048;
		}
		else if((-3.1416f*3/4)>Theta)  //后
		{
		pidChassisPosition.SetPoint = Gimbal.Yaw.Motor-ResetPos-4096;
		}
		else if((3.1416f*3/4)<Theta)  //后
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
			//反馈PID控制+前馈
			pidChassisPosition.ActualValue =(LPF_FOLLOW_POS)*pidChassisPosition.ActualValue + (1-LPF_FOLLOW_POS)*Gimbal.Yaw.Motor;
			//    chassis.carSpeedw = -(-PID_Calc(&pidChassisPosition) +  FeedForward_Calc(&FF_w));
			//新加速度环
			pidChassisPosition_Speed.SetPoint = -(-PID_Calc(&pidChassisPosition) );
			pidChassisPosition_Speed.ActualValue = Gimbal.Yaw.AngularSpeed - YawMotorSpeed*360.0f/60.0f /*此处将rpm转为角度每秒*/;
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
*函 数 名: Chassis_SelfProtect_Cal
*功能说明: 保护模式
*形    参: rc  key
*返 回 值: 无
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
	
/***************************遥控器控制********************************/
#if Robot_ID == 46  //舵轮
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

/***************************键盘控制********************************/
#if Robot_ID == 46  //舵轮	
	if(Status.ControlMode==Control_MouseKey_Mode)  
	{
        chassis.carSpeedy = (key.a-key.d)*speed; //后正
		chassis.carSpeedx = -(key.w-key.s)*speed; //左正
		
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
	

	chassis.carSpeedw = -8000;				//底盘直接设置
	
	if(Turning_flag%2 == 1)
		chassis.carSpeedw *= -1;
}

/**********************************************************************************************************
*函 数 名: Chassis_Solo_Cal
*功能说明: 单挑模式，撞角对着别人
*形    参: rc  key
*返 回 值: 无
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
  {//阶跃控制
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
	/*底盘跟随*/
		if(ABS(chassis.carSpeedx) <=300 && ABS(chassis.carSpeedy) < 300)		//静止时扭腰
		{
			ResetPos = (ChassisPostionAngle_TranSform(Infantry.Solo_Yaw_init))/360*8192;		//与正对角差值计算
	//		SOLO_pidChassisPosition.SetPoint = Gimbal.Yaw.Motor-ResetPos+SOLO_bias;					//单挑模式用单独的pid
		  if(ABS(ResetPos - SOLO_bias) < 50)
			{
				SOLO_bias = -SOLO_bias;
	  	}
			SOLO_pidChassisPosition.SetPoint = SOLO_bias;					//单挑模式用单独的pid
			SOLO_pidChassisPosition.ActualValue = ResetPos;
			pidChassisPosition_Speed.SetPoint = -PID_Calc(&SOLO_pidChassisPosition);
			pidChassisPosition_Speed.ActualValue = F105.ChassisSpeedw;
			chassis.carSpeedw = PID_Calc(&pidChassisPosition_Speed);

		}
		else		//开始运动，停止扭腰
		{
			ResetPos = (ChassisPostionAngle_TranSform(Infantry.Solo_Yaw_init))/360*8192;		//与正对角差值计算
			
			SOLO_pidChassisPosition.SetPoint = 0;
			SOLO_pidChassisPosition.ActualValue = ResetPos;
			pidChassisPosition_Speed.SetPoint = -PID_Calc(&SOLO_pidChassisPosition);
			
			pidChassisPosition_Speed.ActualValue = F105.ChassisSpeedw;
			chassis.carSpeedw = PID_Calc(&pidChassisPosition_Speed);

		}
	
	//速度环
//	pidChassisPosition_Speed.SetPoint = - chassis.carSpeedw;	
//	chassis.carSpeedw = PID_Calc(&pidChassisPosition_Speed, 0);
	
//	if(ABS(chassis.carSpeedw) < 200)		//抑制静态零漂
//		chassis.carSpeedw = 0;	
//	pidChassisPosition_Speed.SetPoint = -PID_Calc(&pidChassisPosition, Gimbal.Yaw.Motor);
//	chassis.carSpeedw = LIMIT_MAX_MIN(PID_Calc(&SOLO_pidChassisPosition, -0.002f*F105.ChassisSpeedw), 400, -400); 
}

/**********************************************************************************************************
*函 数 名: Chassis_Jump_Cal
*功能说明: 无跟随模式
*形    参: rc  key
*返 回 值: 无
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
		if((-1024+RC_Ctl.rc.ch1)>300)  //向前
		{		
			if((-3.1416f/4)>Theta && (-3.1416f*3/4)<Theta)  //左为头
			{
				chassis.carSpeedy = -2000;
			}
			else if((3.1416f/4)<Theta && (3.1416f*3/4)>Theta)  //右为头
			{
				chassis.carSpeedy = 2000;
			}
			else if((-3.1416f*3/4)>Theta || (3.1416f*3/4)<Theta)  //后为头
			{
				chassis.carSpeedx = 2000;
			}
			else  //前为头
			{
				chassis.carSpeedx = -2000;
			}
		}
		else if((-1024+RC_Ctl.rc.ch1)<-300)  //向后
		{
			if((-3.1416f/4)>Theta && (-3.1416f*3/4)<Theta)  //左为头
			{
				chassis.carSpeedy = 2000;
			}
			else if((3.1416f/4)<Theta && (3.1416f*3/4)>Theta)  //右为头
			{
				chassis.carSpeedy = -2000;
			}
			else if((-3.1416f*3/4)>Theta || (3.1416f*3/4)<Theta)  //后为头
			{
				chassis.carSpeedx = -2000;
			}
			else  //前为头
			{
				chassis.carSpeedx = 2000;
			}
		}
		else
		{
			if(((-3.1416f/4)>Theta && (-3.1416f*3/4)<Theta) || ((3.1416f/4)<Theta && (3.1416f*3/4)>Theta)) //左或右为头
				chassis.carSpeedy = 0;
			else  //前或后为头
			  chassis.carSpeedx = 0;
		}
		
		if((-1024+RC_Ctl.rc.ch0)>300)  //向右
		{
			if((-3.1416f/4)>Theta && (-3.1416f*3/4)<Theta)  //左为头
			{
				chassis.carSpeedx = 2000;
			}
			else if((3.1416f/4)<Theta && (3.1416f*3/4)>Theta)  //右为头
			{
				chassis.carSpeedx = -2000;
			}
			else if((-3.1416f*3/4)>Theta || (3.1416f*3/4)<Theta)  //后为头
			{
				chassis.carSpeedy = 2000;
			}
			else  //前为头
			{
				chassis.carSpeedy = -2000;
			}
		}
		else if((-1024+RC_Ctl.rc.ch0)<-300)  //向左
		{
			if((-3.1416f/4)>Theta && (-3.1416f*3/4)<Theta)  //左为头
			{
				chassis.carSpeedx = -2000;
			}
			else if((3.1416f/4)<Theta && (3.1416f*3/4)>Theta)  //右为头
			{
				chassis.carSpeedx = 2000;
			}
			else if((-3.1416f*3/4)>Theta || (3.1416f*3/4)<Theta)  //后为头
			{
				chassis.carSpeedy = -2000;
			}
			else  //前为头
			{
				chassis.carSpeedy = 2000;
			}
		}
		else
		{
			if(((-3.1416f/4)>Theta && (-3.1416f*3/4)<Theta) || ((3.1416f/4)<Theta && (3.1416f*3/4)>Theta)) //左或右为头
				chassis.carSpeedx = 0;
			else  //前或后为头
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
	if(((-3.1416f/4)>Theta && (-3.1416f*3/4)<Theta) || ((3.1416f/4)<Theta && (3.1416f*3/4)>Theta)) //左或右为头
	{
		chassis.carSpeedx = + ( (key.a-key.d)*2000*SinTheTa - (key.s-key.w)*2000*CosTheTa );
		chassis.carSpeedy = - ( (key.s-key.w)*2000*SinTheTa + (key.a-key.d)*2000*CosTheTa );
	}
	else  //前或后为头
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
	
	//头朝前
	ResetPos = (Theta)/6.28318f*8192;
 				/*底盘跟随*/
		if(0>Theta)
	{
	pidChassisPosition.SetPoint = +(Gimbal.Yaw.Motor-ResetPos-4096);
	}
	else 
	{
	pidChassisPosition.SetPoint = +(Gimbal.Yaw.Motor-ResetPos+4096);
	}
	
//反馈PID控制+前馈
	pidChassisPosition.ActualValue = Gimbal.Yaw.Motor;
  chassis.carSpeedw = -PID_Calc(&pidChassisPosition) +  FeedForward_Calc(&FF_w);


}

/**********************************************************************************************************
*函 数 名: ChassisPostionAngle_TranSform
*功能说明: 角度转换函数
           由初始角度和Zero_CheckYawPosition()来计算-180~180的角度
           用过零检测后的值和初始化的Yaw来做位置差获取角度差
           考虑过零检测的Yaw是否大于初始化的值，大于初始化的角度值时TheTa为正，否则为负；
           并且规定TheTa值的范围为与初始值的较小角度
*形    参: 无
*返 回 值: 无
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
*函 数 名: Chassis_CurrentPid_Cal
*功能说明: 向底盘发送xyw向速度
*形    参: 无
*返 回 值: 无
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
*函 数 名: Pid_ChassisPosition
*功能说明: 底盘随云台旋转pid参数初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Pid_ChassisPosition_Init(void)                
{
#if Robot_ID == 3
/********************************************* 3号车 ***********************************************************/	
		pidChassisPosition.P = 5.0f;				//  位置环					3号车
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
	/********************************************* 4号车 ***********************************************************/	
		pidChassisPosition.P = 5.0f;				//2.0  位置环					3号车
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
	/********************************************* 14 号车 ***********************************************************/	
		pidChassisPosition.P = 5.0f;				//  位置环					3号车
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
	/********************************************* 5 号车 ***********************************************************/	
		pidChassisPosition.P = 4.5f;				//  位置环					3号车
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
	/********************************************* 44 号车 ***********************************************************/	
        pidChassisPosition.P = 1.6;				//  位置环					44号车
        pidChassisPosition.I = 0.00f;					
        pidChassisPosition.D = 11.5f;	
        pidChassisPosition.IMax = 300.0f;
        pidChassisPosition.OutMax = 16000.0f;
        pidChassisPosition.DeadZone=0.0f;
        pidChassisPosition.RC_DF = 0.07f;

        FF_w.K1 = 7500.0f;
        FF_w.K2 = 30000.0f;
        FF_w.OutMax = 16000.0f;
        
        pidChassisPosition_Speed.P = 3.7f;          //速度环
        pidChassisPosition_Speed.I = 0.0f;
        pidChassisPosition_Speed.D = 0.0f;
        pidChassisPosition_Speed.IMax = 300.0f;
        pidChassisPosition_Speed.OutMax = 30000.0f;
        pidChassisPosition_Speed.DeadZone = 100.0f;
        pidChassisPosition_Speed.RC_DF = 0.07;
 
#elif Robot_ID == 45
	/********************************************* 45 号车 ***********************************************************/	
        pidChassisPosition.P = 1.8;				//  位置环					44号车
        pidChassisPosition.I = 0.00f;					
        pidChassisPosition.D = 12.5f;	
        pidChassisPosition.IMax = 300.0f;
        pidChassisPosition.OutMax = 16000.0f;
        pidChassisPosition.DeadZone=0.0f;
        pidChassisPosition.RC_DF = 0.07f;

        FF_w.K1 = 7000.0f;
        FF_w.K2 = 32000.0f;
        FF_w.OutMax = 16000.0f;
        
        pidChassisPosition_Speed.P = 4.0f;          //速度环
        pidChassisPosition_Speed.I = 0.0f;
        pidChassisPosition_Speed.D = 0.0f;
        pidChassisPosition_Speed.IMax = 300.0f;
        pidChassisPosition_Speed.OutMax = 30000.0f;
        pidChassisPosition_Speed.DeadZone = 100.0f;
        pidChassisPosition_Speed.RC_DF = 0.07;
 #elif Robot_ID == 46
	/********************************************* 45 号车 ***********************************************************/	
        pidChassisPosition.P = 0.5;				//  位置环					44号车
        pidChassisPosition.I = 0.00f;					
        pidChassisPosition.D = 0.0f;	
        pidChassisPosition.IMax = 300.0f;
        pidChassisPosition.OutMax = 16000.0f;
        pidChassisPosition.DeadZone=0.0f;
        pidChassisPosition.RC_DF = 0.07f;

        FF_w.K1 = 0.0f;
        FF_w.K2 = 0.0f;
        FF_w.OutMax = 0.0f;
        
        pidChassisPosition_Speed.P = 1.0f;          //速度环
        pidChassisPosition_Speed.I = 0.0f;
        pidChassisPosition_Speed.D = 0.0f;
        pidChassisPosition_Speed.IMax = 300.0f;
        pidChassisPosition_Speed.OutMax = 8000.0f;
        pidChassisPosition_Speed.DeadZone = 100.0f;
        pidChassisPosition_Speed.RC_DF = 0.07;
#elif Robot_ID == 47
	/********************************************* 45 号车 ***********************************************************/	
        pidChassisPosition.P = 1.8;				//  位置环					44号车
        pidChassisPosition.I = 0.00f;					
        pidChassisPosition.D = 12.5f;	
        pidChassisPosition.IMax = 300.0f;
        pidChassisPosition.OutMax = 16000.0f;
        pidChassisPosition.DeadZone=0.0f;
        pidChassisPosition.RC_DF = 0.07f;

        FF_w.K1 = 7000.0f;
        FF_w.K2 = 32000.0f;
        FF_w.OutMax = 16000.0f;
        
        pidChassisPosition_Speed.P = 4.0f;          //速度环
        pidChassisPosition_Speed.I = 0.0f;
        pidChassisPosition_Speed.D = 0.0f;
        pidChassisPosition_Speed.IMax = 300.0f;
        pidChassisPosition_Speed.OutMax = 30000.0f;
        pidChassisPosition_Speed.DeadZone = 100.0f;
        pidChassisPosition_Speed.RC_DF = 0.07;
#endif 
}

/**********************************************************************************************************
*函 数 名: Chassis_task
*功能说明: 底盘任务
*形    参: 无
*返 回 值: 无
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
