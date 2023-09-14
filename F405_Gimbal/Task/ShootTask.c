/**********************************************************************************************************
 * @文件     ShootTask.c
 * @说明     发弹控制
 * @版本  	 V1.0
 * @作者     黄志雄
 * @日期     2020.1
**********************************************************************************************************/
/**********************************************************************************************************
 * @文件     ShootTask.c
 * @说明     发弹控制
 * @版本  	 V2.0
 * @作者     戴军
 * @日期     2022.1
**********************************************************************************************************/
/**********************************************************************************************************
 * @文件     ShootTask.c
 * @说明     增加了摩擦轮掉速计数估计射击子弹数目，单独打弹检查ShootCount_Number是否正确
 * @版本  	 V3.0
 * @作者     郭嘉豪
 * @日期     2023.6
**********************************************************************************************************/
# include "main.h"
/*----------------------------------内部变量---------------------------*/
float MirocPosition;
short FrictionWheel_speed=0,BulletSpeed,BodanMotorCurrent,ShootAct_Init_Flag;
short BodanMotorCurrent,FrictionCurrent[2];
//int SendToTx2BullectCnt;
int Shoot_Init_flag = 0;
int ShootCount = 0;
float Onegrid; 
/*----------------------------------结构体------------------------------*/
QUEUE_HandleTypeDef qFrictionSpeed;
QUEUE_DATA_T BufferFrictionSpeedQueen[QueenSampling_number];
Pid_Typedef PidBodanMotorPos,PidBodanMotorSpeed,PidFrictionSpeed[2];
FuzzyPID FuzzyBodanMotorPos;
ShootTask_typedef Shoot;
/*----------------------------------外部变量---------------------------*/
extern RC_Ctl_t RC_Ctl;
extern float Bodan_Pos;
extern F105_Typedef F105;//F105
extern F405_typedef F405;
extern Status_t Status;
extern Gimbal_Typedef Gimbal;
extern PC_Receive_t PC_Receive;
extern BodanMotorReceive_Typedef BodanReceive;
//extern short armor_state;
extern RobotInit_Struct Infantry;
extern short FrictionReceive[2];
extern uint8_t CoolBuffState;

short PullerSpeed ;        //  1000 冷却1级射频4.5     2000 2级6.5  3000 12.5
short checkPullerSpeed;             //  800  4
/**********************************************************************************************************
*函 数 名: 采样摩擦轮转速估计设计子弹数
*功能说明: 
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
uint32_t ShootCountTime_last=0;
float ShootCount_dt=0;
int ShootCount_Number = 0;//射击的总子弹数
void ShootCount_Cal(void)
{
	static short ShootMode_last = 0;
	ShootCount_dt = GetDeltaT(&ShootCountTime_last);
	Shoot.HeatControl.ShootCount_IntervalTime+=ShootCount_dt*1000;
	
	if(ShootMode_last!=Status.ShootMode){
		Shoot.HeatControl.ShootCount_IntervalTime = -200;//切换摩擦轮模式后设置一段时间保护，防止被认为射击
	}
	if(ABS(PidFrictionSpeed[0].PreError)>200 && ABS(PidFrictionSpeed[0].PreError) < 1000.0 &&ABS(PidFrictionSpeed[0].ActualValue)>3000)//满足掉速要求
	{
		if(Shoot.HeatControl.ShootCount_IntervalTime>ShootInterval_Min_time){//连射的最小间隔
			ShootCount_Number++;//总射击子弹数
			Shoot.HeatControl.CurShootNumber++;
			Shoot.HeatControl.ShootCount_IntervalTime = 0;
		}
	}
	ShootMode_last = Status.ShootMode;
}
/**********************************************************************************************************
*函 数 名: PluckSpeedChoose
*功能说明: 拨盘拨速选择
*形    参: flag
*返 回 值: 无
**********************************************************************************************************/
extern char HighFreq_flag;
void Pluck_Speed_Choose()
{
	if(HighFreq_flag)
	{
		PullerSpeed = 4500;
	}
	else
	{
  	switch(F105.JudgeReceive_info.RobotLevel)
  {	
       /***** 1级 ******/
		case 1:
			if(!CoolBuffState)
			{
     	 PullerSpeed = 3500;    //无增益
			}
			else
			{
			 PullerSpeed = 4000;    //有增益 
			}
		break;
		
		 /****** 2级 *****/
		case 2:
		if(!CoolBuffState)
		{
     	 PullerSpeed = 4000;    //无增益
		}
		else
		{
			 PullerSpeed = 4500;    //有增益 
		}
		break;
		
    /******  3级 ******/		
		case 3:
			if(!CoolBuffState)
		{
     	 PullerSpeed = 4000;    //无增益
		}
		else
		{
			 PullerSpeed = 4500;    //有增益 
		}
		break;
		
		
		default:
		PullerSpeed = 1000;
	 }	
  }
}
/**********************************************************************************************************
*函 数 名: FrictionSpeedChoose
*功能说明: 摩擦轮转速选择
*形    参: flag
*返 回 值: 无
**********************************************************************************************************/
void FrictionSpeedChoose(void)
{
	switch(F105.JudgeReceive_info.BulletSpeedLevel)
	{
		case 0:
		{
			FrictionWheel_speed = Infantry.Low_FrictionSpeed;
			break;
		}
		case 1:
		{
				FrictionWheel_speed = Infantry.Medium_FrictionSpeed;
			break;
		}
		case 2:
		{
				FrictionWheel_speed = Infantry.High_FrictionSpeed;
			break;
		}
		default:
		{
			FrictionWheel_speed = Infantry.High_FrictionSpeed;
			break;
		}
	}
}


/**********************************************************************************************************
*函 数 名: HeatControl
*功能说明: 热量更新
*形    参: 处理间隔s
*返 回 值: 无
**********************************************************************************************************/
const float BulletHeat17 = 10;
float HeatControlThreshold = 0.95f;   	//开启热量控制的阈值
float curHeat=0;
void HeatControl(float dt)
{
	Shoot.HeatControl.HeatMax17 = F105.JudgeReceive_info.HeatMax17 - BulletHeat17;  		//判断能否打蛋的最大热量，保留一个子弹
	Shoot.HeatControl.HeatCool17 = F105.JudgeReceive_info.HeatCool17 * dt;          		// 当前热量检测的冷却值
	
	/****************************更新当前热量**************************************/
	if(Shoot.HeatControl.HeatUpdateFlag == 1){												//热量更新了则使用裁判系统
		Shoot.HeatControl.HeatUpdateFlag = 0;
		Shoot.HeatControl.CurHeat17 = F105.JudgeReceive_info.shooterHeat17;
	}else{																														//自行估计热量
		Shoot.HeatControl.CurHeat17 += Shoot.HeatControl.CurShootNumber*BulletHeat17;
		Shoot.HeatControl.CurShootNumber = 0;														//当前子弹处理完毕
		Shoot.HeatControl.CurHeat17 = Shoot.HeatControl.CurHeat17 - Shoot.HeatControl.HeatCool17;															//处理当前周期的冷却
		if(Shoot.HeatControl.CurHeat17<0) Shoot.HeatControl.CurHeat17 = 0;
	}
	
	if(Shoot.HeatControl.CurHeat17 < HeatControlThreshold*Shoot.HeatControl.HeatMax17)
	{
		Shoot.HeatControl.IsShootAble = 1;
	}
	else
	{
		Shoot.HeatControl.IsShootAble = 0;
	}
	Shoot.HeatControl.LastHeat17 = Shoot.HeatControl.CurHeat17;
	curHeat = Shoot.HeatControl.CurHeat17;
}


/**********************************************************************************************************
*函 数 名: Shoot_Fire_Cal
*功能说明: 正常模式
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
char Reverse_Flag=0;
void Shoot_Fire_Cal()
{
	if(ShootAct_Init_Flag!=1)
	{
		ShootAct_Init_Flag=1;
	}
	static int ShootContinue_init_flag=0;

	if(Shoot.HeatControl.IsShootAble || HighFreq_flag)
	{

			if(Shoot.ShootContinue)
			{
				PidBodanMotorSpeed.SetPoint=-Infantry.BodanMotor_pn*PullerSpeed;
			}
			else
			{
				ShootContinue_init_flag = 0;
				 //控制拨盘反转，防卡弹
				if(Shoot.ReverseRotation)
				{
						if(Reverse_Flag==0)
						{
							PidBodanMotorPos.SetPoint=PidBodanMotorPos.SetPoint + Infantry.BodanMotor_pn*Onegrid;
							Reverse_Flag=1;
						}
						if(ABS(Bodan_Pos-PidBodanMotorPos.SetPoint) < PluckThreholdPos)
						{
							Shoot.ReverseRotation=0;
							Reverse_Flag=0;
						}
				
				}
				else
				{

						if(ABS(Bodan_Pos-PidBodanMotorPos.SetPoint) < PluckThreholdPos)
						{
							PidBodanMotorPos.SetPoint=PidBodanMotorPos.SetPoint+Infantry.BodanMotor_pn*MirocPosition; 
							if(MirocPosition != 0)
							{
								ShootCount++;
							}
							
						}
			
				}
			}
		
	}
	else
	{
		PidBodanMotorSpeed.SetPoint=0;	
	}
	MirocPosition = 0;
}

/**********************************************************************************************************
*函 数 名: Shoot_Check_Cal
*功能说明: 检录模式[打开激光，摩擦轮/射频3~4发]
*形    参: rc
*返 回 值: 无
**********************************************************************************************************/
int delay_time =-1000;
void Shoot_Check_Cal()
{ 
	delay_time++;
	if(ShootAct_Init_Flag!=2)
	ShootAct_Init_Flag=2;
	MirocPosition = 0;
	 if(delay_time>0)
	 {
		if(Shoot.HeatControl.IsShootAble==1)
		{	
//		PidBodanMotorPos.SetPoint=PidBodanMotorPos.SetPoint+500; 
//		PidBodanMotorSpeed.SetPoint = PID_Calc(&PidBodanMotorPos,Bodan_Pos);
			PidBodanMotorSpeed.SetPoint=-Infantry.BodanMotor_pn*checkPullerSpeed;
			delay_time=0;	
		}
		else
		{
				PidBodanMotorSpeed.SetPoint=0;	
		}
	}
}

/**********************************************************************************************************
*函 数 名: Shoot_Test_Cal
*功能说明: 测试模式，用于辅瞄上位机测试
*形    参: 
*返 回 值: 无
**********************************************************************************************************/
extern char Aim_Follow;
extern PCRecvData pc_recv_data;
TickType_t EnterAimTick,nowAimTick;
void Shoot_Test_Cal()
{ 
  delay_time++;
	if(ShootAct_Init_Flag!=0xcf)
	{
	ShootAct_Init_Flag=0xcf;
	EnterAimTick = xTaskGetTickCount();
	}
	nowAimTick = xTaskGetTickCount();
	if(delay_time>0)
	{ 
//		if(((nowAimTick-EnterAimTick)>200) && Shoot.HeatControl.IsShootAble==1 && armor_state == ARMOR_AIMED && Aim_Follow==1)
		if(((nowAimTick-EnterAimTick)>200) && Shoot.HeatControl.IsShootAble==1 && pc_recv_data.shoot_flag)		
		{
			if(ABS(Bodan_Pos-PidBodanMotorPos.SetPoint) < PluckThreholdPos)
			{
				PidBodanMotorPos.SetPoint = PidBodanMotorPos.SetPoint+Onegrid; 
//				armor_state=ARMOR_NO_AIM;			
			}
		}
	}	
	
}

/**********************************************************************************************************
*函 数 名: Shoot_Powerdown_Cal
*功能说明: 锁定模式[微动开关/摩擦轮/拨弹转速]
*形    参: rc
*返 回 值: 无
**********************************************************************************************************/
void Shoot_Powerdown_Cal(void)
{
	if(ShootAct_Init_Flag!=4)
	  ShootAct_Init_Flag=4;
	
	PidBodanMotorPos.SetPoint  = Bodan_Pos;
	Shoot.ReverseRotation = 0;
	Reverse_Flag = 0;
	Shoot.ShootContinue = 0;
}

/**********************************************************************************************************
*函 数 名: Shoot_Tx2_Cal
*功能说明: 辅瞄模式
*形    参: rc
*返 回 值: 无
**********************************************************************************************************/
extern PCRecvData pc_recv_data;
extern float k_onegrid;
extern float pc_yaw,pc_pitch;
extern uint8_t PC_Shoot_flag;
uint8_t shoot_test_flag = 1;
uint32_t shoot_dt = 0;
void Shoot_Tx2_Cal()
{
	if(ShootAct_Init_Flag!=3)
	{
		MirocPosition = 0;
		ShootAct_Init_Flag=3;
//		SendToTx2BullectCnt=PC_Receive.ReceiveFromTx2BullectCnt=0;
	}
    shoot_dt++;
/***********************************************Armor******************************************/	
	if(Status.GimbalMode == Gimbal_Armor_Mode)
	{
		if(Shoot.HeatControl.IsShootAble==1 && PC_Shoot_flag/*shoot_test_flag*/ != 0)
  	{
			if(ABS(pc_pitch - Gimbal.Pitch.Gyro)<1.5f && ABS(pc_yaw - Gimbal.Yaw.Gyro)<1.5f)	//已经辅瞄到位，自动开火
			{
				if(ABS(PidBodanMotorPos.SetPoint-Bodan_Pos)<5000)
				{
//                    if(shoot_dt % 100 == 1)
					MirocPosition = -k_onegrid * Onegrid * 0.8;         //控制频率很快，需要放慢拨弹速度
//					SendToTx2BullectCnt++;
				}
			}
    }
	}
	
	if(Status.GimbalMode == Gimbal_BigBuf_Mode || Status.GimbalMode == Gimbal_SmlBuf_Mode)
	{
		if(Shoot.HeatControl.IsShootAble==1 && PC_Shoot_flag/*shoot_test_flag*/ != 0)
        {
			if(ABS(pc_pitch - Gimbal.Pitch.Gyro)<0.8f && ABS(pc_yaw - Gimbal.Yaw.Gyro)<0.8f)	//已经辅瞄到位，自动开火
			{
				if(ABS(PidBodanMotorPos.SetPoint-Bodan_Pos)<8000)
				{
//                    if(shoot_dt % 200 == 1)
					MirocPosition = -k_onegrid * Onegrid * 0.1;
//					SendToTx2BullectCnt++;
				}
			}
        }
	}	
	
        if(Shoot.HeatControl.IsShootAble==1)
		PidBodanMotorPos.SetPoint=PidBodanMotorPos.SetPoint+MirocPosition;  
		MirocPosition = 0;
}


/**********************************************************************************************************
*函 数 名: FrictionWheel_CurrentPid_Cal
*功能说明: 设置摩擦轮PID速度
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void FrictionWheel_CurrentPid_Cal()
{
	if(Status.ShootMode!=Shoot_Check_Mode)
	{
		delay_time = 0;
	}

	PidFrictionSpeed[0].SetPoint=Infantry.FricMotor_pn[0]*FrictionWheel_speed;     //  accelerator+bias_speed;
	PidFrictionSpeed[1].SetPoint=-Infantry.FricMotor_pn[1]*FrictionWheel_speed;
	
	
	PidFrictionSpeed[0].ActualValue = FrictionReceive[0];
	FrictionCurrent[0]=PID_Calc(&PidFrictionSpeed[0]);
	PidFrictionSpeed[1].ActualValue = FrictionReceive[1];
	FrictionCurrent[1]=PID_Calc(&PidFrictionSpeed[1]);
}

/**********************************************************************************************************
*函 数 名: BodanPos_Set
*功能说明: 拨弹电机位置期望设置
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void BodanMotorPos_Set(){
	switch(Status.ShootMode)//射击模式选择
	{
		case Shoot_Check_Mode:
			Shoot_Check_Cal();
			break;

		case Shoot_Fire_Mode:
			Shoot_Fire_Cal();
			break;

		case Shoot_Tx2_Mode:
//			Shoot_Test_Cal();			//测试
			Shoot_Tx2_Cal();
			break;

		case Shoot_Powerdown_Mode:
			Shoot_Powerdown_Cal();
			break;

		default:
			break;
	}
}
/**********************************************************************************************************
*函 数 名: BodanMotor_CurrentPid_Cal
*功能说明: 拨弹电机位置环双环计算输出
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void BodanMotor_CurrentPid_Cal(void)
{
		switch(Status.ShootMode)//射击模式选择
	{
		case Shoot_Check_Mode:
			//不需要位置环
		break;
		
		case Shoot_Fire_Mode:
			if(Shoot.ShootContinue){//连射模式不需要控制位置
				break;
			}
		case Shoot_Tx2_Mode:
		case Shoot_Powerdown_Mode:
				PidBodanMotorPos.ActualValue = Bodan_Pos;
				PidBodanMotorSpeed.SetPoint = PID_Calc(&PidBodanMotorPos);	
			break;

		default:
			break;
	}
	
	PidBodanMotorSpeed.ActualValue = BodanReceive.RealSpeed;
	BodanMotorCurrent = (short)PID_Calc(&PidBodanMotorSpeed);
	if(Status.ShootMode==Shoot_Powerdown_Mode)
	{
		BodanMotorCurrent =0.0f;
	}
}

/**********************************************************************************************************
*函 数 名: Pid_BodanMotor
*功能说明: 拨弹电机位置速度环pid参数初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Pid_BodanMotor_Init(void)
{
	
	PidBodanMotorPos.P=0.4f;  //0.4f
	PidBodanMotorPos.I=0.02f;
	PidBodanMotorPos.D=0.0f;
	PidBodanMotorPos.IMax=1500.0f;
	PidBodanMotorPos.SetPoint=0.0f;
	PidBodanMotorPos.OutMax=20000.0f;
	PidBodanMotorPos.RC_DF = 0.5F;
	PidBodanMotorPos.I_L = 8000;
	PidBodanMotorPos.I_U = 12000;
	
	
#if (Robot_ID == 3 || Robot_ID == 4 || Robot_ID == 14)|| (Robot_ID==46)

    Onegrid=36864.0f;		  //老拨盘
	checkPullerSpeed = 500;
	
	PidBodanMotorSpeed.P=6.2f;  //5.0f
	PidBodanMotorSpeed.I=3.2f;//0.01f;
	PidBodanMotorSpeed.D=0.0f;
	PidBodanMotorSpeed.DeadZone=50.0f;
	PidBodanMotorSpeed.IMax=1000.0f;
	PidBodanMotorSpeed.SetPoint=0.0f;
	PidBodanMotorSpeed.OutMax = 10000.0f;
	PidBodanMotorSpeed.I_L = 100;
	PidBodanMotorSpeed.I_U = 200;
	PidBodanMotorSpeed.RC_DF = 0.5F;
		
#elif (Robot_ID==44)

	Onegrid=36864.0f;
	checkPullerSpeed = 500;
		
	PidBodanMotorSpeed.P=5.0f;  //5.0f
	PidBodanMotorSpeed.I=2.0f;//0.01f;
	PidBodanMotorSpeed.D=0.0f;
	PidBodanMotorSpeed.DeadZone=50.0f;
	PidBodanMotorSpeed.IMax=1000.0f;
	PidBodanMotorSpeed.SetPoint=0.0f;
	PidBodanMotorSpeed.OutMax = 10000.0f;
	PidBodanMotorSpeed.I_L = 100;
	PidBodanMotorSpeed.I_U = 200;
	PidBodanMotorSpeed.RC_DF = 0.5F;
	
#elif (Robot_ID==45 || Robot_ID == 47)

	Onegrid=36864.0f;
	checkPullerSpeed = 500;
	
	PidBodanMotorSpeed.P=6.0f;  //5.0f
	PidBodanMotorSpeed.I=3.0f;//0.01f;
	PidBodanMotorSpeed.D=0.0f;
	PidBodanMotorSpeed.DeadZone=50.0f;
	PidBodanMotorSpeed.IMax=1000.0f;
	PidBodanMotorSpeed.SetPoint=0.0f;
	PidBodanMotorSpeed.OutMax = 10000.0f;
	PidBodanMotorSpeed.I_L = 100;
	PidBodanMotorSpeed.I_U = 200;
	PidBodanMotorSpeed.RC_DF = 0.5F;
#else
	
	  Onegrid=36864.0f;		  //新拨盘
		checkPullerSpeed = 1000;
#endif
}

/**********************************************************************************************************
*函 数 名: Pid_Friction_Init
*功能说明: 拨弹电机位置速度环pid参数初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Pid_Friction_Init(void)
{
	PidFrictionSpeed[0].P=10.0f;//80.0f;
	PidFrictionSpeed[0].I=0.0f;
	PidFrictionSpeed[0].D=0.0f;
	PidFrictionSpeed[0].IMax=1500.0f;
	PidFrictionSpeed[0].SetPoint=0.0f;
	PidFrictionSpeed[0].OutMax = 9900.0f;
	PidFrictionSpeed[0].RC_DF = 0.05F;
	PidFrictionSpeed[0].I_L = 50;
	PidFrictionSpeed[0].I_U =100;
	
	
	PidFrictionSpeed[1].P=10.0f;//80.0f;
	PidFrictionSpeed[1].I=0.0f;
	PidFrictionSpeed[1].D=0.0f;
	PidFrictionSpeed[1].IMax=1500.0f;
	PidFrictionSpeed[1].SetPoint=0.0f;
	PidFrictionSpeed[1].OutMax = 9900.0f;
	PidFrictionSpeed[1].RC_DF = 0.05F;
	PidFrictionSpeed[1].I_L = 50;
	PidFrictionSpeed[1].I_U =100;
	
#if Robot_ID == 3
/********************************************* 3号车 *******************************************************/	
			Infantry.Low_FrictionSpeed = 4900;
			Infantry.Medium_FrictionSpeed = 5800;
			Infantry.High_FrictionSpeed =10000;
	
#elif  Robot_ID == 4
/********************************************* 4号车 *******************************************************/	

			Infantry.Low_FrictionSpeed = 4800;    //4850:14.1  射频：4.5
 			Infantry.Medium_FrictionSpeed = 5650;  //17.4
			Infantry.High_FrictionSpeed =10300;

#elif  Robot_ID == 14
/********************************************* 14号车 *******************************************************/	
			Infantry.Low_FrictionSpeed = 5000;
			Infantry.Medium_FrictionSpeed = 5800;
			Infantry.High_FrictionSpeed = 8000;
	
#elif  Robot_ID == 5
/********************************************* 5号车 *******************************************************/	

			Infantry.Low_FrictionSpeed = 4900;
			Infantry.Medium_FrictionSpeed = 5750;
			Infantry.High_FrictionSpeed = 11500;
		
#elif  Robot_ID == 44
/********************************************* 44号车 *******************************************************/	

			Infantry.Low_FrictionSpeed = 4370;
			Infantry.Medium_FrictionSpeed = 4800;
			Infantry.High_FrictionSpeed = 7300;
		
#elif  Robot_ID == 45
/********************************************* 45号车 *******************************************************/	

			Infantry.Low_FrictionSpeed = 4180;
			Infantry.Medium_FrictionSpeed = 4730;
			Infantry.High_FrictionSpeed = 7300;
#elif  Robot_ID == 46
/********************************************* 46号车 *******************************************************/	
			Infantry.Low_FrictionSpeed = 5100;//15
			Infantry.Medium_FrictionSpeed = 6020;//18
			Infantry.High_FrictionSpeed = 11500;//30
			PidFrictionSpeed[0].OutMax = 13000.0f;
			PidFrictionSpeed[1].OutMax = 13000.0f;			
#elif  Robot_ID == 47
/********************************************* 47号车 *******************************************************/	

			Infantry.Low_FrictionSpeed = 4370;
			Infantry.Medium_FrictionSpeed = 4800;
			Infantry.High_FrictionSpeed = 7100;	
/********************************************* 缺省值 ******************************************************/		
#else
			Infantry.Low_FrictionSpeed = 4850;
			Infantry.Medium_FrictionSpeed = 5800;
			Infantry.High_FrictionSpeed = 16000;
#endif


	
}

/**********************************************************************************************************
*函 数 名: Shoot_task关键参数初始化
*功能说明: 拨弹任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Shoot_init(){
	F105.JudgeReceive_info.HeatMax17 = 100; //Shoot.HeatControl.HeatMax17 = 90;
	F105.JudgeReceive_info.HeatCool17 = 10;
	
	Shoot.HeatControl.ShootCount_IntervalTime=0;
	Shoot.HeatControl.ShootContinue_IntervalTime=0;
}

/**********************************************************************************************************
*函 数 名: Shoot_task
*功能说明: 拨弹任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
#define POWER_ON 1
#define POWER_OFF 0
uint32_t Shoot_high_water;
int Shoot_Power = POWER_ON;		//读取发射机构电压
extern short FrictionCurrent[2];
void Shoot_task(void *pvParameters)
{
  portTickType xLastWakeTime;
	const portTickType xFrequency = 5;
	uint32_t ShootTask_TimeLast = 0;
	
	Shoot_init();
	
	vTaskDelay(500);
  while (1) {
	  xLastWakeTime = xTaskGetTickCount();
		HeatControl(GetDeltaT(&ShootTask_TimeLast));
		
		 if(Shoot_Power == POWER_ON)
		 {

			if(Status.ShootMode == Shoot_Powerdown_Mode)
			{
				F405.Fric_Flag=0;
				FrictionWheel_speed=0;
			}else		//已经初始化成功一次了，并且切回了非掉电模式
			{
				F405.Fric_Flag=1;
				FrictionSpeedChoose();
			}
		 }
		 else if(Shoot_Power == POWER_OFF)
		 {
				//禁止拨盘转即可
			 BodanMotorCurrent = 0.0f;
			 FrictionCurrent[0] = 0.0f;
			 FrictionCurrent[1] = 0.0f;
		 }
		 
		 Pluck_Speed_Choose();   //射频选择
		 BodanMotorPos_Set();
		 IWDG_Feed();
     vTaskDelayUntil(&xLastWakeTime,xFrequency); 
	
#if  INCLUDE_uxTaskGetStackHighWaterMark
        Shoot_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

