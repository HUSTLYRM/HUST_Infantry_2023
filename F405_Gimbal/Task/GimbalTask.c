/**********************************************************************************************************
 * @文件     GimbalTask.c
 * @说明     云台控制
 * @版本  	 V1.0
 * @作者     黄志雄
 * @日期     2020.1
 **********************************************************************************************************/
/**********************************************************************************************************
 * @文件     GimbalTask.c
 * @说明     云台控制
 * @版本  	 V2.0
 * @作者     戴军
 * @日期     2022.4
 **********************************************************************************************************/
#include "main.h"
/*----------------------------------内部变量---------------------------*/

int inttoshort[4];
short GimbalAct_Init_Flag = 0;

short YawCurrent, PitchCurrent;
float GimbalYawPos, GimbalPitchPos,GimbalLastPitchPos,DeltaGimbalPitchPos;

float Recent_Angle_Buff;
float Recent_Yaw_Angle_Armor;
float Recent_Pitch_Angle_Armor;
float pitchpos;
uint8_t ControlMode = 0;     //初始控制模式为陀螺仪控制
/*-----------------------------------结构体-----------------------------*/
Pid_Typedef PidPitchSpeed, PidPitchPos, PidYawSpeed, PidYawPos;
Pid_Typedef PidPitchAidPos[2], PidPitchAidSpeed[2], PidYawAidPos[2], PidYawAidSpeed[2], PidPitchBuffSpeed, PidYawBuffSpeed;
FeedForward_t PitchPosFF,PitchSpeedFF,YawPosFF,YawSpeedFF;
/*----------------------------------外部变量---------------------------*/
extern RC_Ctl_t RC_Ctl;
extern Status_t Status;
extern char PitchMotor_ReceiveFlag;
extern Gimbal_Typedef Gimbal;
extern RobotInit_Struct Infantry;
extern short FrictionWheel_speed;
extern F405_typedef F405;
extern PC_Receive_t PC_Receive;
extern char Budan;
extern float Buff_Yaw_Motor;
extern char q_flag;
extern FeedForward_Typedef FF_w;
extern char pitch_lose_flag;
extern TaskHandle_t User_Tasks[TASK_NUM];
extern PCRecvData pc_recv_data;
extern F105_Typedef F105;

uint32_t cnt_last;
/**********************************************************************************************************
 *函 数 名: Gimbal_Powerdown_Cal
 *功能说明: 云台掉电模式
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
void Gimbal_Powerdown_Cal()
{
	if (GimbalAct_Init_Flag != Gimbal_Powerdown_Mode)
	{
		Laser_Off();
		GimbalAct_Init_Flag = Gimbal_Powerdown_Mode;
	}

	/***********************************************************************************/
	PidPitchPos.SetPoint = Infantry.pitch_min_motor;

	/**************************************计算电流值**************************************/
	// PITCH
	PidPitchPos.ActualValue = Gimbal.Pitch.Gyro;
	PidPitchSpeed.SetPoint = -PID_Calc(&PidPitchPos);
	PidPitchSpeed.ActualValue = Gimbal.Pitch.AngularSpeed;
}
/**********************************************************************************************************
 *函 数 名: FuzzyGimbal_Act_Cal
 *功能说明: 模糊云台正常模式(电机角)
 *形    参: rc  mouse  Pc_RecvData
 *返 回 值: 无
 **********************************************************************************************************/
float k_yaw;
char ch2_return_flag;
int diudiao = 0;
double yuzhi = 0.002;
float yaw_freq=1;
float yaw_Amplite=20;
void MotorGimbal_Act_Cal(Remote rc, Mouse mouse)
{
	if (GimbalAct_Init_Flag != Gimbal_Act_Mode)
	{
		Laser_On();
		if(ControlMode == GyroMode)
        {
		GimbalPitchPos = Gimbal.Pitch.Pos_Using_Value;
		GimbalYawPos = Gimbal.Yaw.Pos_Using_Value;
        }
        else
        {
        GimbalPitchPos = Gimbal.Pitch.Pos_Using_Value;
		GimbalYawPos = -Gimbal.Yaw.Pos_Using_Value;
        }
		GimbalAct_Init_Flag = Gimbal_Act_Mode;
		ch2_return_flag = 0;
	}

	if (Status.ControlMode == Control_RC_Mode) // Rc_Control
	{
//		if (rc.ch2 != 1024)
//		{
			GimbalYawPos = GimbalYawPos + (1024 - rc.ch2) * 0.0020f;
			ch2_return_flag = 1;
//		}
//		else if (ch2_return_flag)
//		{
//			GimbalYawPos = Gimbal.Yaw.Pos_Using_Value;
//			ch2_return_flag = 0;
//		}
		GimbalPitchPos = GimbalPitchPos - (1024 - rc.ch3) * 0.001f;
		FF_w.Now_DeltIn = (1024 - rc.ch2) * 0.0016f;
	}
	
	if (Status.ControlMode == Control_MouseKey_Mode) // Mouse_Key
	{
		GimbalPitchPos -= mouse.y * 0.006f; //原本：0.005f
		GimbalYawPos -= mouse.x * 0.004f;	// 0.0016
		GimbalPitchPos += mouse.z * 0.004f;

		FF_w.Now_DeltIn = -mouse.x * 0.0032f;
	}
    
    //PITCH限位
    if(ControlMode == GyroMode)
    {
		DeltaGimbalPitchPos = Gimbal.Pitch.Gyro - Gimbal.Pitch.MotorTransAngle - Infantry.init_delta_pitch;
		GimbalPitchPos = LIMIT_MAX_MIN(GimbalPitchPos,Infantry.pitch_max_gyro + DeltaGimbalPitchPos,Infantry.pitch_min_gyro + DeltaGimbalPitchPos);
    }
    else
    {
		GimbalPitchPos = LIMIT_MAX_MIN(GimbalPitchPos,Infantry.pitch_max_motor,Infantry.pitch_min_motor);
    }
	/***********************************************************************************/

	//	PidPitchPos.SetPoint = GimbalPitchPos;
	//	PidYawPos.SetPoint = GimbalYawPos;
	//
	//  /**************************************计算电流值**************************************/
    if(ControlMode == MotorMode)
    {
    if(ControlMode == MotorMode && Status.ChassisMode == Chassis_SelfProtect_Mode) //非陀螺仪下小陀螺
    GimbalYawPos += F105.ChassisSpeedw*0.006;
	PidYawAidPos[ControlMode].SetPoint = -GimbalYawPos;
    }
    else
    PidYawAidPos[ControlMode].SetPoint = GimbalYawPos;    

	//PidYawAidPos.SetPoint = yaw_Amplite * arm_sin_f32(2 * 3.1415 * yaw_freq * xTaskGetTickCount()/configTICK_RATE_HZ);
	PidPitchAidPos[ControlMode].SetPoint = GimbalPitchPos;
}
/**********************************************************************************************************
 *函 数 名: FuzzyGimbal_Act_Cal
 *功能说明: 模糊云台正常模式(IMU角)
 *形    参: rc  mouse  Pc_RecvData
 *返 回 值: 无
 **********************************************************************************************************/

void GyroGimbal_Act_Cal(Remote rc, Mouse mouse)
{
	if (GimbalAct_Init_Flag != Gimbal_Act_Mode)
	{
		Laser_On();
        if(ControlMode == GyroMode)
        {
		GimbalPitchPos = Gimbal.Pitch.Pos_Using_Value;
        GimbalLastPitchPos = Gimbal.Pitch.Pos_Using_Value;
		GimbalYawPos = Gimbal.Yaw.Pos_Using_Value;
        }
        else
        {
        GimbalPitchPos = Gimbal.Pitch.Pos_Using_Value;
        GimbalLastPitchPos = Gimbal.Pitch.Pos_Using_Value;
		GimbalYawPos = -Gimbal.Yaw.Pos_Using_Value;
        }
		GimbalAct_Init_Flag = Gimbal_Act_Mode;
	}

	if (Status.ControlMode == Control_RC_Mode) // Rc_Control
	{
		GimbalYawPos = (rc.ch2 == 1024) ? Gimbal.Yaw.Gyro : (GimbalYawPos + (1024 - rc.ch2) * 0.0010f);
		GimbalPitchPos = (rc.ch3 == 1024) ? Gimbal.Pitch.Gyro : (GimbalPitchPos - (1024 - rc.ch3) * 0.0005f); //旧陀螺仪
		FF_w.Now_DeltIn = (1024 - rc.ch2) * 0.0005f;
	}
	if (Status.ControlMode == Control_MouseKey_Mode) // Mouse_Key
	{
		GimbalPitchPos -= mouse.y * 0.005f;
		GimbalYawPos -= mouse.x * 0.005f;
		GimbalPitchPos -= mouse.z * 0.001f;
		FF_w.Now_DeltIn = -mouse.x * 0.005f;
	}

	//PITCH限位
    if(ControlMode == GyroMode)
    {
    DeltaGimbalPitchPos = Gimbal.Pitch.Gyro - Gimbal.Pitch.MotorTransAngle - Infantry.init_delta_pitch;
    GimbalPitchPos = LIMIT_MAX_MIN(GimbalPitchPos,Infantry.pitch_max_gyro + DeltaGimbalPitchPos,Infantry.pitch_min_gyro + DeltaGimbalPitchPos);
    }
    else
    {
    GimbalPitchPos = LIMIT_MAX_MIN(GimbalPitchPos,Infantry.pitch_max_motor,Infantry.pitch_min_motor);
    }
    
	/***********************************************************************************/
	PidPitchPos.SetPoint = GimbalPitchPos;
	PidYawPos.SetPoint = GimbalYawPos;

}

/**********************************************************************************************************
 *函 数 名: Gimbal_Armor_Cal
 *功能说明: 云台辅瞄模式
 *形    参: rc  mouse  Pc_RecvData
 *返 回 值: 无
 **********************************************************************************************************/
float speed_limit = 30.0f;
char Aim_Follow;
float Inte_z;
extern float pc_yaw, pc_pitch;
float last_aim_yaw;

void Gimbal_Armor_Cal(Remote rc, Mouse mouse)
{
	if (GimbalAct_Init_Flag != Gimbal_Armor_Mode)
	{
		GimbalAct_Init_Flag = Gimbal_Armor_Mode;
        if(ControlMode == GyroMode)
        {
		last_aim_yaw = GimbalYawPos = Gimbal.Yaw.Pos_Using_Value;
		GimbalPitchPos = Gimbal.Pitch.Pos_Using_Value;
		pc_yaw = Gimbal.Yaw.Pos_Using_Value;
		pc_pitch = Gimbal.Pitch.Pos_Using_Value;
        }
        else
        {
        last_aim_yaw = GimbalYawPos = -Gimbal.Yaw.Pos_Using_Value;
		GimbalPitchPos = Gimbal.Pitch.Pos_Using_Value;
		pc_yaw = -Gimbal.Yaw.Pos_Using_Value;
		pc_pitch = Gimbal.Pitch.Pos_Using_Value;
        }
		//Inte_z = 0;
	}

    if(pc_recv_data.enemy_id !=0) //判断辅瞄是否工作
    {
	Recent_Pitch_Angle_Armor = pc_pitch;
	Recent_Yaw_Angle_Armor = pc_yaw;
	Inte_z += mouse.z * 0.002f;

	if (ABS(Recent_Yaw_Angle_Armor - Gimbal.Yaw.Pos_Using_Value) < 70 && ABS(Recent_Pitch_Angle_Armor - (Gimbal.Pitch.Pos_Using_Value)) < 60) //程序安全
	{
		GimbalPitchPos = Recent_Pitch_Angle_Armor + Inte_z;
		GimbalYawPos = Recent_Yaw_Angle_Armor;
		FF_w.Now_DeltIn = ((GimbalYawPos - last_aim_yaw) > 0.2f) ? ((GimbalYawPos - last_aim_yaw)) : (0);
	}
	else
	{
		GimbalPitchPos = Gimbal.Pitch.Pos_Using_Value; //更新值
		GimbalYawPos = Gimbal.Yaw.Pos_Using_Value;
		FF_w.Now_DeltIn = 0;
	}
    }

	last_aim_yaw = GimbalYawPos;
    
    //PITCH限位
    if(ControlMode == GyroMode)
    {
    DeltaGimbalPitchPos = Gimbal.Pitch.Gyro - Gimbal.Pitch.MotorTransAngle - Infantry.init_delta_pitch;
    GimbalPitchPos = LIMIT_MAX_MIN(GimbalPitchPos,Infantry.pitch_max_gyro + DeltaGimbalPitchPos,Infantry.pitch_min_gyro + DeltaGimbalPitchPos);
    }
    else
    {
    GimbalPitchPos = LIMIT_MAX_MIN(GimbalPitchPos,Infantry.pitch_max_motor,Infantry.pitch_min_motor);
    }
    
	/***********************************************************************************/ 
        PidPitchAidPos[ControlMode].SetPoint = GimbalPitchPos;
		if(ControlMode == MotorMode)
        PidYawAidPos[ControlMode].SetPoint = -GimbalYawPos;
        else
        PidYawAidPos[ControlMode].SetPoint = GimbalYawPos; 
		
    
}

/**********************************************************************************************************
 *函 数 名: Gimbal_Buff_Cal
 *功能说明: 云台打符模式
 *形    参: rc  mouse  Pc_RecvData
 *返 回 值: 无
 **********************************************************************************************************/
void Gimbal_Buff_Cal(Remote rc, Mouse mouse)
{
	if (GimbalAct_Init_Flag != Gimbal_BigBuf_Mode)
	{
		GimbalAct_Init_Flag = Gimbal_BigBuf_Mode;
        if(ControlMode == GyroMode)
        {
		GimbalYawPos = Gimbal.Yaw.Pos_Using_Value;
		GimbalPitchPos = Gimbal.Pitch.Pos_Using_Value;
		pc_yaw = Gimbal.Yaw.Pos_Using_Value;
		pc_pitch = Gimbal.Pitch.Pos_Using_Value;
        }
        else
        {
        GimbalYawPos = -Gimbal.Yaw.Pos_Using_Value;
		GimbalPitchPos = Gimbal.Pitch.Pos_Using_Value;
		pc_yaw = -Gimbal.Yaw.Pos_Using_Value;
		pc_pitch = Gimbal.Pitch.Pos_Using_Value;
        }
	}


        Recent_Pitch_Angle_Armor = pc_pitch;
        Recent_Yaw_Angle_Armor = pc_yaw;
        Inte_z += mouse.z * 0.002f;

        if (ABS(Recent_Yaw_Angle_Armor - Gimbal.Yaw.Pos_Using_Value) < 70 && ABS(Recent_Pitch_Angle_Armor - Gimbal.Pitch.Pos_Using_Value) < 60) //程序安全
        {
            GimbalPitchPos = Recent_Pitch_Angle_Armor + Inte_z; //更新值
            GimbalYawPos = Recent_Yaw_Angle_Armor;
        }
        else
        {
            GimbalPitchPos = Gimbal.Pitch.Gyro; //更新值
            GimbalYawPos = Gimbal.Yaw.Gyro;
        }

    //PITCH限位
    if(ControlMode == GyroMode)
    {
    DeltaGimbalPitchPos = Gimbal.Pitch.Gyro - Gimbal.Pitch.MotorTransAngle - Infantry.init_delta_pitch;
    GimbalPitchPos = LIMIT_MAX_MIN(GimbalPitchPos,Infantry.pitch_max_gyro + DeltaGimbalPitchPos,Infantry.pitch_min_gyro + DeltaGimbalPitchPos);
    }
    else
    {
    GimbalPitchPos = LIMIT_MAX_MIN(GimbalPitchPos,Infantry.pitch_max_motor,Infantry.pitch_min_motor);
    }
    
	/***************************************************************************************/
    PidPitchAidPos[ControlMode].SetPoint = GimbalPitchPos;
	if(ControlMode == MotorMode)
	PidYawAidPos[ControlMode].SetPoint = -GimbalYawPos;
    else
    PidYawAidPos[ControlMode].SetPoint = GimbalYawPos; 
}

/**********************************************************************************************************
 *函 数 名: Gimbal_Test_Cal
 *功能说明: 云台测试模式
 *形    参: rc  mouse  Pc_RecvData
 *返 回 值: 无
 **********************************************************************************************************/
void Gimbal_Test_Cal(Remote rc, Mouse mouse)
{
	if (GimbalAct_Init_Flag != Gimbal_Test_Mode)
	{
		GimbalAct_Init_Flag = Gimbal_Test_Mode;
		GimbalYawPos = Gimbal.Yaw.Pos_Using_Value;
		GimbalPitchPos = Gimbal.Pitch.Pos_Using_Value;
	}
	if (Status.ControlMode == Control_RC_Mode) // Rc_Control
	{
		GimbalYawPos += (1024 - rc.ch2) * 0.00005f;
		GimbalPitchPos -= (1024 - rc.ch3) * 0.00005f; //陀螺仪
	}

    //PITCH限位
    if(ControlMode == GyroMode)
    {
    DeltaGimbalPitchPos = Gimbal.Pitch.Gyro - Gimbal.Pitch.MotorTransAngle - Infantry.init_delta_pitch;
    GimbalPitchPos = LIMIT_MAX_MIN(GimbalPitchPos,Infantry.pitch_max_gyro + DeltaGimbalPitchPos,Infantry.pitch_min_gyro + DeltaGimbalPitchPos);
    }
    else
    {
    GimbalPitchPos = LIMIT_MAX_MIN(GimbalPitchPos,Infantry.pitch_max_motor,Infantry.pitch_min_motor);
    }
    
	/***************************************************************************************/
//	PidPitchAidPos[ControlMode].SetPoint = GimbalPitchPos;
//  PidPitchAidPos.SetPoint = TestPos[countertest%4][0]; //矩形
	float pitch_freq = 1.5; //HZ
	PidPitchAidPos[ControlMode].SetPoint = 20 * arm_sin_f32(2 * 3.1415 * pitch_freq * xTaskGetTickCount()/configTICK_RATE_HZ) + 8; //正弦
//  PidPitchAidPos.SetPoint = TD_Calculate(&PitchTD,TestPitch);
	
//	if(ControlMode == MotorMode)
//	PidYawAidPos[ControlMode].SetPoint = -GimbalYawPos;
//    else
//    PidYawAidPos[ControlMode].SetPoint = GimbalYawPos; 
//  PidYawAidPos.SetPoint = TestPos[countertest%4][1]; //矩形
//  PidPitchAidPos[ControlMode].SetPoint = yaw_Amplite * arm_sin_f32(2 * 3.1415 * yaw_freq * xTaskGetTickCount()/configTICK_RATE_HZ); //正弦
//  PidYawAidPos.SetPoint = TD_Calculate(&YawTD,TestYaw);
//  PidYawAidPos.SetPoint = TD_Calculate(&YawTD,GimbalYawPos);
	
//  PidYawAidSpeed.SetPoint = A*arm_sin_f32((float)counterbase/w); //正弦

}

/**********************************************************************************************************
 *函 数 名: Gimbal_SI_Cal
 *功能说明: 云台系统辨识模式
 *形    参: rc  mouse  Pc_RecvData
 *返 回 值: 无
 **********************************************************************************************************/
short F_Change_flag = 0; //切换频率标志
float Gimbal_direct;

int T;					   //周期
int T_cnt = 0;			   //计数
int T_Time_cnt = 0;		   //周期次数计数
int F_cnt = 0, F_cnt_last; //指向F的指针
float F = 1;

void Gimbal_SI_Cal(float Gimbal_pitch, float Gimbal_yaw)
{
	if (GimbalAct_Init_Flag != Gimbal_SI_Mode)
	{
		GimbalPitchPos = 0;
		GimbalYawPos = Gimbal.Yaw.Pos_Using_Value;
		GimbalPitchPos = Gimbal.Pitch.Pos_Using_Value; //用于阶跃响应测试
		GimbalYawPos = Gimbal.Yaw.Pos_Using_Value + 40.0f;
		F = 1;
		GimbalAct_Init_Flag = Gimbal_SI_Mode;
	}

	T_change();
	//		GimbalPitchPos = LIMIT_MAX_MIN((Gimbal_direct*Gimbal_pitch),Infantry.pitch_max_motor,Infantry.pitch_min_motor);

	/**************************************计算电流值**************************************/
	/***************************************************************************************/
    //PITCH限位
    if(ControlMode == GyroMode)
    {
    DeltaGimbalPitchPos = Gimbal.Pitch.Gyro - Gimbal.Pitch.MotorTransAngle - Infantry.init_delta_pitch;
    GimbalPitchPos = LIMIT_MAX_MIN(GimbalPitchPos,Infantry.pitch_max_gyro + DeltaGimbalPitchPos,Infantry.pitch_min_gyro + DeltaGimbalPitchPos);
    }
    else
    {
    GimbalPitchPos = LIMIT_MAX_MIN(GimbalPitchPos,Infantry.pitch_max_motor,Infantry.pitch_min_motor);
    }
    
	/***********************************************************************************/
//	FuzzyAidPidPitchPos.SetPoint = GimbalPitchPos;
//	FuzzyAidPidPitchPos.ActualValue = Gimbal.Pitch.MotorTransAngle;

//	PidPitchAidSpeed.SetPoint = -FuzzyPID_Calc(&FuzzyAidPidPitchPos);
//	PidPitchAidSpeed.ActualValue = GyroReceive.GY;
//	inttoshort[0] = -(PID_Calc(&PidPitchAidSpeed)); //旧陀螺仪
//	PitchCurrent = -(short)inttoshort[0] * Infantry.motor_pn;

//	FuzzyAidPidYawPos.SetPoint = GimbalYawPos;
//	FuzzyAidPidYawPos.ActualValue = Gimbal.Yaw.Gyro;

//	PidYawAidSpeed.SetPoint = FuzzyPID_Calc(&FuzzyAidPidYawPos);
//	PidYawAidSpeed.ActualValue = GyroReceive.GZ;
//	inttoshort[1] = -PID_Calc(&PidYawAidSpeed);
//	YawCurrent = inttoshort[1];

	///************************************** 用于开环系统辨识 ******************************************/
	//  	PitchCurrent= Gimbal_direct*Gimbal_pitch;
	//	  YawCurrent = Gimbal_direct*Gimbal_yaw;
}

void get_F(void)
{
	static int j = 0;
	if (F < 22)
	{
		j++;
		F += 0.5f;
	}
	else if (F == 22)
	{
		F = 24;
	}
	else if (F >= 24 && F < 40)
	{
		F = F + 2;
	}
	else if (F == 40)
	{
		F = 50;
	}
	else if (F >= 50 && F < 120)
	{
		F = F + 10;
	}
	else if (F == 120)
	{
		F = 200;
	}
	else if (F == 250)
	{
		F = 333;
	}
	else if (F == 333)
	{
		F = 500;
	}
}
/**********************************************************************************************************
 *函 数 名: T_Change
 *功能说明: 根据不同的频率获得对应不同周期下sin信号,F低于50重复10周期，否则20周期
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
void T_change(void)
{
	T = round(1000 / F);
	if (GimbalAct_Init_Flag == Gimbal_SI_Mode)
	{
		if (F_Change_flag == 0)
		{
			Gimbal_direct = sin(2 * 3.14 * T_cnt / T);
			if (T_cnt >= T)
			{
				T_cnt = 0;
				T_Time_cnt++;
			}
			T_cnt++;
			F_cnt_last = F_cnt;

			if (T_Time_cnt >= 10 && F < 22)
			{
				T_Time_cnt = 0;
				F_cnt++;
			}
			else if (T_Time_cnt >= 10 && F >= 22 && F < 50)
			{
				T_Time_cnt = 0;
				F_cnt++;
			}
			else if (T_Time_cnt >= 20 && F >= 50)
			{
				T_Time_cnt = 0;
				F_cnt++;
			}
			if (F_cnt != F_cnt_last) //频率改变
			{
				F_Change_flag = 1;
			}
		}
		else if (F_Change_flag == 1)
		{
			Gimbal_direct = 0;
			if ((ABS(PidPitchPos.SetPoint - PidPitchPos.ActualValue) < 0.2f) || (ABS(PidYawPos.SetPoint - PidYawPos.ActualValue) < 0.2f)) //回到初始位置
			{
				get_F();
				T_cnt = 0;
				F_Change_flag = 0;
			}
		}
	}
	else if (GimbalAct_Init_Flag != Gimbal_SI_Mode)
	{
		F = 1;
	}
}
/**********************************************************************************************************
 *函 数 名: Gimbal_CurrentPid_Cal
 *功能说明: 发送电流值
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
void GimbalPos_Set(void){
	switch (Status.GimbalMode)
	{
	case Gimbal_Powerdown_Mode:
		Gimbal_Powerdown_Cal();
		break;

	case Gimbal_Act_Mode:
		MotorGimbal_Act_Cal(RC_Ctl.rc, RC_Ctl.mouse);
		break;

	case Gimbal_Armor_Mode:
		Gimbal_Armor_Cal(RC_Ctl.rc, RC_Ctl.mouse);
		break;

	case Gimbal_BigBuf_Mode:
	case Gimbal_SmlBuf_Mode:
		Gimbal_Buff_Cal(RC_Ctl.rc, RC_Ctl.mouse);
		break;

		//		case Gimbal_DropShot_Mode:
		//			Gimbal_DropShot_Cal(RC_Ctl.rc,RC_Ctl.mouse);
		//			break;
	
	case Gimbal_SI_Mode:
		Gimbal_SI_Cal(8000.0, 0); // pitch
		//			Gimbal_SI_Cal(0.0, 30.0);		//yaw
		break;

	case Gimbal_Jump_Mode:
		GyroGimbal_Act_Cal(RC_Ctl.rc, RC_Ctl.mouse);
		break;

	case Gimbal_Test_Mode:
		Gimbal_Test_Cal(RC_Ctl.rc, RC_Ctl.mouse);
		break;

	default:
		Gimbal_Powerdown_Cal();
		break;
	}
    
//	if (q_flag)
//		F405.Gimbal_Flag = 7; //云台自己打弹
//	else
//		F405.Gimbal_Flag = Status.GimbalMode;
  F405.Gimbal_Flag = Status.GimbalMode;
}
/**********************************************************************************************************
 *函 数 名: Gimbal_CurrentPid_Cal
 *功能说明: 发送电流值
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
float GimbalYawPos_last,GimbalPitchPos_last;
void Gimbal_CurrentPid_Cal(void)
{
	switch (Status.GimbalMode)
	{
	case Gimbal_Powerdown_Mode:
		PitchCurrent = 0;
		YawCurrent = 0;
		break;

	case Gimbal_Act_Mode:
		PidPitchAidPos[ControlMode].ActualValue = Gimbal.Pitch.Pos_Using_Value;
		PidPitchAidSpeed[ControlMode].SetPoint = -PID_Calc(&PidPitchAidPos[ControlMode]);  //+FeedForward_Cal(&PitchPosFF,PidPitchAidPos.SetPoint);
		PidPitchAidSpeed[ControlMode].ActualValue = Gimbal.Pitch.Speed_Using_Value;  
		inttoshort[0] = -(PID_Calc(&PidPitchAidSpeed[ControlMode])); //旧陀螺仪
		PitchCurrent = -(short)inttoshort[0] * Infantry.motor_pn;
		
		PidYawAidPos[ControlMode].ActualValue = Gimbal.Yaw.Pos_Using_Value;
		PidYawAidSpeed[ControlMode].SetPoint = PID_Calc(&PidYawAidPos[ControlMode])+FeedForward_Cal(&YawPosFF,PidYawAidPos[ControlMode].SetPoint);
        PidYawAidSpeed[ControlMode].ActualValue = Gimbal.Yaw.Speed_Using_Value;
		inttoshort[1] = -PID_Calc(&PidYawAidSpeed[ControlMode]);
		YawCurrent = -(short)inttoshort[1];
		break;

	case Gimbal_Armor_Mode:
		PidPitchAidPos[ControlMode].ActualValue = Gimbal.Pitch.Pos_Using_Value;
		PidPitchAidSpeed[ControlMode].SetPoint = -PID_Calc(&PidPitchAidPos[ControlMode]);  //+FeedForward_Cal(&PitchPosFF,PidPitchAidPos.SetPoint);
		PidPitchAidSpeed[ControlMode].ActualValue = Gimbal.Pitch.Speed_Using_Value;  
		inttoshort[0] = -(PID_Calc(&PidPitchAidSpeed[ControlMode])); //旧陀螺仪
		PitchCurrent = -(short)inttoshort[0] * Infantry.motor_pn;

		PidYawAidPos[ControlMode].ActualValue = Gimbal.Yaw.Pos_Using_Value;
		PidYawAidSpeed[ControlMode].SetPoint = PID_Calc(&PidYawAidPos[ControlMode]);//+FeedForward_Cal(&YawPosFF,PidYawAidPos.SetPoint);
		PidYawAidSpeed[ControlMode].ActualValue = Gimbal.Yaw.Speed_Using_Value;
		inttoshort[1] = -PID_Calc(&PidYawAidSpeed[ControlMode]);
		YawCurrent = -(short)inttoshort[1];
		break;

	case Gimbal_BigBuf_Mode:
	case Gimbal_SmlBuf_Mode:
		PidPitchAidPos[ControlMode].ActualValue = Gimbal.Pitch.Pos_Using_Value;
		PidPitchAidSpeed[ControlMode].SetPoint = -PID_Calc(&PidPitchAidPos[ControlMode]);  //+FeedForward_Cal(&PitchPosFF,PidPitchAidPos.SetPoint);
		PidPitchAidSpeed[ControlMode].ActualValue = Gimbal.Pitch.Speed_Using_Value;  
		inttoshort[0] = -(PID_Calc(&PidPitchAidSpeed[ControlMode])); //旧陀螺仪
		PitchCurrent = -(short)inttoshort[0] * Infantry.motor_pn;
		
		PidYawAidPos[ControlMode].ActualValue = Gimbal.Yaw.Pos_Using_Value;
		PidYawAidSpeed[ControlMode].SetPoint = PID_Calc(&PidYawAidPos[ControlMode])+FeedForward_Cal(&YawPosFF,PidYawAidPos[ControlMode].SetPoint);
		PidYawAidSpeed[ControlMode].ActualValue = Gimbal.Yaw.Speed_Using_Value;
		inttoshort[1] = -PID_Calc(&PidYawAidSpeed[ControlMode]);
		YawCurrent = -(short)inttoshort[1];
		break;

	case Gimbal_SI_Mode:
		//Gimbal_SI_Cal(8000.0, 0); // pitch
		//			Gimbal_SI_Cal(0.0, 30.0);		//yaw
		break;

	case Gimbal_Jump_Mode:
	
		PidPitchPos.ActualValue = Gimbal.Pitch.Pos_Using_Value;
		PidPitchSpeed.SetPoint = -PID_Calc(&PidPitchPos);
		PidPitchSpeed.ActualValue = Gimbal.Pitch.Speed_Using_Value;
		inttoshort[0] = -(PID_Calc(&PidPitchSpeed)); //旧陀螺仪
		PitchCurrent = (short)(-inttoshort[0]) * Infantry.motor_pn;
	
		PidYawPos.ActualValue = Gimbal.Yaw.Pos_Using_Value;
		PidYawSpeed.SetPoint = PID_Calc(&PidYawPos); //需要YAW轴陀螺仪角度做ActualValue
		PidYawSpeed.ActualValue = Gimbal.Yaw.Speed_Using_Value;
		inttoshort[1] = -PID_Calc(&PidYawSpeed);
		YawCurrent = (short)inttoshort[1];

		break;

	case Gimbal_Test_Mode:
		PidPitchAidPos[ControlMode].ActualValue = Gimbal.Pitch.Pos_Using_Value;
		PidPitchAidSpeed[ControlMode].SetPoint = -PID_Calc(&PidPitchAidPos[ControlMode]);  //+FeedForward_Cal(&PitchPosFF,PidPitchAidPos.SetPoint);
		PidPitchAidSpeed[ControlMode].ActualValue = Gimbal.Pitch.Speed_Using_Value;  
		inttoshort[0] = -(PID_Calc(&PidPitchAidSpeed[ControlMode])); //旧陀螺仪
		PitchCurrent = -(short)inttoshort[0] * Infantry.motor_pn;
	
		//PidYawAidPos[ControlMode].ActualValue = Gimbal.Yaw.Pos_Using_Value;
		//PidYawAidSpeed[ControlMode].SetPoint = PID_Calc(&PidYawAidPos[ControlMode]);  //+FeedForward_Cal(&YawPosFF,PidYawAidPos.SetPoint);
		PidYawAidSpeed[ControlMode].SetPoint = 330;
		PidYawAidSpeed[ControlMode].ActualValue = Gimbal.Yaw.Speed_Using_Value;
		inttoshort[1] = -PID_Calc(&PidYawAidSpeed[ControlMode]);
		YawCurrent = -(short)inttoshort[1];
		break;

	default:
		PitchCurrent = 0;
		YawCurrent = 0;
		break;
	}
}

/**********************************************************************************************************
*函 数 名: Gimbal_FF_Init
*功能说明: 云台前馈初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Gimbal_FF_Init()
{
	
  PitchPosFF.param[0] = 0;
  PitchPosFF.param[1] = 0;
  PitchPosFF.param[2] = 0;
  PitchPosFF.val = 0;
  PitchPosFF.val_dot = 0;
  PitchPosFF.val_ddot = 0;
  PitchPosFF.OutMax = 5;
  
  PitchSpeedFF.param[0] = 0;
  PitchSpeedFF.param[1] = 0;
  PitchSpeedFF.param[2] = 0;
  PitchSpeedFF.val = 0;
  PitchSpeedFF.val_dot = 0;
  PitchSpeedFF.val_ddot = 0;
  PitchSpeedFF.OutMax = 5;
  
  YawPosFF.param[0] = 0;
  YawPosFF.param[1] = 100;
  YawPosFF.param[2] = 0;
  YawPosFF.val = 0;
  YawPosFF.val_dot = 0;
	YawPosFF.val_dot_out_RC = 0.8;
  YawPosFF.val_ddot = 0;
  YawPosFF.OutMax = 200;

  YawSpeedFF.param[0] = 0;
  YawSpeedFF.param[1] = 0;
  YawSpeedFF.param[2] = 0;
  YawSpeedFF.val = 0;
  YawSpeedFF.val_dot = 0;
  YawSpeedFF.val_ddot = 0;
  YawSpeedFF.OutMax = 8000;
	
}

/**********************************************************************************************************
 *函 数 名: Pid_Yaw_MotorPos_GyroSpeed
 *功能说明: Yaw轴辅瞄双环PID
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
void PidGimbalMotor_Init(void)
{
#if Robot_ID == 3
	/********************************************* 3号车 ********************************************************/
	//手动pitch双环
	PidPitchPos.P = 0.25f; //手动pitch角度环
	PidPitchPos.I = 0.000f;
	PidPitchPos.D = 0.0f;
	PidPitchPos.IMax = 10.0f;
	PidPitchPos.SetPoint = 0.0f;
	PidPitchPos.OutMax = 5.5f;
	PidPitchPos.I_L = 0.2f;
	PidPitchPos.I_U = 0.4f;
	PidPitchPos.RC_DF = 0.5f;

	PidPitchSpeed.P = 10000.0f; //手动pitch速度环
	PidPitchSpeed.I = 12.0f;
	PidPitchSpeed.D = 0.0f;
	PidPitchSpeed.IMax = 550.0f;
	PidPitchSpeed.SetPoint = 0.0f;
	PidPitchSpeed.OutMax = 30000.0f;
	PidPitchSpeed.I_L = 0.3f;
	PidPitchSpeed.I_U = 0.7f;
	PidPitchSpeed.RC_DF = 0.5f;

	//手动yaw双环                                 // 3号车
	PidYawPos.P = 0.22f; //手动yaw角度环
	PidYawPos.I = 0.00f;
	PidYawPos.D = 0.0f;
	PidYawPos.IMax = 10.0f;
	PidYawPos.SetPoint = 0.0f;
	PidYawPos.OutMax = 5.5f;
	PidYawPos.I_L = 0.2f;
	PidYawPos.I_U = 0.4f;
	PidYawPos.RC_DF = 0.5f;

	PidYawSpeed.P = 22000.0f; //手动yaw速度环
	PidYawSpeed.I = 1.0f;
	PidYawSpeed.D = 0.0f;
	PidYawSpeed.IMax = 2000.0f;
	PidYawSpeed.SetPoint = 0.0f;
	PidYawSpeed.OutMax = 30000.0f;
	PidYawSpeed.I_L = 0.3f;
	PidYawSpeed.I_U = 0.7f;
	PidYawSpeed.RC_DF = 0.5f;

	//辅瞄yaw
	PidYawAidPos.P = 0.25f; // yaw PID位置环（辅瞄）
	PidYawAidPos.I = 0.000f;
	PidYawAidPos.D = 0.20f;
	PidYawAidPos.IMax = 40.0f;
	PidYawAidPos.SetPoint = 0.0f;
	PidYawAidPos.OutMax = 5.5f;
	PidYawAidPos.I_L = 0.2f;
	PidYawAidPos.I_U = 0.4f;
	PidYawAidPos.RC_DF = 0.5f;

	FuzzyAidPidYawPos.Kp0 = 0.25f; //模糊PID yaw角度环（辅瞄）
	FuzzyAidPidYawPos.Ki0 = 0.0001f;
	FuzzyAidPidYawPos.Kd0 = 0.1f;
	FuzzyAidPidYawPos.IMax = 40.0f;
	FuzzyAidPidYawPos.SetPoint = 0.0f;
	FuzzyAidPidYawPos.OutMax = 8.5f;
	FuzzyAidPidYawPos.I_L = 0.2f;
	FuzzyAidPidYawPos.I_U = 0.4f;
	FuzzyAidPidYawPos.RC_DF = 0.5f;

	FuzzyAidPidYawPos.stair = 0.2f;
	FuzzyAidPidYawPos.Kp_stair = 0.015f;
	FuzzyAidPidYawPos.Ki_stair = 0.0005f;
	FuzzyAidPidYawPos.Kd_stair = 0.001f;

	PidYawAidSpeed.P = 10000.0f; // 32000  yaw速度环PID(辅瞄)
	PidYawAidSpeed.I = 3.0f;
	PidYawAidSpeed.D = 0.0f;
	PidYawAidSpeed.IMax = 2000.0f;
	PidYawAidSpeed.SetPoint = 0.0f;
	PidYawAidSpeed.OutMax = 30000.0f;
	PidYawAidSpeed.I_L = 0.3f;
	PidYawAidSpeed.I_U = 0.7f;
	PidYawAidSpeed.RC_DF = 0.5f;
	//辅瞄pitch
	PidPitchAidPos.P = 0.2f; //普通PID位置环(辅瞄)
	PidPitchAidPos.I = 0.00001f;
	PidPitchAidPos.D = 0.0f;
	PidPitchAidPos.IMax = 40.0f;
	PidPitchAidPos.SetPoint = 0.0f;
	PidPitchAidPos.OutMax = 5.5f;
	PidPitchAidPos.I_L = 0.2f;
	PidPitchAidPos.I_U = 0.4f;
	PidPitchAidPos.RC_DF = 0.5f;

	FuzzyAidPidPitchPos.Kp0 = 0.28f;
	FuzzyAidPidPitchPos.Ki0 = 0.00f; //模糊PID位置环（辅瞄）
	FuzzyAidPidPitchPos.Kd0 = 0.1f;
	FuzzyAidPidPitchPos.IMax = 40.0f;
	FuzzyAidPidPitchPos.SetPoint = 0.0f;
	FuzzyAidPidPitchPos.OutMax = 8.5f;
	FuzzyAidPidPitchPos.I_L = 0.2f;
	FuzzyAidPidPitchPos.I_U = 0.4f;
	FuzzyAidPidPitchPos.RC_DF = 0.5f;

	FuzzyAidPidPitchPos.stair = 0.2f;
	FuzzyAidPidPitchPos.Kp_stair = 0.015f;
	FuzzyAidPidPitchPos.Ki_stair = 0.0005f;
	FuzzyAidPidPitchPos.Kd_stair = 0.001f;

	PidPitchAidSpeed.P = 10000.0f; //速度环PID（辅瞄）
	PidPitchAidSpeed.I = 18.0f;
	PidPitchAidSpeed.D = 0.0f;
	PidPitchAidSpeed.IMax = 250.0f;
	PidPitchAidSpeed.SetPoint = 0.0f;
	PidPitchAidSpeed.OutMax = 30000.0f;
	PidPitchAidSpeed.I_L = 0.3f;
	PidPitchAidSpeed.I_U = 0.7f;
	PidPitchAidSpeed.RC_DF = 0.5f;

	//大符 Pitch
	FuzzyBuffPidPitchPos.Kp0 = 0.28f;
	FuzzyBuffPidPitchPos.Ki0 = 0.003f; //模糊PID位置环（辅瞄）
	FuzzyBuffPidPitchPos.Kd0 = 0.0f;
	FuzzyBuffPidPitchPos.IMax = 40.0f;
	FuzzyBuffPidPitchPos.SetPoint = 0.0f;
	FuzzyBuffPidPitchPos.OutMax = 8.5f;
	FuzzyBuffPidPitchPos.I_L = 0.2f;
	FuzzyBuffPidPitchPos.I_U = 0.4f;
	FuzzyBuffPidPitchPos.RC_DF = 0.5f;

	FuzzyBuffPidPitchPos.stair = 0.2f;
	FuzzyBuffPidPitchPos.Kp_stair = 0.015f;
	FuzzyBuffPidPitchPos.Ki_stair = 0.0005f;
	FuzzyBuffPidPitchPos.Kd_stair = 0.001f;

	PidPitchBuffSpeed.P = 10000.0f; //速度环PID（辅瞄）
	PidPitchBuffSpeed.I = 15.0f;
	PidPitchBuffSpeed.D = 0.0f;
	PidPitchBuffSpeed.IMax = 250.0f;
	PidPitchBuffSpeed.SetPoint = 0.0f;
	PidPitchBuffSpeed.OutMax = 30000.0f;
	PidPitchBuffSpeed.I_L = 0.3f;
	PidPitchBuffSpeed.I_U = 0.7f;
	PidPitchBuffSpeed.RC_DF = 0.5f;

	//大符 Yaw
	FuzzyBuffPidYawPos.Kp0 = 0.25f; //模糊PID yaw角度环（辅瞄）
	FuzzyBuffPidYawPos.Ki0 = 0.0001f;
	FuzzyBuffPidYawPos.Kd0 = 0.0f;
	FuzzyBuffPidYawPos.IMax = 40.0f;
	FuzzyBuffPidYawPos.SetPoint = 0.0f;
	FuzzyBuffPidYawPos.OutMax = 8.5f;
	FuzzyBuffPidYawPos.I_L = 0.2f;
	FuzzyBuffPidYawPos.I_U = 0.4f;
	FuzzyBuffPidYawPos.RC_DF = 0.5f;

	FuzzyBuffPidYawPos.stair = 0.2f;
	FuzzyBuffPidYawPos.Kp_stair = 0.015f;
	FuzzyBuffPidYawPos.Ki_stair = 0.0005f;
	FuzzyBuffPidYawPos.Kd_stair = 0.001f;

	PidYawBuffSpeed.P = 10000.0f; //  yaw速度环PID(辅瞄)
	PidYawBuffSpeed.I = 5.0f;
	PidYawBuffSpeed.D = 0.0f;
	PidYawBuffSpeed.IMax = 2000.0f;
	PidYawBuffSpeed.SetPoint = 0.0f;
	PidYawBuffSpeed.OutMax = 30000.0f;
	PidYawBuffSpeed.I_L = 0.3f;
	PidYawBuffSpeed.I_U = 0.7f;
	PidYawBuffSpeed.RC_DF = 0.5f;

#elif Robot_ID == 4
	/********************************************* 4号车 ********************************************************/
	//手动pitch双环
	PidPitchPos.P = 0.25f; //手动pitch角度环
	PidPitchPos.I = 0.01f;
	PidPitchPos.D = 0.0f;
	PidPitchPos.IMax = 10.0f;
	PidPitchPos.SetPoint = 0.0f;
	PidPitchPos.OutMax = 5.5f;
	PidPitchPos.I_L = 0.2f;
	PidPitchPos.I_U = 0.4f;
	PidPitchPos.RC_DF = 0.5f;

	PidPitchSpeed.P = 12000.0f; //手动pitch速度环
	PidPitchSpeed.I = 7.0f;
	PidPitchSpeed.D = 0.0f;
	PidPitchSpeed.IMax = 550.0f;
	PidPitchSpeed.SetPoint = 0.0f;
	PidPitchSpeed.OutMax = 30000.0f;
	PidPitchSpeed.I_L = 0.3f;
	PidPitchSpeed.I_U = 0.7f;
	PidPitchSpeed.RC_DF = 0.5f;

	//手动yaw双环                                 // 4号车
	PidYawPos.P = 0.25f; //手动yaw角度环
	PidYawPos.I = 0.00f;
	PidYawPos.D = 0.0f;
	PidYawPos.IMax = 10.0f;
	PidYawPos.SetPoint = 0.0f;
	PidYawPos.OutMax = 5.5f;
	PidYawPos.I_L = 0.2f;
	PidYawPos.I_U = 0.4f;
	PidYawPos.RC_DF = 0.5f;

	PidYawSpeed.P = 12000.0f; //手动yaw速度环
	PidYawSpeed.I = 0.0f;
	PidYawSpeed.D = 0.0f;
	PidYawSpeed.IMax = 2000.0f;
	PidYawSpeed.SetPoint = 0.0f;
	PidYawSpeed.OutMax = 30000.0f;
	PidYawSpeed.I_L = 0.3f;
	PidYawSpeed.I_U = 0.7f;
	PidYawSpeed.RC_DF = 0.5f;

	//辅瞄yaw
	PidYawAidPos.P = 0.25f; // yaw PID位置环（辅瞄）
	PidYawAidPos.I = 0.000f;
	PidYawAidPos.D = 0.0f;
	PidYawAidPos.IMax = 40.0f;
	PidYawAidPos.SetPoint = 0.0f;
	PidYawAidPos.OutMax = 5.5f;
	PidYawAidPos.I_L = 0.2f;
	PidYawAidPos.I_U = 0.4f;
	PidYawAidPos.RC_DF = 0.5f;

	FuzzyAidPidYawPos.Kp0 = 0.25f; //模糊PID yaw角度环（辅瞄）
	FuzzyAidPidYawPos.Ki0 = 0.0000f;
	FuzzyAidPidYawPos.Kd0 = 0.0f;
	FuzzyAidPidYawPos.IMax = 40.0f;
	FuzzyAidPidYawPos.SetPoint = 0.0f;
	FuzzyAidPidYawPos.OutMax = 8.5f;
	FuzzyAidPidYawPos.I_L = 0.2f;
	FuzzyAidPidYawPos.I_U = 0.4f;
	FuzzyAidPidYawPos.RC_DF = 0.5f;

	FuzzyAidPidYawPos.stair = 0.2f;
	FuzzyAidPidYawPos.Kp_stair = 0.015f;
	FuzzyAidPidYawPos.Ki_stair = 0.0005f;
	FuzzyAidPidYawPos.Kd_stair = 0.001f;

	PidYawAidSpeed.P = 10000.0f; // 32000  yaw速度环PID(辅瞄)
	PidYawAidSpeed.I = 5.0f;
	PidYawAidSpeed.D = 0.0f;
	PidYawAidSpeed.IMax = 2000.0f;
	PidYawAidSpeed.SetPoint = 0.0f;
	PidYawAidSpeed.OutMax = 30000.0f;
	PidYawAidSpeed.I_L = 0.3f;
	PidYawAidSpeed.I_U = 0.7f;
	PidYawAidSpeed.RC_DF = 0.5f;
	//辅瞄pitch
	PidPitchAidPos.P = 0.25f; //普通PID位置环(辅瞄)
	PidPitchAidPos.I = 0.003f;
	PidPitchAidPos.D = 0.0f;
	PidPitchAidPos.IMax = 40.0f;
	PidPitchAidPos.SetPoint = 0.0f;
	PidPitchAidPos.OutMax = 5.5f;
	PidPitchAidPos.I_L = 0.2f;
	PidPitchAidPos.I_U = 0.4f;
	PidPitchAidPos.RC_DF = 0.5f;

	FuzzyAidPidPitchPos.Kp0 = 0.23f;
	FuzzyAidPidPitchPos.Ki0 = 0.003f; //模糊PID位置环（辅瞄）
	FuzzyAidPidPitchPos.Kd0 = 0.0f;
	FuzzyAidPidPitchPos.IMax = 40.0f;
	FuzzyAidPidPitchPos.SetPoint = 0.0f;
	FuzzyAidPidPitchPos.OutMax = 8.5f;
	FuzzyAidPidPitchPos.I_L = 0.2f;
	FuzzyAidPidPitchPos.I_U = 0.4f;
	FuzzyAidPidPitchPos.RC_DF = 0.5f;

	FuzzyAidPidPitchPos.stair = 0.2f;
	FuzzyAidPidPitchPos.Kp_stair = 0.015f;
	FuzzyAidPidPitchPos.Ki_stair = 0.0005f;
	FuzzyAidPidPitchPos.Kd_stair = 0.001f;

	PidPitchAidSpeed.P = 10000.0f; //速度环PID（辅瞄）
	PidPitchAidSpeed.I = 15.0f;
	PidPitchAidSpeed.D = 0.0f;
	PidPitchAidSpeed.IMax = 250.0f;
	PidPitchAidSpeed.SetPoint = 0.0f;
	PidPitchAidSpeed.OutMax = 30000.0f;
	PidPitchAidSpeed.I_L = 0.3f;
	PidPitchAidSpeed.I_U = 0.7f;
	PidPitchAidSpeed.RC_DF = 0.5f;

	PidPitchBuffSpeed.P = 10000.0f; //速度环PID（辅瞄）
	PidPitchBuffSpeed.I = 15.0f;
	PidPitchBuffSpeed.D = 0.0f;
	PidPitchBuffSpeed.IMax = 250.0f;
	PidPitchBuffSpeed.SetPoint = 0.0f;
	PidPitchBuffSpeed.OutMax = 30000.0f;
	PidPitchBuffSpeed.I_L = 0.3f;
	PidPitchBuffSpeed.I_U = 0.7f;
	PidPitchBuffSpeed.RC_DF = 0.5f;


	PidYawBuffSpeed.P = 12000.0f; //  yaw速度环PID(辅瞄)
	PidYawBuffSpeed.I = 5.0f;
	PidYawBuffSpeed.D = 0.0f;
	PidYawBuffSpeed.IMax = 2000.0f;
	PidYawBuffSpeed.SetPoint = 0.0f;
	PidYawBuffSpeed.OutMax = 30000.0f;
	PidYawBuffSpeed.I_L = 0.3f;
	PidYawBuffSpeed.I_U = 0.7f;
	PidYawBuffSpeed.RC_DF = 0.5f;

#elif Robot_ID == 14
	/********************************************* 14号车 ********************************************************/
	//手动pitch双环
	PidPitchPos.P = 0.35f; //手动pitch角度环
	PidPitchPos.I = 0.0001f;
	PidPitchPos.D = 0.0f;
	PidPitchPos.IMax = 10.0f;
	PidPitchPos.SetPoint = 0.0f;
	PidPitchPos.OutMax = 5.5f;
	PidPitchPos.I_L = 0.2f;
	PidPitchPos.I_U = 0.4f;
	PidPitchPos.RC_DF = 0.5f;

	PidPitchSpeed.P = 12000.0f; //手动pitch速度环
	PidPitchSpeed.I = 12.0f;
	PidPitchSpeed.D = 0.0f;
	PidPitchSpeed.IMax = 550.0f;
	PidPitchSpeed.SetPoint = 0.0f;
	PidPitchSpeed.OutMax = 30000.0f;
	PidPitchSpeed.I_L = 0.3f;
	PidPitchSpeed.I_U = 0.7f;
	PidPitchSpeed.RC_DF = 0.5f;

	//手动yaw双环                                 // 5号车
	PidYawPos.P = 0.22f; //手动yaw角度环
	PidYawPos.I = 0.00f;
	PidYawPos.D = 0.0f;
	PidYawPos.IMax = 10.0f;
	PidYawPos.SetPoint = 0.0f;
	PidYawPos.OutMax = 5.5f;
	PidYawPos.I_L = 0.2f;
	PidYawPos.I_U = 0.4f;
	PidYawPos.RC_DF = 0.5f;

	PidYawSpeed.P = 22000.0f; //手动yaw速度环
	PidYawSpeed.I = 0.0f;
	PidYawSpeed.D = 0.0f;
	PidYawSpeed.IMax = 2000.0f;
	PidYawSpeed.SetPoint = 0.0f;
	PidYawSpeed.OutMax = 30000.0f;
	PidYawSpeed.I_L = 0.3f;
	PidYawSpeed.I_U = 0.7f;
	PidYawSpeed.RC_DF = 0.5f;

	//辅瞄yaw
	PidYawAidPos.P = 0.20f; // yaw PID位置环（辅瞄）
	PidYawAidPos.I = 0.000f;
	PidYawAidPos.D = 0.20f;
	PidYawAidPos.IMax = 40.0f;
	PidYawAidPos.SetPoint = 0.0f;
	PidYawAidPos.OutMax = 5.5f;
	PidYawAidPos.I_L = 0.2f;
	PidYawAidPos.I_U = 0.4f;
	PidYawAidPos.RC_DF = 0.5f;


	PidYawAidSpeed.P = 10000.0f; // 32000  yaw速度环PID(辅瞄)
	PidYawAidSpeed.I = 5.0f;
	PidYawAidSpeed.D = 0.0f;
	PidYawAidSpeed.IMax = 2000.0f;
	PidYawAidSpeed.SetPoint = 0.0f;
	PidYawAidSpeed.OutMax = 30000.0f;
	PidYawAidSpeed.I_L = 0.3f;
	PidYawAidSpeed.I_U = 0.7f;
	PidYawAidSpeed.RC_DF = 0.5f;
	//辅瞄pitch
	PidPitchAidPos.P = 0.45f; //普通PID位置环(辅瞄)
	PidPitchAidPos.I = 0.01f;
	PidPitchAidPos.D = 0.0f;
	PidPitchAidPos.IMax = 40.0f;
	PidPitchAidPos.SetPoint = 0.0f;
	PidPitchAidPos.OutMax = 5.5f;
	PidPitchAidPos.I_L = 0.2f;
	PidPitchAidPos.I_U = 0.4f;
	PidPitchAidPos.RC_DF = 0.5f;


	PidPitchAidSpeed.P = 10000.0f; //速度环PID（辅瞄）
	PidPitchAidSpeed.I = 15.0f;
	PidPitchAidSpeed.D = 0.0f;
	PidPitchAidSpeed.IMax = 250.0f;
	PidPitchAidSpeed.SetPoint = 0.0f;
	PidPitchAidSpeed.OutMax = 30000.0f;
	PidPitchAidSpeed.I_L = 0.3f;
	PidPitchAidSpeed.I_U = 0.7f;
	PidPitchAidSpeed.RC_DF = 0.5f;

#elif Robot_ID == 5
	/********************************************* 5号车 ********************************************************/
	//手动pitch双环
	PidPitchPos.P = 0.20f; //手动pitch角度环
	PidPitchPos.I = 0.0f;
	PidPitchPos.D = 0.05f;
	PidPitchPos.IMax = 10.0f;
	PidPitchPos.SetPoint = 0.0f;
	PidPitchPos.OutMax = 5.5f;
	PidPitchPos.I_L = 0.2f;
	PidPitchPos.I_U = 0.4f;
	PidPitchPos.RC_DF = 0.8f;

	PidPitchSpeed.P = 14000.0f; //手动pitch速度环
	PidPitchSpeed.I = 15.0f;
	PidPitchSpeed.D = 0.0f;
	PidPitchSpeed.IMax = 550.0f;
	PidPitchSpeed.SetPoint = 0.0f;
	PidPitchSpeed.OutMax = 30000.0f;
	PidPitchSpeed.I_L = 0.3f;
	PidPitchSpeed.I_U = 0.7f;
	PidPitchSpeed.RC_DF = 0.5f;

	//手动yaw双环                                 // 5号车
	PidYawPos.P = 0.25f; //手动yaw角度环
	PidYawPos.I = 0.00f;
	PidYawPos.D = 0.0f;
	PidYawPos.IMax = 10.0f;
	PidYawPos.SetPoint = 0.0f;
	PidYawPos.OutMax = 5.5f;
	PidYawPos.I_L = 0.2f;
	PidYawPos.I_U = 0.4f;
	PidYawPos.RC_DF = 0.5f;

	PidYawSpeed.P = 15000.0f; //手动yaw速度环
	PidYawSpeed.I = 1.0f;
	PidYawSpeed.D = 0.0f;
	PidYawSpeed.IMax = 2000.0f;
	PidYawSpeed.SetPoint = 0.0f;
	PidYawSpeed.OutMax = 30000.0f;
	PidYawSpeed.I_L = 0.3f;
	PidYawSpeed.I_U = 0.7f;
	PidYawSpeed.RC_DF = 0.5f;

	//辅瞄yaw
	PidYawAidPos.P = 0.3f; // yaw PID位置环（辅瞄）
	PidYawAidPos.I = 0.000f;
	PidYawAidPos.D = 0.20f;
	PidYawAidPos.IMax = 40.0f;
	PidYawAidPos.SetPoint = 0.0f;
	PidYawAidPos.OutMax = 5.5f;
	PidYawAidPos.I_L = 0.2f;
	PidYawAidPos.I_U = 0.4f;
	PidYawAidPos.RC_DF = 0.5f;


	PidYawAidSpeed.P = 10000.0f; // 32000  yaw速度环PID(辅瞄)
	PidYawAidSpeed.I = 3.0f;
	PidYawAidSpeed.D = 0.0f;
	PidYawAidSpeed.IMax = 2000.0f;
	PidYawAidSpeed.SetPoint = 0.0f;
	PidYawAidSpeed.OutMax = 30000.0f;
	PidYawAidSpeed.I_L = 0.3f;
	PidYawAidSpeed.I_U = 0.7f;
	PidYawAidSpeed.RC_DF = 0.5f;
	//辅瞄pitch
	PidPitchAidPos.P = 0.25f; //普通PID位置环(辅瞄)
	PidPitchAidPos.I = 0.00001f;
	PidPitchAidPos.D = 0.0f;
	PidPitchAidPos.IMax = 40.0f;
	PidPitchAidPos.SetPoint = 0.0f;
	PidPitchAidPos.OutMax = 5.5f;
	PidPitchAidPos.I_L = 0.2f;
	PidPitchAidPos.I_U = 0.4f;
	PidPitchAidPos.RC_DF = 0.5f;


	PidPitchAidSpeed.P = 10000.0f; //速度环PID（辅瞄）
	PidPitchAidSpeed.I = 18.0f;
	PidPitchAidSpeed.D = 0.0f;
	PidPitchAidSpeed.IMax = 250.0f;
	PidPitchAidSpeed.SetPoint = 0.0f;
	PidPitchAidSpeed.OutMax = 30000.0f;
	PidPitchAidSpeed.I_L = 0.3f;
	PidPitchAidSpeed.I_U = 0.7f;
	PidPitchAidSpeed.RC_DF = 0.5f;


	PidPitchBuffSpeed.P = 10000.0f; //速度环PID（辅瞄）
	PidPitchBuffSpeed.I = 18.0f;	// 15.0f;
	PidPitchBuffSpeed.D = 0.0f;
	PidPitchBuffSpeed.IMax = 250.0f;
	PidPitchBuffSpeed.SetPoint = 0.0f;
	PidPitchBuffSpeed.OutMax = 30000.0f;
	PidPitchBuffSpeed.I_L = 0.3f;
	PidPitchBuffSpeed.I_U = 0.7f;
	PidPitchBuffSpeed.RC_DF = 0.5f;


	PidYawBuffSpeed.P = 10000.0f; //  yaw速度环PID(辅瞄)
	PidYawBuffSpeed.I = 3.0f;	  // 5.0f;
	PidYawBuffSpeed.D = 0.0f;
	PidYawBuffSpeed.IMax = 2000.0f;
	PidYawBuffSpeed.SetPoint = 0.0f;
	PidYawBuffSpeed.OutMax = 30000.0f;
	PidYawBuffSpeed.I_L = 0.3f;
	PidYawBuffSpeed.I_U = 0.7f;
	PidYawBuffSpeed.RC_DF = 0.5f;

#elif Robot_ID == 44
	/********************************************* 44号车 ********************************************************/

// 陀螺仪模式
	//辅瞄yaw
	PidYawAidPos[GyroMode].P = 18.0f; // yaw PID位置环（辅瞄）
	PidYawAidPos[GyroMode].I = 0.1f;
	PidYawAidPos[GyroMode].D = 0.0f;
	PidYawAidPos[GyroMode].IMax = 40.0f;
	PidYawAidPos[GyroMode].SetPoint = 0.0f;
	PidYawAidPos[GyroMode].OutMax = 500.0f;
	PidYawAidPos[GyroMode].I_L = 0.2f;
	PidYawAidPos[GyroMode].I_U = 0.4f;
	PidYawAidPos[GyroMode].RC_DF = 0.07f;


	PidYawAidSpeed[GyroMode].P = -600.0f; // 32000  yaw速度环PID(辅瞄)
	PidYawAidSpeed[GyroMode].I = 0.0f;
	PidYawAidSpeed[GyroMode].D = 0.0f;
	PidYawAidSpeed[GyroMode].IMax = 500.0f;
	PidYawAidSpeed[GyroMode].SetPoint = 0.0f;
	PidYawAidSpeed[GyroMode].OutMax = 30000.0f;
	PidYawAidSpeed[GyroMode].I_L = 0.3f;
	PidYawAidSpeed[GyroMode].I_U = 0.7f;
	PidYawAidSpeed[GyroMode].RC_DF = 0.5f;
	//辅瞄pitch
	PidPitchAidPos[GyroMode].P = -22.0f; //普通PID位置环(辅瞄)
	PidPitchAidPos[GyroMode].I = -0.3f;
	PidPitchAidPos[GyroMode].D = 0.0f;
	PidPitchAidPos[GyroMode].IMax = 40.0f;
	PidPitchAidPos[GyroMode].SetPoint = 0.0f;
	PidPitchAidPos[GyroMode].OutMax = 1000.0f;
	PidPitchAidPos[GyroMode].I_L = 0.2f;
	PidPitchAidPos[GyroMode].I_U = 0.4f;
	PidPitchAidPos[GyroMode].RC_DF = 0.5f;

	PidPitchAidSpeed[GyroMode].P = -330.0f; //速度环PID（辅瞄）
	PidPitchAidSpeed[GyroMode].I = -1.0f;
	PidPitchAidSpeed[GyroMode].D = 0.0f;
	PidPitchAidSpeed[GyroMode].IMax =3300.0f;
	PidPitchAidSpeed[GyroMode].SetPoint = 0.0f;
	PidPitchAidSpeed[GyroMode].OutMax = 30000.0f;
	PidPitchAidSpeed[GyroMode].I_L = 18.0f;
	PidPitchAidSpeed[GyroMode].I_U = 42.0f;
	PidPitchAidSpeed[GyroMode].RC_DF = 0.5f;

//电机模式
	//辅瞄yaw
	PidYawAidPos[MotorMode].P = 1.0f; // yaw PID位置环（辅瞄）
	PidYawAidPos[MotorMode].I = 0.0f;
	PidYawAidPos[MotorMode].D = 0.0f;
	PidYawAidPos[MotorMode].IMax = 40.0f;
	PidYawAidPos[MotorMode].SetPoint = 0.0f;
	PidYawAidPos[MotorMode].OutMax = 500.0f;
	PidYawAidPos[MotorMode].I_L = 0.0f;
	PidYawAidPos[MotorMode].I_U = 0.0f;
	PidYawAidPos[MotorMode].RC_DF = 0.07f;


	PidYawAidSpeed[MotorMode].P = 300.0f; // 32000  yaw速度环PID(辅瞄)
	PidYawAidSpeed[MotorMode].I = 0.0f;
	PidYawAidSpeed[MotorMode].D = 0.0f;
	PidYawAidSpeed[MotorMode].IMax = 500.0f;
	PidYawAidSpeed[MotorMode].SetPoint = 0.0f;
	PidYawAidSpeed[MotorMode].OutMax = 30000.0f;
	PidYawAidSpeed[MotorMode].I_L = 0.0f;
	PidYawAidSpeed[MotorMode].I_U = 0.0f;
	PidYawAidSpeed[MotorMode].RC_DF = 0.5f;
	//辅瞄pitch
	PidPitchAidPos[MotorMode].P = 2.0f; //普通PID位置环(辅瞄)
	PidPitchAidPos[MotorMode].I = 0.0f;
	PidPitchAidPos[MotorMode].D = 0.0f;
	PidPitchAidPos[MotorMode].IMax = 40.0f;
	PidPitchAidPos[MotorMode].SetPoint = 0.0f;
	PidPitchAidPos[MotorMode].OutMax = 1000.0f;
	PidPitchAidPos[MotorMode].I_L = 0.2f;
	PidPitchAidPos[MotorMode].I_U = 0.4f;
	PidPitchAidPos[MotorMode].RC_DF = 0.5f;

	PidPitchAidSpeed[MotorMode].P = 500.0f; //速度环PID（辅瞄）
	PidPitchAidSpeed[MotorMode].I = 0.0f;
	PidPitchAidSpeed[MotorMode].D = 0.0f;
	PidPitchAidSpeed[MotorMode].IMax =3300.0f;
	PidPitchAidSpeed[MotorMode].SetPoint = 0.0f;
	PidPitchAidSpeed[MotorMode].OutMax = 30000.0f;
	PidPitchAidSpeed[MotorMode].I_L = 18.0f;
	PidPitchAidSpeed[MotorMode].I_U = 42.0f;
	PidPitchAidSpeed[MotorMode].RC_DF = 0.5f;


    #elif Robot_ID == 45
	/********************************************* 45号车 ********************************************************/
//陀螺仪模式
	//辅瞄yaw
	PidYawAidPos[GyroMode].P = 25.0f; // yaw PID位置环（辅瞄）
	PidYawAidPos[GyroMode].I = 0.1f;
	PidYawAidPos[GyroMode].D = 0.0f;
	PidYawAidPos[GyroMode].IMax = 40.0f;
	PidYawAidPos[GyroMode].SetPoint = 0.0f;
	PidYawAidPos[GyroMode].OutMax = 500.0f;
	PidYawAidPos[GyroMode].I_L = 0.2f;
	PidYawAidPos[GyroMode].I_U = 0.4f;
	PidYawAidPos[GyroMode].RC_DF = 0.07f;


	PidYawAidSpeed[GyroMode].P = -300.0f; // 32000  yaw速度环PID(辅瞄)
	PidYawAidSpeed[GyroMode].I = 0.0f;
	PidYawAidSpeed[GyroMode].D = 0.0f;
	PidYawAidSpeed[GyroMode].IMax = 500.0f;
	PidYawAidSpeed[GyroMode].SetPoint = 0.0f;
	PidYawAidSpeed[GyroMode].OutMax = 30000.0f;
	PidYawAidSpeed[GyroMode].I_L = 0.3f;
	PidYawAidSpeed[GyroMode].I_U = 0.7f;
	PidYawAidSpeed[GyroMode].RC_DF = 0.5f;
	//辅瞄pitch
	PidPitchAidPos[GyroMode].P = -22.0f; //普通PID位置环(辅瞄)
	PidPitchAidPos[GyroMode].I = -0.3f;
	PidPitchAidPos[GyroMode].D = 0.0f;
	PidPitchAidPos[GyroMode].IMax = 40.0f;
	PidPitchAidPos[GyroMode].SetPoint = 0.0f;
	PidPitchAidPos[GyroMode].OutMax = 1000.0f;
	PidPitchAidPos[GyroMode].I_L = 0.2f;
	PidPitchAidPos[GyroMode].I_U = 0.4f;
	PidPitchAidPos[GyroMode].RC_DF = 0.5f;

	PidPitchAidSpeed[GyroMode].P = -330.0f; //速度环PID（辅瞄）
	PidPitchAidSpeed[GyroMode].I = -1.0f;
	PidPitchAidSpeed[GyroMode].D = 0.0f;
	PidPitchAidSpeed[GyroMode].IMax =3300.0f;
	PidPitchAidSpeed[GyroMode].SetPoint = 0.0f;
	PidPitchAidSpeed[GyroMode].OutMax = 30000.0f;
	PidPitchAidSpeed[GyroMode].I_L = 18.0f;
	PidPitchAidSpeed[GyroMode].I_U = 42.0f;
	PidPitchAidSpeed[GyroMode].RC_DF = 0.5f;

//电机模式
	//辅瞄yaw
	PidYawAidPos[MotorMode].P = 1.0f; // yaw PID位置环（辅瞄）
	PidYawAidPos[MotorMode].I = 0.0f;
	PidYawAidPos[MotorMode].D = 0.0f;
	PidYawAidPos[MotorMode].IMax = 40.0f;
	PidYawAidPos[MotorMode].SetPoint = 0.0f;
	PidYawAidPos[MotorMode].OutMax = 500.0f;
	PidYawAidPos[MotorMode].I_L = 0.0f;
	PidYawAidPos[MotorMode].I_U = 0.0f;
	PidYawAidPos[MotorMode].RC_DF = 0.07f;


	PidYawAidSpeed[MotorMode].P = 300.0f; // 32000  yaw速度环PID(辅瞄)
	PidYawAidSpeed[MotorMode].I = 0.0f;
	PidYawAidSpeed[MotorMode].D = 0.0f;
	PidYawAidSpeed[MotorMode].IMax = 500.0f;
	PidYawAidSpeed[MotorMode].SetPoint = 0.0f;
	PidYawAidSpeed[MotorMode].OutMax = 30000.0f;
	PidYawAidSpeed[MotorMode].I_L = 0.0f;
	PidYawAidSpeed[MotorMode].I_U = 0.0f;
	PidYawAidSpeed[MotorMode].RC_DF = 0.5f;
	//辅瞄pitch
	PidPitchAidPos[MotorMode].P = 2.0f; //普通PID位置环(辅瞄)
	PidPitchAidPos[MotorMode].I = 0.0f;
	PidPitchAidPos[MotorMode].D = 0.0f;
	PidPitchAidPos[MotorMode].IMax = 40.0f;
	PidPitchAidPos[MotorMode].SetPoint = 0.0f;
	PidPitchAidPos[MotorMode].OutMax = 1000.0f;
	PidPitchAidPos[MotorMode].I_L = 0.2f;
	PidPitchAidPos[MotorMode].I_U = 0.4f;
	PidPitchAidPos[MotorMode].RC_DF = 0.5f;

	PidPitchAidSpeed[MotorMode].P = 500.0f; //速度环PID（辅瞄）
	PidPitchAidSpeed[MotorMode].I = 0.0f;
	PidPitchAidSpeed[MotorMode].D = 0.0f;
	PidPitchAidSpeed[MotorMode].IMax =3300.0f;
	PidPitchAidSpeed[MotorMode].SetPoint = 0.0f;
	PidPitchAidSpeed[MotorMode].OutMax = 30000.0f;
	PidPitchAidSpeed[MotorMode].I_L = 18.0f;
	PidPitchAidSpeed[MotorMode].I_U = 42.0f;
	PidPitchAidSpeed[MotorMode].RC_DF = 0.5f;
#elif Robot_ID == 46
	/********************************************* 舵轮车 ********************************************************/
	//陀螺仪
	//辅瞄yaw	
	
	PidYawAidPos[GyroMode].P = 25.0f; // yaw PID位置环（辅瞄）
	PidYawAidPos[GyroMode].I = 0.1f;
	PidYawAidPos[GyroMode].D = 0.0f;
	PidYawAidPos[GyroMode].IMax = 40.0f;
	PidYawAidPos[GyroMode].SetPoint = 0.0f;
	PidYawAidPos[GyroMode].OutMax = 500.0f;
	PidYawAidPos[GyroMode].I_L = 0.2f;
	PidYawAidPos[GyroMode].I_U = 0.4f;
	PidYawAidPos[GyroMode].RC_DF = 0.5f;


	PidYawAidSpeed[GyroMode].P = 350.0f; // 32000  yaw速度环PID(辅瞄)
	PidYawAidSpeed[GyroMode].I = 0.0f;
	PidYawAidSpeed[GyroMode].D = 0.0f;
	PidYawAidSpeed[GyroMode].IMax = 500.0f;
	PidYawAidSpeed[GyroMode].SetPoint = 0.0f;
	PidYawAidSpeed[GyroMode].OutMax = 30000.0f;
	PidYawAidSpeed[GyroMode].I_L = 0.3f;
	PidYawAidSpeed[GyroMode].I_U = 0.7f;
	PidYawAidSpeed[GyroMode].RC_DF = 0.5f;
	
	//辅瞄pitch
	PidPitchAidPos[GyroMode].P = -25.0f; //普通PID位置环(辅瞄)
	PidPitchAidPos[GyroMode].I = 0.0f;
	PidPitchAidPos[GyroMode].D = 0.0f;
	PidPitchAidPos[GyroMode].IMax = 40.0f;
	PidPitchAidPos[GyroMode].SetPoint = 0.0f;
	PidPitchAidPos[GyroMode].OutMax = 1000.0f;
	PidPitchAidPos[GyroMode].I_L = 0.2f;
	PidPitchAidPos[GyroMode].I_U = 0.4f;
	PidPitchAidPos[GyroMode].RC_DF = 0.5f;


	PidPitchAidSpeed[GyroMode].P = -220.0f; //速度环PID（辅瞄）
	PidPitchAidSpeed[GyroMode].I = -3.0f;
	PidPitchAidSpeed[GyroMode].D = 0.0f;
	PidPitchAidSpeed[GyroMode].IMax = 250.0f;
	PidPitchAidSpeed[GyroMode].SetPoint = 0.0f;
	PidPitchAidSpeed[GyroMode].OutMax = 30000.0f;
	PidPitchAidSpeed[GyroMode].I_L = 0.3f;
	PidPitchAidSpeed[GyroMode].I_U = 0.7f;
	PidPitchAidSpeed[GyroMode].RC_DF = 0.5f;



	//电机模式
	//辅瞄yaw
	PidYawAidPos[MotorMode].P = 1.0f; // yaw PID位置环（辅瞄）
	PidYawAidPos[MotorMode].I = 0.0f;
	PidYawAidPos[MotorMode].D = 0.0f;
	PidYawAidPos[MotorMode].IMax = 40.0f;
	PidYawAidPos[MotorMode].SetPoint = 0.0f;
	PidYawAidPos[MotorMode].OutMax = 500.0f;
	PidYawAidPos[MotorMode].I_L = 0.0f;
	PidYawAidPos[MotorMode].I_U = 0.0f;
	PidYawAidPos[MotorMode].RC_DF = 0.07f;


	PidYawAidSpeed[MotorMode].P = 300.0f; // 32000  yaw速度环PID(辅瞄)
	PidYawAidSpeed[MotorMode].I = 0.0f;
	PidYawAidSpeed[MotorMode].D = 0.0f;
	PidYawAidSpeed[MotorMode].IMax = 500.0f;
	PidYawAidSpeed[MotorMode].SetPoint = 0.0f;
	PidYawAidSpeed[MotorMode].OutMax = 30000.0f;
	PidYawAidSpeed[MotorMode].I_L = 0.0f;
	PidYawAidSpeed[MotorMode].I_U = 0.0f;
	PidYawAidSpeed[MotorMode].RC_DF = 0.5f;
	//辅瞄pitch
	PidPitchAidPos[MotorMode].P = 2.0f; //普通PID位置环(辅瞄)
	PidPitchAidPos[MotorMode].I = 0.0f;
	PidPitchAidPos[MotorMode].D = 0.0f;
	PidPitchAidPos[MotorMode].IMax = 40.0f;
	PidPitchAidPos[MotorMode].SetPoint = 0.0f;
	PidPitchAidPos[MotorMode].OutMax = 1000.0f;
	PidPitchAidPos[MotorMode].I_L = 0.2f;
	PidPitchAidPos[MotorMode].I_U = 0.4f;
	PidPitchAidPos[MotorMode].RC_DF = 0.5f;

	PidPitchAidSpeed[MotorMode].P = 500.0f; //速度环PID（辅瞄）
	PidPitchAidSpeed[MotorMode].I = 0.0f;
	PidPitchAidSpeed[MotorMode].D = 0.0f;
	PidPitchAidSpeed[MotorMode].IMax =3300.0f;
	PidPitchAidSpeed[MotorMode].SetPoint = 0.0f;
	PidPitchAidSpeed[MotorMode].OutMax = 30000.0f;
	PidPitchAidSpeed[MotorMode].I_L = 18.0f;
	PidPitchAidSpeed[MotorMode].I_U = 42.0f;
	PidPitchAidSpeed[MotorMode].RC_DF = 0.5f;
    
#elif Robot_ID == 47
	/********************************************* 45号车 ********************************************************/
//陀螺仪模式
	//辅瞄yaw
	PidYawAidPos[GyroMode].P = 28.0f; // yaw PID位置环（辅瞄）28
	PidYawAidPos[GyroMode].I = 0.3f;    //0.3
	PidYawAidPos[GyroMode].D = 0.0f;
	PidYawAidPos[GyroMode].IMax = 40.0f;
	PidYawAidPos[GyroMode].SetPoint = 0.0f;
	PidYawAidPos[GyroMode].OutMax = 500.0f;
	PidYawAidPos[GyroMode].I_L = 0.2f;
	PidYawAidPos[GyroMode].I_U = 0.4f;
	PidYawAidPos[GyroMode].RC_DF = 0.07f;


	PidYawAidSpeed[GyroMode].P = -285.0f; // 32000  yaw速度环PID(辅瞄)-285
	PidYawAidSpeed[GyroMode].I = -1.0f;     //-1
	PidYawAidSpeed[GyroMode].D = 0.0f;
	PidYawAidSpeed[GyroMode].IMax = 500.0f;
	PidYawAidSpeed[GyroMode].SetPoint = 0.0f;
	PidYawAidSpeed[GyroMode].OutMax = 30000.0f;
	PidYawAidSpeed[GyroMode].I_L = 0.3f;
	PidYawAidSpeed[GyroMode].I_U = 0.7f;
	PidYawAidSpeed[GyroMode].RC_DF = 0.5f;
	//辅瞄pitch
	PidPitchAidPos[GyroMode].P = -30.0f; //普通PID位置环(辅瞄)-30
	PidPitchAidPos[GyroMode].I = -1.1f;     //-1.1
	PidPitchAidPos[GyroMode].D = 0.0f;
	PidPitchAidPos[GyroMode].IMax = 40.0f;
	PidPitchAidPos[GyroMode].SetPoint = 0.0f;
	PidPitchAidPos[GyroMode].OutMax = 1000.0f;
	PidPitchAidPos[GyroMode].I_L = 0.2f;
	PidPitchAidPos[GyroMode].I_U = 0.4f;
	PidPitchAidPos[GyroMode].RC_DF = 0.5f;

	PidPitchAidSpeed[GyroMode].P = -295.0f; //速度环PID（辅瞄）-295
	PidPitchAidSpeed[GyroMode].I = -1.5f;    //-1.5
	PidPitchAidSpeed[GyroMode].D = 0.0f;
	PidPitchAidSpeed[GyroMode].IMax =3300.0f;
	PidPitchAidSpeed[GyroMode].SetPoint = 0.0f;
	PidPitchAidSpeed[GyroMode].OutMax = 30000.0f;
	PidPitchAidSpeed[GyroMode].I_L = 18.0f;
	PidPitchAidSpeed[GyroMode].I_U = 42.0f;
	PidPitchAidSpeed[GyroMode].RC_DF = 0.5f;

//电机模式
	//辅瞄yaw
	PidYawAidPos[MotorMode].P = 1.0f; // yaw PID位置环（辅瞄）
	PidYawAidPos[MotorMode].I = 0.0f;
	PidYawAidPos[MotorMode].D = 0.0f;
	PidYawAidPos[MotorMode].IMax = 40.0f;
	PidYawAidPos[MotorMode].SetPoint = 0.0f;
	PidYawAidPos[MotorMode].OutMax = 500.0f;
	PidYawAidPos[MotorMode].I_L = 0.0f;
	PidYawAidPos[MotorMode].I_U = 0.0f;
	PidYawAidPos[MotorMode].RC_DF = 0.07f;


	PidYawAidSpeed[MotorMode].P = 300.0f; // 32000  yaw速度环PID(辅瞄)
	PidYawAidSpeed[MotorMode].I = 0.0f;
	PidYawAidSpeed[MotorMode].D = 0.0f;
	PidYawAidSpeed[MotorMode].IMax = 500.0f;
	PidYawAidSpeed[MotorMode].SetPoint = 0.0f;
	PidYawAidSpeed[MotorMode].OutMax = 30000.0f;
	PidYawAidSpeed[MotorMode].I_L = 0.0f;
	PidYawAidSpeed[MotorMode].I_U = 0.0f;
	PidYawAidSpeed[MotorMode].RC_DF = 0.5f;
	//辅瞄pitch
	PidPitchAidPos[MotorMode].P = 2.0f; //普通PID位置环(辅瞄)
	PidPitchAidPos[MotorMode].I = 0.0f;
	PidPitchAidPos[MotorMode].D = 0.0f;
	PidPitchAidPos[MotorMode].IMax = 40.0f;
	PidPitchAidPos[MotorMode].SetPoint = 0.0f;
	PidPitchAidPos[MotorMode].OutMax = 1000.0f;
	PidPitchAidPos[MotorMode].I_L = 0.2f;
	PidPitchAidPos[MotorMode].I_U = 0.4f;
	PidPitchAidPos[MotorMode].RC_DF = 0.5f;

	PidPitchAidSpeed[MotorMode].P = 500.0f; //速度环PID（辅瞄）
	PidPitchAidSpeed[MotorMode].I = 0.0f;
	PidPitchAidSpeed[MotorMode].D = 0.0f;
	PidPitchAidSpeed[MotorMode].IMax =3300.0f;
	PidPitchAidSpeed[MotorMode].SetPoint = 0.0f;
	PidPitchAidSpeed[MotorMode].OutMax = 30000.0f;
	PidPitchAidSpeed[MotorMode].I_L = 18.0f;
	PidPitchAidSpeed[MotorMode].I_U = 42.0f;
	PidPitchAidSpeed[MotorMode].RC_DF = 0.5f;
#endif
}

/**********************************************************************************************************
 *函 数 名: Gimbal_task
 *功能说明: 云台任务函数
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
uint32_t Gimbal_high_water;
UBaseType_t GGGG;
extern TaskHandle_t RCReceiveTask_Handler; //任务句柄
extern TaskHandle_t PCReceiveTask_Handler; //任务句柄
extern uint8_t Remote_Receive_Flag;
//extern uint8_t PC_ReceiveFlag;

void Gimbal_task(void *pvParameters)
{
	portTickType xLastWakeTime;
	const portTickType xFrequency = 2;
	
	Gimbal_FF_Init();
	vTaskDelay(200);
	while (1)
	{
		xLastWakeTime = xTaskGetTickCount();
		
		if(Remote_Receive_Flag) //数据解码
		{
			xTaskNotifyGive(User_Tasks[RC_TASK]);
		}

//		if (PC_ReceiveFlag) //数据解码
//		{
//			xTaskNotifyGive(User_Tasks[PC_TASK]);
//		}

		GimbalPos_Set();
		
		IWDG_Feed();

		vTaskDelayUntil(&xLastWakeTime, xFrequency);

#if INCLUDE_uxTaskGetStackHighWaterMark
		Gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
	}
}
