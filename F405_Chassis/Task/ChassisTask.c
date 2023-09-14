/**********************************************************************************************************
 * @文件     ChassisTask.c
 * @说明     新底盘控制+新功率限制
 * @版本  	 V1.0
 * @作者     郭嘉豪
 * @日期     2023.7.6
**********************************************************************************************************/
#include "main.h"

#define CAP_MAX_W      7000
#define Rand_S         0.5f   //周期长短
#define RandThreshold  0.4f   //直流偏置
#define RANDA          1.3f   //正弦幅值
/*----------------------------------结构体-----------------------------*/
F105_Typedef F105;//底盘信息，用于板间通讯
Power_Limit_type powerlimit;
chassis_type chassis={0};//底盘控制结构体
/*----------------------------------外部变量---------------------------*/
extern INA260 INA260_1,INA260_2;
extern SuperPower superpower;;
extern JudgeReceive_t JudgeReceive;
extern ChassisSpeed_t chassisSpeed;
extern F405_typedef F405;
extern enum CHARGESTATE_Typedef ChargeState;
extern SD_Saver_t SD_saver;
/**********************************************************************************************************
*函 数 名: motion_decomposition
*功能说明: 运动分解
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
float test_x,test_y,test_w;
char filter_en=1;
char test_cnt[4];
#define  SetUP_T  0.99f

 void motion_decomposition(Velocity_vector_type Velocity_vector[4])
{
#if Mecanum == 1  //麦轮
	Velocity_vector[0].v = LowPass_SetChassis(Velocity_vector[0].v,
										chassis.k_xy*(+chassisSpeed.carSpeedy+(+chassisSpeed.carSpeedx))-chassis.carSpeedw);
	
	Velocity_vector[1].v = LowPass_SetChassis(Velocity_vector[1].v,
										chassis.k_xy*(+chassisSpeed.carSpeedy+(-chassisSpeed.carSpeedx))-chassis.carSpeedw);
	
	Velocity_vector[2].v = LowPass_SetChassis(Velocity_vector[2].v,
										chassis.k_xy*(-chassisSpeed.carSpeedy+(+chassisSpeed.carSpeedx))-chassis.carSpeedw);
	
	Velocity_vector[3].v = LowPass_SetChassis(Velocity_vector[3].v,
										chassis.k_xy*(-chassisSpeed.carSpeedy+(-chassisSpeed.carSpeedx))-chassis.carSpeedw);

#elif Mecanum == 0   //全向轮
/*直着走*/
//	LowPass_SetChassis(&pidChassisWheelSpeed[0].SetPoint,chassis.sgn*(-k_xy*(-chassis.carSpeedy+(+chassis.carSpeedx))-chassis.carSpeedw));
//	LowPass_SetChassis(&pidChassisWheelSpeed[1].SetPoint,chassis.sgn*(-k_xy*(-chassis.carSpeedy+(-chassis.carSpeedx))-chassis.carSpeedw));
//	LowPass_SetChassis(&pidChassisWheelSpeed[2].SetPoint,chassis.sgn*(-k_xy*(+chassis.carSpeedy+(-chassis.carSpeedx))-chassis.carSpeedw));
//	LowPass_SetChassis(&pidChassisWheelSpeed[3].SetPoint,chassis.sgn*(-k_xy*(+chassis.carSpeedy+(+chassis.carSpeedx))-chassis.carSpeedw));

/*斜着走*/	
	Velocity_vector[0].v = LowPass_SetChassis(Velocity_vector[0].v,
										chassis.sgn*(-chassis.k_xy*(-chassisSpeed.carSpeedy)-chassis.carSpeedw));
	
	Velocity_vector[1].v = LowPass_SetChassis(Velocity_vector[1].v,
										chassis.sgn*(-chassis.k_xy*(-chassisSpeed.carSpeedx)-chassis.carSpeedw);
	
	Velocity_vector[2].v = LowPass_SetChassis(Velocity_vector[2].v,
										chassis.sgn*(-chassis.k_xy*(+chassisSpeed.carSpeedy)-chassis.carSpeedw));
	
	Velocity_vector[3].v = LowPass_SetChassis(Velocity_vector[3].v,
										chassis.sgn*(-chassis.k_xy*(+chassisSpeed.carSpeedx)-chassis.carSpeedw));
										

#elif Mecanum == 2   //舵轮
	//速度解算
	ZeroCheck_cal();
	//角度处理 
	float max_speed=16000;
	float car_yaw = R2DEG(atan2(chassisSpeed.carSpeedy, chassisSpeed.carSpeedx)) - chassis.yaw_diff_rec;
	float car_V = 0;
	arm_sqrt_f32(chassisSpeed.carSpeedy * chassisSpeed.carSpeedy + chassisSpeed.carSpeedx * chassisSpeed.carSpeedx,&car_V);
	float flag = chassis.carSpeedw>0?1:-1;//按照旋转方向进行小陀螺补偿
	switch(chassis.Steeringwheel_Mode){
		case Stright:
			chassis.car_yaw = car_yaw;
			chassis.carSpeedw = LIMIT_MAX_MIN(chassisSpeed.carSpeedw*10,max_speed,-max_speed);
			chassis.car_V = LIMIT_MAX_MIN(car_V*10,max_speed,-max_speed);
		break;
		case Small_top_avoid:
			chassis.car_yaw = car_yaw + flag*25;//补偿移动偏移
			chassis.carSpeedw = LIMIT_MAX_MIN(chassisSpeed.carSpeedw*10,max_speed,-max_speed);//匀速小陀螺
			chassis.car_V = LIMIT_MAX_MIN(car_V*10,max_speed*0.7,-max_speed*0.7);
//			if(ABS(car_V)>100){
//				chassis.carSpeedw = LIMIT_MAX_MIN(chassisSpeed.carSpeedw*10,max_speed,-max_speed);//匀速小陀螺
//				chassis.car_V = LIMIT_MAX_MIN(car_V*10,max_speed*0.7,-max_speed*0.7);
//			}else{
//				float flag = chassisSpeed.carSpeedw>0?1:-1;
//				chassis.carSpeedw = (5000*arm_cos_f32(2*DEG2R(chassis.yaw_diff_rec+8))+6000)*flag;
//				chassis.car_V = LIMIT_MAX_MIN(car_V*10,4000,-4000);
//			}
		break;
		case Small_top_move:
			chassis.car_yaw = car_yaw + flag*20;//补偿移动偏移
			chassis.carSpeedw = LIMIT_MAX_MIN(chassisSpeed.carSpeedw*10,max_speed*0.5,-max_speed*0.5);
			chassis.car_V = LIMIT_MAX_MIN(car_V*10,max_speed,-max_speed);
		break;
		case Recovery://小陀螺结束回正
			if(ABS(chassisSpeed.carSpeedw)<500&&(ABS(chassis.yaw_diff_rec-0)<5 || ABS(chassis.yaw_diff_rec-90)<5 || ABS(chassis.yaw_diff_rec-180)<5 || ABS(chassis.yaw_diff_rec+90)<5 || ABS(chassis.yaw_diff_rec+180)<5)){
				chassis.Steeringwheel_Mode = Stright;
			}
		break;
	}
	controlSteering_wheel(&chassis, chassis.car_yaw, chassis.car_V, chassis.carSpeedw);	
	ChassisSteer_cal(&chassis);//计算舵角电流
#endif 
}
/**********************************************************************************************************
*函 数 名: cal_speed_now()
*功能说明: 速度逆解
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void cal_speed_now(int *speedx, int *speedy, int *speedz){
#if Mecanum == 1  //麦轮
	*speedx = chassis.ChassisMotorCanReceive[0].RealSpeed - chassis.ChassisMotorCanReceive[1].RealSpeed
			+ chassis.ChassisMotorCanReceive[2].RealSpeed - chassis.ChassisMotorCanReceive[3].RealSpeed;
	
	*speedy = chassis.ChassisMotorCanReceive[0].RealSpeed + chassis.ChassisMotorCanReceive[1].RealSpeed
			- chassis.ChassisMotorCanReceive[2].RealSpeed - chassis.ChassisMotorCanReceive[3].RealSpeed;
	
	*speedz = - chassis.ChassisMotorCanReceive[0].RealSpeed - chassis.ChassisMotorCanReceive[1].RealSpeed
			- chassis.ChassisMotorCanReceive[2].RealSpeed - chassis.ChassisMotorCanReceive[3].RealSpeed;
	
	*speedx /= 4;
	*speedy /= 4;
	*speedz /= 4;
	
#endif
}


/**********************************************************************************************************
*函 数 名: Add_vector,输入角度为度，返回为度
*功能说明: 速度向量相加,返回向量为-180到180角度
*形    参: 两个向量
*返 回 值: 无
**********************************************************************************************************/
Velocity_vector_type Add_vector(Velocity_vector_type *a, Velocity_vector_type *b)//实现向量a+b
{

	Velocity_vector_type temp;
	float a_yaw = DEG2R(a->yaw);
	float b_yaw = DEG2R(b->yaw);
	float x = a->v * arm_cos_f32(a_yaw) + b->v * arm_cos_f32(b_yaw);
	float y = a->v * arm_sin_f32(a_yaw) + b->v * arm_sin_f32(b_yaw);
	arm_sqrt_f32(x * x + y * y,&(temp.v));
	temp.yaw = R2DEG(atan2(y, x));
	return temp;
}

/**********************************************************************************************************
*函 数 名: calculateShortestDistance
*功能说明: 求两个0-360电机位置的绝对距离，适用于舵轮舵角目标的最短旋转距离，该函数求得的移动角度会是的舵角电机最大旋转不超过90°
*形    参: 当前位置和目标位置，以及是否反向
*返 回 值: 最短移动距离和是否反向，最短旋转距离该函数结果一定能保证不超过90度
**********************************************************************************************************/
float calculateShortestDistance(float yaw_now_360, float yaw_set_360,float* reverflag) {
    float clockwise_distance = fmodf((yaw_set_360 - yaw_now_360 + 360), 360);
    float counter_clockwise_distance = 360 - clockwise_distance;
    float reverse_distance = fabsf(fmodf(yaw_set_360 - yaw_now_360 + 180, 360)) - 180;

    float shortest_distance = clockwise_distance;
	
    if (counter_clockwise_distance < shortest_distance) {
        shortest_distance = -counter_clockwise_distance;
    }
	*reverflag = 1.0;
	//如果反向正向距离都要大于90
	if(ABS(shortest_distance)>90){
		//翻转
		float flipped_yaw_now = yaw_now_360 + 180;
		
		if (flipped_yaw_now >= 360) {
			flipped_yaw_now -= 360;
		}
		if(clockwise_distance > counter_clockwise_distance)//未翻转前正向大于反向，则反转后取反向
		{
			reverse_distance = fmodf((yaw_set_360 - flipped_yaw_now + 360), 360);//求正向距离
		}else{
			reverse_distance = -fmodf(flipped_yaw_now - yaw_set_360 + 360, 360);//求反向距离
		}
		*reverflag = -1.0;
        shortest_distance = reverse_distance;
	}

    return shortest_distance;
}

/**********************************************************************************************************
*函 数 名: control_wheel
*功能说明: 舵轮底盘操控 w_speed旋转速度，move_speed平移速度
*形    参: 外部接口输出速度分解,在曲线模式下 //Gimbal表示云台与底盘朝向夹角，小陀螺模式下表示云台与当前视角的夹角
*返 回 值: 无
**********************************************************************************************************/
void controlSteering_wheel(chassis_type *chassis_, float car_yaw, float car_V, float w_speed)
{	
	Velocity_vector_type Wheel_Temp[4];//用于控制四个轮子运动进行运动合成时候的旋转速度向量

	for (int i = 0; i < 4; i++) {//记录上次速度
		chassis.Velocity_vectors_last[i] = chassis.Velocity_vectors[i];
	}
	
	//arm_sqrt_f32(x_speed * x_speed + y_speed * y_speed,&car_V);
	
	for(int t=0;t<4;t++)
	{
		chassis.Velocity_vectors[t].yaw = car_yaw;
		chassis.Velocity_vectors[t].v = car_V;
	}
	
	if (chassis.Steeringwheel_Mode==Small_top_move || chassis.Steeringwheel_Mode==Small_top_avoid || chassis.Steeringwheel_Mode==Recovery)
	{
		//速度赋值
		Wheel_Temp[0].v = +w_speed;
		Wheel_Temp[1].v = -w_speed;
		Wheel_Temp[2].v = -w_speed;
		Wheel_Temp[3].v = +w_speed;

		//计算轮子垂直向量坐标的角度朝向,底盘坐标系
		Wheel_Temp[0].yaw =   + SMALLTOP_CIRCLE;
		Wheel_Temp[1].yaw =   - SMALLTOP_CIRCLE;
		Wheel_Temp[2].yaw =   + SMALLTOP_CIRCLE;
		Wheel_Temp[3].yaw =   - SMALLTOP_CIRCLE;				
		
		//分别在底盘坐标系中直线与垂直向量相加得到和矢量
		for (int u = 0; u < 4; u++) 
		{
			chassis.Velocity_vectors[u] = Add_vector(&(chassis.Velocity_vectors[u]), &(Wheel_Temp[u]));
		}
	}
	//角度限制，转角大于90，反转
	float yaw_now_360=0,yaw_set_360=0,yaw_diff_360=0;
	for (int i = 0; i < 4; i++) {
		//当前值
		yaw_now_360 = (chassis.SteerMotorsCanReceive[i].Angle - chassis.SteeringAnglePosition_init[i])/8192.0f*360.0f;
		if(yaw_now_360<0) yaw_now_360+=360;
		//目标值
		yaw_set_360 = fmodf(chassis.Velocity_vectors[i].yaw, 360);
		if(yaw_set_360<0) yaw_set_360+=360;
		float reverflag = 1.0;
		yaw_diff_360 = calculateShortestDistance(yaw_now_360, yaw_set_360, &reverflag);//yaw_diff_360[i]不超过90°
		chassis.Velocity_vectors[i].yaw = chassis.SteerMotorsDecode[i].MotorTransAngle + yaw_diff_360;
		chassis.Velocity_vectors[i].v *= reverflag;
		
		//如果旋转角度diff过大，则对转速做一定衰减，这里利用cos3次方函数
		float cos_k = arm_cos_f32(DEG2R(yaw_diff_360));
		chassis.Velocity_vectors[i].v *= (cos_k*cos_k*cos_k);
	}
	
	//电机安装
	chassis.Velocity_vectors[0].v = -chassis.Velocity_vectors[0].v;
	chassis.Velocity_vectors[3].v = -chassis.Velocity_vectors[3].v;
	
	for (int i = 0; i < 4; i++) {//目标低通
		chassis.Velocity_vectors[i].v = LowPass_SetChassis(chassis.Velocity_vectors_last[i].v,
															chassis.Velocity_vectors[i].v);
		
		chassis.Velocity_vectors[i].yaw = LowPass_SetSteer(chassis.Velocity_vectors_last[i].yaw,
															chassis.Velocity_vectors[i].yaw);
	}
}

/**********************************************************************************************************
*函 数 名: ChassisSteer_cal
*功能说明: 舵角电流pid计算
*形    参: 
*返 回 值: 无
**********************************************************************************************************/
void ChassisSteer_cal(chassis_type *chassis_)
{
	short i=0;
	/**************舵向电机******************/

	//传参入位置环
	for(i = 0;i < 4;i++)
	{
		chassis_->PIDSteeringAnglePosition[i].SetPoint = chassis_->Velocity_vectors[i].yaw;//位置环setpoint设置
		chassis_->PIDSteeringAngleSpeed[i].SetPoint = PID_Calc(&chassis_->PIDSteeringAnglePosition[i],chassis_->SteerMotorsDecode[i].MotorTransAngle);	//速度环setpoint设置
		chassis_->SteeringVolSend[i] = PID_Calc(&chassis_->PIDSteeringAngleSpeed[i],chassis_->SteerMotorsDecode[i].RealSpeed);	//控制值
		
		//通过电压电流关系求对应的电流期望值
		chassis.SteeringCurrentSend[i] = Vol2Current(chassis_->SteeringVolSend[i],  chassis_->SteerMotorsDecode[i].RealSpeed);
		
		//保护
		if(chassis.SteerMotorsCanReceive[i].temp > 90){
			chassis_->SteeringVolSend[0] = 0;
			chassis_->SteeringVolSend[1] = 0;
			chassis_->SteeringVolSend[2] = 0;
			chassis_->SteeringVolSend[3] = 0;
		}
	}
}

/**********************************************************************************************************
*函 数 名: Chassis_Speed_Cal
*功能说明: 根据xyw向速度计算目标速度值
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
short test_Self_Protect_Limit = 3600;
float test_k_BAT = 1.0f;
short pre_in[2];
char SelfProtect_Cross_Flag;
int Rand_T,rand_p,rand_cnt,rand_w;
float rand_A,test_rand;
#if  Mecanum == 2
void Chassis_Speed_Cal(void)
{
	static short Angular_Velocity;
	float rotation_lim=1.0f;	
	
	//xy向速度与w向速度配比
	switch(F405.Chassis_Flag)
	{
		case Chassis_Powerdown_Mode:
			chassis.carSpeedw = 0;
			break;
		
		case Chassis_Act_Mode:
		case Chassis_Jump_Mode:
			if(chassis.Steeringwheel_Mode == Small_top_avoid || chassis.Steeringwheel_Mode == Small_top_move){
				chassis.Steeringwheel_Mode = Recovery;
			}
			break;
			
		case Chassis_SelfProtect_Mode:
			chassis.Steeringwheel_Mode=Small_top_avoid;
		break;
			
		case Chassis_Solo_Mode:
			chassis.Steeringwheel_Mode=Small_top_avoid;
		break;
		
		default: 
			chassis.carSpeedw = 0;
			break;
	}
	
#if Mecanum == 0
	if(F405.Chassis_Flag == Chassis_SelfProtect_Mode)
	{
		chassis.sgn = -1.0f;
	}
	else
	{
		chassis.sgn = 1.0f;
	}
#endif
}

#else
//全向轮和麦克纳姆轮
void Chassis_Speed_Cal(void)
{
	static short Angular_Velocity;
	float rotation_lim=1.0f;	
	
	//xy向速度与w向速度配比
	switch(F405.Chassis_Flag)
	{
		case Chassis_Powerdown_Mode:
			chassis.k_xy = 0;
			chassis.carSpeedw = 0;
		break;
		
		case Chassis_Act_Mode:
		case Chassis_Jump_Mode:
			
		if(ABS(chassisSpeed.carSpeedw) > 1000 && ((ABS(chassisSpeed.carSpeedx)>500) || (ABS(chassisSpeed.carSpeedy)) >500)) //车辆行驶中大幅转弯减速
        {
            chassis.k_xy = 2.0f;
            chassis.k_w = 1.3f;
        }
        else if(ABS(chassisSpeed.carSpeedw) > 1000)        //原地转
        {
            chassis.k_xy = 3.0f;
            chassis.k_w = 1.0f;
        }
        else
        {
            chassis.k_xy = 3.0f;
            chassis.k_w = 1.0f;
        }
//		else if((ABS(chassis.carSpeedx)>500) && (ABS(chassis.carSpeedy) >500))  //以下皆为平移或小幅转弯情况
//        {
//            k_xy = 2.5f;    //斜着走
//            k_w = 1.0;
//        }
//        else if((ABS(chassis.carSpeedx)>500) && (ABS(chassis.carSpeedy) <500))
//        {
//            k_xy = 3.0f;       //直着走
//            k_w = 1.0f;
//        }		
//		else if((ABS(chassis.carSpeedx) <500) && (ABS(chassis.carSpeedy)>500))
//        {
//            k_xy =3.0f;     //横着走
//            k_w = 1.0f;
//        }
				
		
	   chassis.carSpeedw = chassisSpeed.carSpeedw;

	 
		break;
		
		case Chassis_SelfProtect_Mode:
		{
			//变速处理
			if( rand_p == rand_cnt)
			{
				Rand_T = (rand()%4000+4000)*Rand_S;
				rand_p = Rand_T*(0.3f+rand()%7/10.0f);
				rand_cnt = 0;
			}
			else
			{
			  rand_cnt ++;
			}
			test_rand = rand_cnt*3.14f/Rand_T*2;
			rand_A = RANDA*ABS(arm_cos_f32(rand_cnt*3.14f/Rand_T*2));
			
			
				if((ABS(chassisSpeed.carSpeedx) >100) || (ABS(chassisSpeed.carSpeedy) >100))
				{
	
							chassis.k_xy = 1.7f;         
							rotation_lim=0.80f;
				}
				else
				{
					chassis.k_xy=0.0f;
					rotation_lim=1.0f;
				}		
				
				if(superpower.PowerState == CAP)
				{
				//匀速小陀螺
					chassis.carSpeedw = LIMIT_MAX_MIN(chassisSpeed.carSpeedw,CAP_MAX_W,-CAP_MAX_W);
//				//        变速小陀螺(消耗能量过高)
//				rand_w = (RandThreshold+(1-RandThreshold)*rand_A)*CAP_MAX_W;
//				chassis.carSpeedw = LIMIT_MAX_MIN(chassis.carSpeedw, rotation_lim*(rand_w), -rotation_lim*(rand_w));
//					
				}
				else 
				{	chassis.Self_Protect_Limit = (powerlimit.referee_max_power - 60)*30+5000;//将60w到最大功率映射到5000-8000
					if(powerlimit.referee_max_power < 120)
					{
			//        匀速小陀螺					
							chassis.carSpeedw = LIMIT_MAX_MIN(chassisSpeed.carSpeedw, rotation_lim*chassis.Self_Protect_Limit, -rotation_lim*chassis.Self_Protect_Limit);
					}
					else
					{
			//        变速小陀螺
							rand_w = (RandThreshold+(1-RandThreshold)*rand_A)*chassis.Self_Protect_Limit;
							chassis.carSpeedw = LIMIT_MAX_MIN(chassisSpeed.carSpeedw, rotation_lim*(rand_w), -rotation_lim*(rand_w));
					}
				}
			}
		break;
			
		case Chassis_Solo_Mode:
		{
			if((ABS(chassisSpeed.carSpeedx) >100) && (ABS(chassisSpeed.carSpeedy) >100))
				chassis.k_xy = 1.4f;
			else
				chassis.k_xy = 2;
			chassis.carSpeedw = chassisSpeed.carSpeedw; 
		}
		break;
		
		default: 
			chassis.k_xy = 0;
			chassis.carSpeedw = 0;
			break;
	}
	
	#if Mecanum == 0
		if(F405.Chassis_Flag == Chassis_SelfProtect_Mode)
		{
			chassis.sgn = -1.0f;
		}
		else
		{
			chassis.sgn = 1.0f;
		}
	#endif
}
#endif
/**********************************************************************************************************
*函 数 名: cal_powerSet
*功能说明: 计算当前功率期望
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void cal_powerSet(Power_Limit_type *power_limit,int powerstate_cur){
	
	// 根据当前功率选择目标功率
	powerlimit.P_capEngry = power_limit->referee_max_power/(0.5*22.0*22.0*CAP_C - powerlimit.Min_capEnergy) + 0.1;//如果电容控不到最低电压，也可以手调，差不多0.15左右
	powerlimit.P_remainEngry = power_limit->referee_max_power/(60 - power_limit->Min_remainEnergy);//缓冲能量保留系数，也可以手调，差不多1.8左右
	
	// 根据当前功率选择目标功率
    if(power_limit->remainEnergy >= 40.0f && ABS(power_limit->Speed_Now[0])<3000)    //剩余能量大于40J，相当于判断为起步，全功率跑
    {
        power_limit->HalfCAP_Power = power_limit->referee_max_power + power_limit->Add_HalfCAP_Power + 100;
    }
	else
			power_limit->HalfCAP_Power = power_limit->referee_max_power + power_limit->Add_HalfCAP_Power;
	
	// 动态最大功率
	switch(power_limit->PowerState_control){
		case CAP:
			if(superpower.PowerState == CAP)
				power_limit->set_power = power_limit->No_limited_Power;
            else//实际电池
            {
                if (power_limit->remainEnergy < power_limit->Min_remainEnergy*1.3){	//最小阈值之前都是恒功率		
                power_limit->set_power = power_limit->referee_max_power + 
                                         power_limit->P_remainEngry*(power_limit->remainEnergy - power_limit->Min_remainEnergy);	
                }else{
                    power_limit->set_power = power_limit->referee_max_power + 60;	
                }
            }  
			break;
		case HalfCAP:
		if(powerstate_cur == CAP){
			if (power_limit->capEnergy < power_limit->Min_capEnergy*1.2){	//最小阈值之前都是恒功率		
				power_limit->set_power = power_limit->referee_max_power + 
										 power_limit->P_capEngry*(power_limit->capEnergy - power_limit->Min_capEnergy);
				power_limit->PowerState_Set = BAT;
			}else{
				power_limit->set_power = power_limit->HalfCAP_Power;
			}
		}else{
			if (power_limit->remainEnergy < power_limit->Min_remainEnergy*1.2){	//最小阈值之前都是恒功率		
				power_limit->set_power = power_limit->referee_max_power + 
										 power_limit->P_remainEngry*(power_limit->remainEnergy - power_limit->Min_remainEnergy);	
				power_limit->PowerState_Set = CAP;
			}else{
				power_limit->set_power = power_limit->HalfCAP_Power;	
			}			
		}
			break;
			
		case BAT:
		default:
			if (power_limit->remainEnergy < power_limit->Min_remainEnergy*1.3){	//最小阈值之前都是恒功率		
				power_limit->set_power = power_limit->referee_max_power + 
										 power_limit->P_remainEngry*(power_limit->remainEnergy - power_limit->Min_remainEnergy);	
			}else{
				power_limit->set_power = power_limit->referee_max_power + 60;	
			}	
		break;
	}
}
/**********************************************************************************************************
*函 数 名: set_powersate
*功能说明: 设置能量状态
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
int set_powersate(Power_Limit_type *power_limit,SuperPower *super_power,int control_powerstate){
	
	if(control_powerstate == HalfCAP)
    {
        if(super_power->CAP_AD_H+0.5 < super_power->actual_vol && power_limit->Min_capVol-0.5 < super_power->actual_vol)//当前电压大于飞坡所需最小的电容电压才能开启
			super_power->PowerState_Set = power_limit->PowerState_Set;//发送能量系统切换信号
        else
			super_power->PowerState_Set = BAT;
		
		if(super_power->actual_vol < power_limit->Min_capVol-1){
			power_limit->LowVol_halfcap = 1;
		}else{
			power_limit->LowVol_halfcap = 0;
		}
    }
    
	else{
		if(control_powerstate == BAT)			//操作手关闭超级电容
		{
			if(super_power->PowerState == CAP)		//刚从电容切回
			{
				if(power_limit->remainEnergy > 20)		//缓存能量够用才切回
					{
						super_power->PowerState_Set = BAT;
					}
			}
		}

	//AD_L AD_H有差值：刚开启电容时，由于电容有内阻，瞬间会有4V压降
		else if(control_powerstate == CAP)				//操作手开启超级电容
		{
			if(super_power->actual_vol<super_power->CAP_AD_L)  	//电容实际电压<10V
			{
				super_power->PowerState_Set = BAT;
			}
			else if(super_power->actual_vol>super_power->CAP_AD_H)  
			{
				super_power->PowerState_Set = CAP;
			}
		}
	}
}
/**********************************************************************************************************
*函 数 名: PowerLimit
*功能说明: 功率限制
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
/*
*calfromPID
*从pid控制算法中反解转速和电流映射关系常数M
*/
void calfromPID(Power_Limit_type *power_limit){
	power_limit->k_s_min = 1.0;
	for(int i=0;i<4;i++){
		Pid_Typedef P = (power_limit->pidChassisWheelSpeed[i]);
		short speed_now = power_limit->Speed_Now[i];
		
		float PreError =  P.SetPoint - speed_now;
		float SumError_last = P.SumError;
		float SumError = SumError_last;
		if(PreError > -P.ErrorMax && PreError < P.ErrorMax)
		{
			SumError += (P.PreError + P.LastError)/2;   //梯形积分提高精度
		}
		
		if(ABS(SumError) >= P.IMax || ABS(PreError) > P.ErrorMax)//不积分的情况
		{
			power_limit->M[i] = (-P.P*speed_now + P.I*SumError_last)*RM3508_CURRENT_RATIO;
			power_limit->K[i] = P.P*RM3508_CURRENT_RATIO;
		}
		else
		{
			power_limit->M[i] = (-P.P*speed_now + P.I*(SumError_last+0.5*(-speed_now+P.LastError)))*RM3508_CURRENT_RATIO;
			power_limit->K[i] = (P.P + P.I/2)*RM3508_CURRENT_RATIO;
		}
		
		
		double k_s = 1.0;
		double K_W = power_limit->K[i] * P.SetPoint;
		double current = K_W + power_limit->M[i];
		if(ABS(current) > power_limit->pidChassisWheelSpeed[i].OutMax*RM3508_CURRENT_RATIO){
			double l = current>0?1:-1;
			power_limit->k_s[i] = LIMIT_MAX_MIN((l*power_limit->pidChassisWheelSpeed[i].OutMax*RM3508_CURRENT_RATIO - power_limit->M[i])/K_W,1.0,0.0);
			if(power_limit->k_s[i] < power_limit->k_s_min){
				power_limit->k_s_min = power_limit->k_s[i];
			}
		}
	}
	//power_limit->k_s_min = 1.0;
	for(int i=0;i<4;i++){
		power_limit->pidChassisWheelSpeed[i].SetPoint *= power_limit->k_s_min;//最大值越界衰减
	}
	
}

void predict_old(Power_Limit_type *power_limit){
    // 发送期望电流之后产生的功率预测
	double i_2=0, w_i=0, a, b, c;
	i_2=0,w_i=0;
	for(int i=0;i<4;i++){
		
		i_2+=(powerlimit.Current_set[i]*RM3508_CURRENT_RATIO * powerlimit.Current_set[i]*RM3508_CURRENT_RATIO);
		w_i+=(powerlimit.Current_set[i]*RM3508_CURRENT_RATIO * chassis.ChassisMotorCanReceive[i].RealSpeed);
	}
    
    a = RM3508_R * i_2;
	b = RM3508_K * w_i;
    power_limit->predict_send_power_other = a + b + RM3508_P0;

}

void PowerLimit(Power_Limit_type *power_limit,int powerstate_cur){
	// 动态最大功率
	cal_powerSet(power_limit,powerstate_cur);
	
	calfromPID(power_limit);//求电流转速转换系数
    double i_2=0, w_i=0, a, b, c,i_2_6020=0,w_i_6020=0;

    // 实时功率预测
	for(int i=0;i<4;i++){
		i_2 += (chassis.ChassisMotorCanReceive[i].Current*RM3508_CURRENT_RATIO * chassis.ChassisMotorCanReceive[i].Current*RM3508_CURRENT_RATIO);
		w_i += (chassis.ChassisMotorCanReceive[i].Current*RM3508_CURRENT_RATIO * chassis.ChassisMotorCanReceive[i].RealSpeed);
	}
	
	#if Mecanum == 2 
	// 实时舵角电机功率
	for(int i=0;i<4;i++){
		i_2_6020 += (chassis.SteerMotorsCanReceive[i].Current*RM6020_CURRENT_RATIO * chassis.SteerMotorsCanReceive[i].Current*RM6020_CURRENT_RATIO);
		w_i_6020 += (chassis.SteerMotorsCanReceive[i].Current*RM6020_CURRENT_RATIO * chassis.SteerMotorsCanReceive[i].RealSpeed);
	}
	power_limit->predict_p0 = RM3508_P0 + i_2_6020*RM3508_R + w_i_6020*power_limit->k_dynamic;
	#else
	power_limit->predict_p0 = RM3508_P0;
	#endif
	
	//保存数据,用matlab拟合
	power_limit->actual_i_2 = i_2;
	power_limit->actual_w_i = w_i;
	power_limit->actual_i_2_6020 = i_2_6020;
	power_limit->actual_w_i_6020 = w_i_6020;
	
	//电机数据拟合功率
  power_limit->predict_power = RM3508_R * (i_2) + power_limit->k_dynamic * (w_i) + power_limit->predict_p0;

	//更新k
	if(power_limit->predict_power>0){
		power_limit->k_dynamic = LowPass_SetK(power_limit->k_dynamic,
											(power_limit->actual_ina260_power - RM3508_P0 - RM3508_R * (i_2+i_2_6020))/(w_i+w_i_6020));
		power_limit->k_dynamic = LIMIT_MAX_MIN(power_limit->k_dynamic,RM3508_K_MAX,RM3508_K_MIN);
	}
		
	/***********************用期望值进行功率预测（电流期望跟随效果要好）*****************************************/
	a=0,b=0,c=0;
	for(int i=0;i<4;i++){
		if(powerlimit.Motor_lost[i]>=5)
			continue;
		Pid_Typedef P = (power_limit->pidChassisWheelSpeed[i]);
		a += RM3508_R * P.SetPoint * P.SetPoint * (power_limit->K[i]) * (power_limit->K[i]);
		b += P.SetPoint * (2*RM3508_R*power_limit->K[i]*power_limit->M[i] + power_limit->k_dynamic*power_limit->K[i]*power_limit->Speed_Now[i]);
		c += (RM3508_R*power_limit->M[i]*power_limit->M[i] + power_limit->k_dynamic*power_limit->M[i]*power_limit->Speed_Now[i]);
	}
	#if Mecanum == 2 
	//舵轮功率预测增加舵角电机
		for(int i=0;i<4;i++){
			i_2_6020 += (chassis.SteeringCurrentSend[i]*RM6020_CURRENT_RATIO * chassis.SteeringCurrentSend[i] *RM6020_CURRENT_RATIO);
			w_i_6020 += (chassis.SteeringCurrentSend[i]*RM6020_CURRENT_RATIO * chassis.SteerMotorsCanReceive[i].RealSpeed);
		}
		c += i_2_6020*RM3508_R + w_i_6020*power_limit->k_dynamic;
	#endif
	c += RM3508_P0;
	//当k_c=1时，预测功率如下
    power_limit->predict_send_power = a + b + c;//过滤转速预测异常数据

	/***********************功率预测后超功率则进行衰减*****************************************/
//	power_limit->set_power = 80;
//	power_limit->set_power -= power_limit->error_control;
	// 超功率衰减
	if(power_limit->predict_send_power > power_limit->set_power)
	{	
		// 模型衰减计算
		if(b*b < 4*(c - power_limit->set_power)*a){//   b方小于4ac：没有解
			power_limit->k_c = LIMIT_MAX_MIN(-b/(2*a) , 1.0,0.0);//   -b/(2a)
		}else{
			float32_t sqrt_result;
			arm_sqrt_f32(b * b - 4*(c - power_limit->set_power)*a,&sqrt_result);
			power_limit->k_c = LIMIT_MAX_MIN((-b + sqrt_result) / 2 / a,1.0 ,0.0);
		}
		
		//衰减
		for(int i=0;i<4;i++){
			power_limit->pidChassisWheelSpeed[i].SetPoint *= power_limit->k_c;//衰减
		}
	}
	
	power_limit->set_superpower = LIMIT_MAX_MIN(power_limit->set_power - (a*power_limit->k_c*power_limit->k_c+b*power_limit->k_c+c),22*8, 0);
	
	//PI控制器计算目标电流
	for(int i=0;i<4;i++){
		power_limit->Current_set[i] = PID_Calc(&(power_limit->pidChassisWheelSpeed[i]),power_limit->Speed_Now[i]);
	}
}

/**********************************************************************************************************
*函 数 名: Chassis_Current_send()
*功能说明: 底盘电流发送函数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/

#if Mecanum == 2  //舵轮
void Chassis_Current_send(){
	ChassisCan1Send(chassis.WheelCurrentSend[0],chassis.WheelCurrentSend[1],chassis.WheelCurrentSend[2],chassis.WheelCurrentSend[3]);
	SteerCan1Send(chassis.SteeringVolSend[0],chassis.SteeringVolSend[1],chassis.SteeringVolSend[2],chassis.SteeringVolSend[3]);
}
#else
void Chassis_Current_send(void){
	ChassisCan1Send(chassis.WheelCurrentSend[0],chassis.WheelCurrentSend[1],chassis.WheelCurrentSend[2],chassis.WheelCurrentSend[3]);
}
#endif

/**********************************************************************************************************
*函 数 名: Chassis_CurrentPid_Cal
*功能说明: 底盘操作
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Chassis_Loop_Cal(void)
{
	int i=0;
	Chassis_Speed_Cal();//根据xyw向速度计算目标速度值,计算xyw三轴速度
	
	motion_decomposition(chassis.Velocity_vectors);//运动分解，计算每个轮子的setpoint
	
	
	//功率限制，计算每个轮子的电流值
	if(F405.Chassis_Flag == Chassis_Powerdown_Mode)
	{
				
		for(i=0;i<4;i++)
		{
			chassis.WheelCurrentSend[i] = 0;
			chassis.SteeringVolSend[i] = 0;
		}
		powerlimit.set_superpower = JudgeReceive.MaxPower;//全部充电
	}
	else
	{

		for(int i=0;i<4;i++){
//			powerlimit.pidChassisWheelSpeed[i].SetPoint = chassis.Velocity_vectors[i].v;
//			powerlimit.Speed_Now[i] = chassis.ChassisMotorCanReceive[i].RealSpeed;
			if((chassis.ChassisMotorCanReceive[i].Current != powerlimit.Current_Last[i]))
			{
				powerlimit.Motor_lost[i] = 0;
				powerlimit.pidChassisWheelSpeed[i].SetPoint = chassis.Velocity_vectors[i].v;
			}
			else{
				powerlimit.Motor_lost[i] ++;
				powerlimit.pidChassisWheelSpeed[i].SetPoint = chassis.ChassisMotorCanReceive[i].RealSpeed;
				if(powerlimit.Motor_lost[i]>=5){
					if(i==0)SD_saver.number_error[Drop_ChassisMotor1]++;
					if(i==1)SD_saver.number_error[Drop_ChassisMotor2]++;
					if(i==2)SD_saver.number_error[Drop_ChassisMotor3]++;
					if(i==3)SD_saver.number_error[Drop_ChassisMotor4]++;				
				}
			}	
			powerlimit.Current_Last[i] = chassis.ChassisMotorCanReceive[i].Current;
			powerlimit.Speed_Now[i] = chassis.ChassisMotorCanReceive[i].RealSpeed;
		}		
		
		//输入数据
		powerlimit.actual_ina260_power = INA260_1.Power/1000;
		if(JudgeReceive.MaxPower<150 && JudgeReceive.MaxPower>0){
			powerlimit.referee_max_power = JudgeReceive.MaxPower;
		}else{
			powerlimit.referee_max_power = 200;
		}
		powerlimit.remainEnergy = JudgeReceive.remainEnergy;
		powerlimit.capEnergy = 0.5*CAP_C*(superpower.actual_vol*superpower.actual_vol);

		if(F405.SuperPowerLimit==CAP){
			powerlimit.PowerState_control = CAP;
		}else{
			powerlimit.PowerState_control = HalfCAP;
		}		

//        powerlimit.PowerState_control = BAT;
		
		PowerLimit(&powerlimit,superpower.PowerState);//调整电流期望值 并确定好期望的能量系统目标
		
		set_powersate(&powerlimit,&superpower,powerlimit.PowerState_control);//设置真正电容控制信号superpower.PowerState_Set
	
		
		for(int i=0;i<4;i++){
			chassis.WheelCurrentSend[i] = powerlimit.Current_set[i];
		}
	}
	
	//发送电流
	Chassis_Current_send();
	
}

/**********************************************************************************************************
*函 数 名: BuildF105
*功能说明: 构建要传给上层板的F105结构体
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void BuildF105(void)
{
	
	if(JudgeReceive.robot_id < 10)
		F105.Sendmessage.RobotRed = 1;
	else
		F105.Sendmessage.RobotRed = 0;			//0为蓝色，1为红色
	switch(JudgeReceive.BulletSpeedMax17)
	{
		case 15:
		{
			F105.Sendmessage.BulletSpeedLevel = 0;
			break;
		}
		case 18:
		{
			F105.Sendmessage.BulletSpeedLevel = 1;
			break;
		}
		case 30:
		{
			F105.Sendmessage.BulletSpeedLevel = 2;
			break;
		}
		default:
		{
			F105.Sendmessage.BulletSpeedLevel = 0;
			break;
		}
	}
	F105.Sendmessage.shooterHeat17 = JudgeReceive.shooterHeat17;
	F105.Sendmessage.HeatMax17 = JudgeReceive.HeatMax17;
	F105.Sendmessage.HeatCool17 = (unsigned char)(JudgeReceive.HeatCool17);
	F105.Sendmessage.RobotLevel = JudgeReceive.RobotLevel;
	
	F105.ChassisSpeedw=0.026f*(chassis.ChassisMotorCanReceive[0].RealSpeed+chassis.ChassisMotorCanReceive[1].RealSpeed+chassis.ChassisMotorCanReceive[2].RealSpeed+chassis.ChassisMotorCanReceive[3].RealSpeed);
}



/**********************************************************************************************************
*函 数 名: Chassis_Init
*功能说明: 底盘向运动参数初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Chassis_Init(void)
{	
	ZeroCheck_Init();
	powerlimit.k_dynamic = RM3508_K;
#if Robot_ID  == 46  
	powerlimit.error_control = 5; //多控了5w
	powerlimit.No_limited_Power = 250; 		//W 主动电容无限制功率
	
	powerlimit.Min_remainEnergy = 25;  		//15J 不修改
	powerlimit.Add_HalfCAP_Power = 10;			//W 被动电容多跑的功率
	powerlimit.Min_capVol = 16.5;			//V  被动电容保留能量用来飞坡
	
	//计算最小电容能量
	powerlimit.Min_capEnergy = 0.5*CAP_C*(powerlimit.Min_capVol*powerlimit.Min_capVol);
#else
	powerlimit.error_control = 5; //多控了5W
	powerlimit.No_limited_Power = 300; 		//W 主动电容无限制功率
	
	powerlimit.Min_remainEnergy = 20;  		//15J 不修改
	powerlimit.Add_HalfCAP_Power = 10;			//W 被动电容多跑的功率
	powerlimit.Min_capVol = 17.0;			//V  被动电容保留能量用来飞坡
	
	
	//计算最小电容能量
	powerlimit.Min_capEnergy = 0.5*CAP_C*(powerlimit.Min_capVol*powerlimit.Min_capVol);	
#endif
	short i=0;
	for(i=0;i<4;i++)
	{
		//功率限制限制车轮转速
		powerlimit.pidChassisWheelSpeed[i].deadband=0.0;
		powerlimit.pidChassisWheelSpeed[i].P = 20.0f;
		powerlimit.pidChassisWheelSpeed[i].I = 1.0f;		
		powerlimit.pidChassisWheelSpeed[i].D = 0.0f;
		powerlimit.pidChassisWheelSpeed[i].ErrorMax = 1000.0f;
		powerlimit.pidChassisWheelSpeed[i].IMax = 2500.0f;
		powerlimit.pidChassisWheelSpeed[i].SetPoint = 0.0f;	
		powerlimit.pidChassisWheelSpeed[i].OutMax = 16000.0f;
		
		//舵向电机
		chassis.PIDSteeringAnglePosition[i].deadband=2.5;
		chassis.PIDSteeringAnglePosition[i].P=7.5f;
		chassis.PIDSteeringAnglePosition[i].I=0.0f;
		chassis.PIDSteeringAnglePosition[i].D=0.0f;
		chassis.PIDSteeringAnglePosition[i].IMax=1000.0f;
		chassis.PIDSteeringAnglePosition[i].SetPoint=0.0f;
		chassis.PIDSteeringAnglePosition[i].ErrorMax = 1000.0f;
		chassis.PIDSteeringAnglePosition[i].OutMax=4000.0f;
		
		chassis.PIDSteeringAngleSpeed[i].deadband=0.0;
		chassis.PIDSteeringAngleSpeed[i].P=120.0f;
		chassis.PIDSteeringAngleSpeed[i].I=0.2f; 
		chassis.PIDSteeringAngleSpeed[i].D=0.0f;
		chassis.PIDSteeringAngleSpeed[i].IMax=16000.0f;
		chassis.PIDSteeringAngleSpeed[i].SetPoint=0.0f;
		chassis.PIDSteeringAngleSpeed[i].ErrorMax = 8000;
		chassis.PIDSteeringAngleSpeed[i].OutMax=GM6020_MAX_VOLTAGE;
		
		chassis.Velocity_vectors[i].v = 0;
		chassis.Velocity_vectors_last[i].v = 0;
		//ChassisSteeringPIDRUN_Send(Wheel);
	}
	
	
	chassis.k_xy = 3.0f;
	chassis.k_w = 1.5f;
	chassis.Steeringwheel_Mode = Stright;
	chassis.sgn = 1.0;
	
	//初始角度
	chassis.SteeringAnglePosition_init[0] = 5147;
	chassis.SteeringAnglePosition_init[1] = 3061;
	chassis.SteeringAnglePosition_init[2] = 4921;
	chassis.SteeringAnglePosition_init[3] = 326;
}
/**********************************************************************************************************
*函 数 名: Chassis_task
*功能说明: 底盘任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
uint32_t Chassis_high_water;

extern uint8_t JudgeReveice_Flag;
//extern TaskHandle_t JudgeReceiveTask_Handler; //任务句柄
extern TaskHandle_t User_Tasks[TASK_NUM];
uint32_t testtask=0;
void Chassis_task(void *pvParameters)
{
	portTickType xLastWakeTime;
#if Mecanum == 2
	const portTickType xFrequency = 2;
#else
	const portTickType xFrequency = 1;
#endif
	Chassis_Init();
	
	vTaskDelay(100);
	while (1) {
		xLastWakeTime = xTaskGetTickCount();
		
		if(JudgeReveice_Flag)
		{
		 xTaskNotifyGive(User_Tasks[JUDGERECEIVE_TASK]);
		}
		
		//电容充放电控制
		if(JudgeReceive.remainEnergy<20)
		{
		Charge_Off;
		ChargeState = ChargeOff ;
		}
		else
		{
			Charge_On;
			ChargeState = ChargeOn;
		}	
		//功率限制
		Chassis_Loop_Cal();
		BuildF105();
		Can2Send0(&F105);
		
		VOFA_Send();

		IWDG_Feed();//喂狗		
		vTaskDelayUntil(&xLastWakeTime,xFrequency); 
		 
	#if INCLUDE_uxTaskGetStackHighWaterMark
		Chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
	#endif
	}
}
