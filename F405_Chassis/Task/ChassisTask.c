/**********************************************************************************************************
 * @�ļ�     ChassisTask.c
 * @˵��     �µ��̿���+�¹�������
 * @�汾  	 V1.0
 * @����     ���κ�
 * @����     2023.7.6
**********************************************************************************************************/
#include "main.h"

#define CAP_MAX_W      7000
#define Rand_S         0.5f   //���ڳ���
#define RandThreshold  0.4f   //ֱ��ƫ��
#define RANDA          1.3f   //���ҷ�ֵ
/*----------------------------------�ṹ��-----------------------------*/
F105_Typedef F105;//������Ϣ�����ڰ��ͨѶ
Power_Limit_type powerlimit;
chassis_type chassis={0};//���̿��ƽṹ��
/*----------------------------------�ⲿ����---------------------------*/
extern INA260 INA260_1,INA260_2;
extern SuperPower superpower;;
extern JudgeReceive_t JudgeReceive;
extern ChassisSpeed_t chassisSpeed;
extern F405_typedef F405;
extern enum CHARGESTATE_Typedef ChargeState;
extern SD_Saver_t SD_saver;
/**********************************************************************************************************
*�� �� ��: motion_decomposition
*����˵��: �˶��ֽ�
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
float test_x,test_y,test_w;
char filter_en=1;
char test_cnt[4];
#define  SetUP_T  0.99f

 void motion_decomposition(Velocity_vector_type Velocity_vector[4])
{
#if Mecanum == 1  //����
	Velocity_vector[0].v = LowPass_SetChassis(Velocity_vector[0].v,
										chassis.k_xy*(+chassisSpeed.carSpeedy+(+chassisSpeed.carSpeedx))-chassis.carSpeedw);
	
	Velocity_vector[1].v = LowPass_SetChassis(Velocity_vector[1].v,
										chassis.k_xy*(+chassisSpeed.carSpeedy+(-chassisSpeed.carSpeedx))-chassis.carSpeedw);
	
	Velocity_vector[2].v = LowPass_SetChassis(Velocity_vector[2].v,
										chassis.k_xy*(-chassisSpeed.carSpeedy+(+chassisSpeed.carSpeedx))-chassis.carSpeedw);
	
	Velocity_vector[3].v = LowPass_SetChassis(Velocity_vector[3].v,
										chassis.k_xy*(-chassisSpeed.carSpeedy+(-chassisSpeed.carSpeedx))-chassis.carSpeedw);

#elif Mecanum == 0   //ȫ����
/*ֱ����*/
//	LowPass_SetChassis(&pidChassisWheelSpeed[0].SetPoint,chassis.sgn*(-k_xy*(-chassis.carSpeedy+(+chassis.carSpeedx))-chassis.carSpeedw));
//	LowPass_SetChassis(&pidChassisWheelSpeed[1].SetPoint,chassis.sgn*(-k_xy*(-chassis.carSpeedy+(-chassis.carSpeedx))-chassis.carSpeedw));
//	LowPass_SetChassis(&pidChassisWheelSpeed[2].SetPoint,chassis.sgn*(-k_xy*(+chassis.carSpeedy+(-chassis.carSpeedx))-chassis.carSpeedw));
//	LowPass_SetChassis(&pidChassisWheelSpeed[3].SetPoint,chassis.sgn*(-k_xy*(+chassis.carSpeedy+(+chassis.carSpeedx))-chassis.carSpeedw));

/*б����*/	
	Velocity_vector[0].v = LowPass_SetChassis(Velocity_vector[0].v,
										chassis.sgn*(-chassis.k_xy*(-chassisSpeed.carSpeedy)-chassis.carSpeedw));
	
	Velocity_vector[1].v = LowPass_SetChassis(Velocity_vector[1].v,
										chassis.sgn*(-chassis.k_xy*(-chassisSpeed.carSpeedx)-chassis.carSpeedw);
	
	Velocity_vector[2].v = LowPass_SetChassis(Velocity_vector[2].v,
										chassis.sgn*(-chassis.k_xy*(+chassisSpeed.carSpeedy)-chassis.carSpeedw));
	
	Velocity_vector[3].v = LowPass_SetChassis(Velocity_vector[3].v,
										chassis.sgn*(-chassis.k_xy*(+chassisSpeed.carSpeedx)-chassis.carSpeedw));
										

#elif Mecanum == 2   //����
	//�ٶȽ���
	ZeroCheck_cal();
	//�Ƕȴ��� 
	float max_speed=16000;
	float car_yaw = R2DEG(atan2(chassisSpeed.carSpeedy, chassisSpeed.carSpeedx)) - chassis.yaw_diff_rec;
	float car_V = 0;
	arm_sqrt_f32(chassisSpeed.carSpeedy * chassisSpeed.carSpeedy + chassisSpeed.carSpeedx * chassisSpeed.carSpeedx,&car_V);
	float flag = chassis.carSpeedw>0?1:-1;//������ת�������С���ݲ���
	switch(chassis.Steeringwheel_Mode){
		case Stright:
			chassis.car_yaw = car_yaw;
			chassis.carSpeedw = LIMIT_MAX_MIN(chassisSpeed.carSpeedw*10,max_speed,-max_speed);
			chassis.car_V = LIMIT_MAX_MIN(car_V*10,max_speed,-max_speed);
		break;
		case Small_top_avoid:
			chassis.car_yaw = car_yaw + flag*25;//�����ƶ�ƫ��
			chassis.carSpeedw = LIMIT_MAX_MIN(chassisSpeed.carSpeedw*10,max_speed,-max_speed);//����С����
			chassis.car_V = LIMIT_MAX_MIN(car_V*10,max_speed*0.7,-max_speed*0.7);
//			if(ABS(car_V)>100){
//				chassis.carSpeedw = LIMIT_MAX_MIN(chassisSpeed.carSpeedw*10,max_speed,-max_speed);//����С����
//				chassis.car_V = LIMIT_MAX_MIN(car_V*10,max_speed*0.7,-max_speed*0.7);
//			}else{
//				float flag = chassisSpeed.carSpeedw>0?1:-1;
//				chassis.carSpeedw = (5000*arm_cos_f32(2*DEG2R(chassis.yaw_diff_rec+8))+6000)*flag;
//				chassis.car_V = LIMIT_MAX_MIN(car_V*10,4000,-4000);
//			}
		break;
		case Small_top_move:
			chassis.car_yaw = car_yaw + flag*20;//�����ƶ�ƫ��
			chassis.carSpeedw = LIMIT_MAX_MIN(chassisSpeed.carSpeedw*10,max_speed*0.5,-max_speed*0.5);
			chassis.car_V = LIMIT_MAX_MIN(car_V*10,max_speed,-max_speed);
		break;
		case Recovery://С���ݽ�������
			if(ABS(chassisSpeed.carSpeedw)<500&&(ABS(chassis.yaw_diff_rec-0)<5 || ABS(chassis.yaw_diff_rec-90)<5 || ABS(chassis.yaw_diff_rec-180)<5 || ABS(chassis.yaw_diff_rec+90)<5 || ABS(chassis.yaw_diff_rec+180)<5)){
				chassis.Steeringwheel_Mode = Stright;
			}
		break;
	}
	controlSteering_wheel(&chassis, chassis.car_yaw, chassis.car_V, chassis.carSpeedw);	
	ChassisSteer_cal(&chassis);//�����ǵ���
#endif 
}
/**********************************************************************************************************
*�� �� ��: cal_speed_now()
*����˵��: �ٶ����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void cal_speed_now(int *speedx, int *speedy, int *speedz){
#if Mecanum == 1  //����
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
*�� �� ��: Add_vector,����Ƕ�Ϊ�ȣ�����Ϊ��
*����˵��: �ٶ��������,��������Ϊ-180��180�Ƕ�
*��    ��: ��������
*�� �� ֵ: ��
**********************************************************************************************************/
Velocity_vector_type Add_vector(Velocity_vector_type *a, Velocity_vector_type *b)//ʵ������a+b
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
*�� �� ��: calculateShortestDistance
*����˵��: ������0-360���λ�õľ��Ծ��룬�����ڶ��ֶ��Ŀ��������ת���룬�ú�����õ��ƶ��ǶȻ��ǵĶ�ǵ�������ת������90��
*��    ��: ��ǰλ�ú�Ŀ��λ�ã��Լ��Ƿ���
*�� �� ֵ: ����ƶ�������Ƿ��������ת����ú������һ���ܱ�֤������90��
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
	//�������������붼Ҫ����90
	if(ABS(shortest_distance)>90){
		//��ת
		float flipped_yaw_now = yaw_now_360 + 180;
		
		if (flipped_yaw_now >= 360) {
			flipped_yaw_now -= 360;
		}
		if(clockwise_distance > counter_clockwise_distance)//δ��תǰ������ڷ�����ת��ȡ����
		{
			reverse_distance = fmodf((yaw_set_360 - flipped_yaw_now + 360), 360);//���������
		}else{
			reverse_distance = -fmodf(flipped_yaw_now - yaw_set_360 + 360, 360);//�������
		}
		*reverflag = -1.0;
        shortest_distance = reverse_distance;
	}

    return shortest_distance;
}

/**********************************************************************************************************
*�� �� ��: control_wheel
*����˵��: ���ֵ��̲ٿ� w_speed��ת�ٶȣ�move_speedƽ���ٶ�
*��    ��: �ⲿ�ӿ�����ٶȷֽ�,������ģʽ�� //Gimbal��ʾ��̨����̳���нǣ�С����ģʽ�±�ʾ��̨�뵱ǰ�ӽǵļн�
*�� �� ֵ: ��
**********************************************************************************************************/
void controlSteering_wheel(chassis_type *chassis_, float car_yaw, float car_V, float w_speed)
{	
	Velocity_vector_type Wheel_Temp[4];//���ڿ����ĸ������˶������˶��ϳ�ʱ�����ת�ٶ�����

	for (int i = 0; i < 4; i++) {//��¼�ϴ��ٶ�
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
		//�ٶȸ�ֵ
		Wheel_Temp[0].v = +w_speed;
		Wheel_Temp[1].v = -w_speed;
		Wheel_Temp[2].v = -w_speed;
		Wheel_Temp[3].v = +w_speed;

		//�������Ӵ�ֱ��������ĽǶȳ���,��������ϵ
		Wheel_Temp[0].yaw =   + SMALLTOP_CIRCLE;
		Wheel_Temp[1].yaw =   - SMALLTOP_CIRCLE;
		Wheel_Temp[2].yaw =   + SMALLTOP_CIRCLE;
		Wheel_Temp[3].yaw =   - SMALLTOP_CIRCLE;				
		
		//�ֱ��ڵ�������ϵ��ֱ���봹ֱ������ӵõ���ʸ��
		for (int u = 0; u < 4; u++) 
		{
			chassis.Velocity_vectors[u] = Add_vector(&(chassis.Velocity_vectors[u]), &(Wheel_Temp[u]));
		}
	}
	//�Ƕ����ƣ�ת�Ǵ���90����ת
	float yaw_now_360=0,yaw_set_360=0,yaw_diff_360=0;
	for (int i = 0; i < 4; i++) {
		//��ǰֵ
		yaw_now_360 = (chassis.SteerMotorsCanReceive[i].Angle - chassis.SteeringAnglePosition_init[i])/8192.0f*360.0f;
		if(yaw_now_360<0) yaw_now_360+=360;
		//Ŀ��ֵ
		yaw_set_360 = fmodf(chassis.Velocity_vectors[i].yaw, 360);
		if(yaw_set_360<0) yaw_set_360+=360;
		float reverflag = 1.0;
		yaw_diff_360 = calculateShortestDistance(yaw_now_360, yaw_set_360, &reverflag);//yaw_diff_360[i]������90��
		chassis.Velocity_vectors[i].yaw = chassis.SteerMotorsDecode[i].MotorTransAngle + yaw_diff_360;
		chassis.Velocity_vectors[i].v *= reverflag;
		
		//�����ת�Ƕ�diff�������ת����һ��˥������������cos3�η�����
		float cos_k = arm_cos_f32(DEG2R(yaw_diff_360));
		chassis.Velocity_vectors[i].v *= (cos_k*cos_k*cos_k);
	}
	
	//�����װ
	chassis.Velocity_vectors[0].v = -chassis.Velocity_vectors[0].v;
	chassis.Velocity_vectors[3].v = -chassis.Velocity_vectors[3].v;
	
	for (int i = 0; i < 4; i++) {//Ŀ���ͨ
		chassis.Velocity_vectors[i].v = LowPass_SetChassis(chassis.Velocity_vectors_last[i].v,
															chassis.Velocity_vectors[i].v);
		
		chassis.Velocity_vectors[i].yaw = LowPass_SetSteer(chassis.Velocity_vectors_last[i].yaw,
															chassis.Velocity_vectors[i].yaw);
	}
}

/**********************************************************************************************************
*�� �� ��: ChassisSteer_cal
*����˵��: ��ǵ���pid����
*��    ��: 
*�� �� ֵ: ��
**********************************************************************************************************/
void ChassisSteer_cal(chassis_type *chassis_)
{
	short i=0;
	/**************������******************/

	//������λ�û�
	for(i = 0;i < 4;i++)
	{
		chassis_->PIDSteeringAnglePosition[i].SetPoint = chassis_->Velocity_vectors[i].yaw;//λ�û�setpoint����
		chassis_->PIDSteeringAngleSpeed[i].SetPoint = PID_Calc(&chassis_->PIDSteeringAnglePosition[i],chassis_->SteerMotorsDecode[i].MotorTransAngle);	//�ٶȻ�setpoint����
		chassis_->SteeringVolSend[i] = PID_Calc(&chassis_->PIDSteeringAngleSpeed[i],chassis_->SteerMotorsDecode[i].RealSpeed);	//����ֵ
		
		//ͨ����ѹ������ϵ���Ӧ�ĵ�������ֵ
		chassis.SteeringCurrentSend[i] = Vol2Current(chassis_->SteeringVolSend[i],  chassis_->SteerMotorsDecode[i].RealSpeed);
		
		//����
		if(chassis.SteerMotorsCanReceive[i].temp > 90){
			chassis_->SteeringVolSend[0] = 0;
			chassis_->SteeringVolSend[1] = 0;
			chassis_->SteeringVolSend[2] = 0;
			chassis_->SteeringVolSend[3] = 0;
		}
	}
}

/**********************************************************************************************************
*�� �� ��: Chassis_Speed_Cal
*����˵��: ����xyw���ٶȼ���Ŀ���ٶ�ֵ
*��    ��: ��
*�� �� ֵ: ��
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
	
	//xy���ٶ���w���ٶ����
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
//ȫ���ֺ������ķ��
void Chassis_Speed_Cal(void)
{
	static short Angular_Velocity;
	float rotation_lim=1.0f;	
	
	//xy���ٶ���w���ٶ����
	switch(F405.Chassis_Flag)
	{
		case Chassis_Powerdown_Mode:
			chassis.k_xy = 0;
			chassis.carSpeedw = 0;
		break;
		
		case Chassis_Act_Mode:
		case Chassis_Jump_Mode:
			
		if(ABS(chassisSpeed.carSpeedw) > 1000 && ((ABS(chassisSpeed.carSpeedx)>500) || (ABS(chassisSpeed.carSpeedy)) >500)) //������ʻ�д��ת�����
        {
            chassis.k_xy = 2.0f;
            chassis.k_w = 1.3f;
        }
        else if(ABS(chassisSpeed.carSpeedw) > 1000)        //ԭ��ת
        {
            chassis.k_xy = 3.0f;
            chassis.k_w = 1.0f;
        }
        else
        {
            chassis.k_xy = 3.0f;
            chassis.k_w = 1.0f;
        }
//		else if((ABS(chassis.carSpeedx)>500) && (ABS(chassis.carSpeedy) >500))  //���½�Ϊƽ�ƻ�С��ת�����
//        {
//            k_xy = 2.5f;    //б����
//            k_w = 1.0;
//        }
//        else if((ABS(chassis.carSpeedx)>500) && (ABS(chassis.carSpeedy) <500))
//        {
//            k_xy = 3.0f;       //ֱ����
//            k_w = 1.0f;
//        }		
//		else if((ABS(chassis.carSpeedx) <500) && (ABS(chassis.carSpeedy)>500))
//        {
//            k_xy =3.0f;     //������
//            k_w = 1.0f;
//        }
				
		
	   chassis.carSpeedw = chassisSpeed.carSpeedw;

	 
		break;
		
		case Chassis_SelfProtect_Mode:
		{
			//���ٴ���
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
				//����С����
					chassis.carSpeedw = LIMIT_MAX_MIN(chassisSpeed.carSpeedw,CAP_MAX_W,-CAP_MAX_W);
//				//        ����С����(������������)
//				rand_w = (RandThreshold+(1-RandThreshold)*rand_A)*CAP_MAX_W;
//				chassis.carSpeedw = LIMIT_MAX_MIN(chassis.carSpeedw, rotation_lim*(rand_w), -rotation_lim*(rand_w));
//					
				}
				else 
				{	chassis.Self_Protect_Limit = (powerlimit.referee_max_power - 60)*30+5000;//��60w�������ӳ�䵽5000-8000
					if(powerlimit.referee_max_power < 120)
					{
			//        ����С����					
							chassis.carSpeedw = LIMIT_MAX_MIN(chassisSpeed.carSpeedw, rotation_lim*chassis.Self_Protect_Limit, -rotation_lim*chassis.Self_Protect_Limit);
					}
					else
					{
			//        ����С����
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
*�� �� ��: cal_powerSet
*����˵��: ���㵱ǰ��������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void cal_powerSet(Power_Limit_type *power_limit,int powerstate_cur){
	
	// ���ݵ�ǰ����ѡ��Ŀ�깦��
	powerlimit.P_capEngry = power_limit->referee_max_power/(0.5*22.0*22.0*CAP_C - powerlimit.Min_capEnergy) + 0.1;//������ݿز�����͵�ѹ��Ҳ�����ֵ������0.15����
	powerlimit.P_remainEngry = power_limit->referee_max_power/(60 - power_limit->Min_remainEnergy);//������������ϵ����Ҳ�����ֵ������1.8����
	
	// ���ݵ�ǰ����ѡ��Ŀ�깦��
    if(power_limit->remainEnergy >= 40.0f && ABS(power_limit->Speed_Now[0])<3000)    //ʣ����������40J���൱���ж�Ϊ�𲽣�ȫ������
    {
        power_limit->HalfCAP_Power = power_limit->referee_max_power + power_limit->Add_HalfCAP_Power + 100;
    }
	else
			power_limit->HalfCAP_Power = power_limit->referee_max_power + power_limit->Add_HalfCAP_Power;
	
	// ��̬�����
	switch(power_limit->PowerState_control){
		case CAP:
			if(superpower.PowerState == CAP)
				power_limit->set_power = power_limit->No_limited_Power;
            else//ʵ�ʵ��
            {
                if (power_limit->remainEnergy < power_limit->Min_remainEnergy*1.3){	//��С��ֵ֮ǰ���Ǻ㹦��		
                power_limit->set_power = power_limit->referee_max_power + 
                                         power_limit->P_remainEngry*(power_limit->remainEnergy - power_limit->Min_remainEnergy);	
                }else{
                    power_limit->set_power = power_limit->referee_max_power + 60;	
                }
            }  
			break;
		case HalfCAP:
		if(powerstate_cur == CAP){
			if (power_limit->capEnergy < power_limit->Min_capEnergy*1.2){	//��С��ֵ֮ǰ���Ǻ㹦��		
				power_limit->set_power = power_limit->referee_max_power + 
										 power_limit->P_capEngry*(power_limit->capEnergy - power_limit->Min_capEnergy);
				power_limit->PowerState_Set = BAT;
			}else{
				power_limit->set_power = power_limit->HalfCAP_Power;
			}
		}else{
			if (power_limit->remainEnergy < power_limit->Min_remainEnergy*1.2){	//��С��ֵ֮ǰ���Ǻ㹦��		
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
			if (power_limit->remainEnergy < power_limit->Min_remainEnergy*1.3){	//��С��ֵ֮ǰ���Ǻ㹦��		
				power_limit->set_power = power_limit->referee_max_power + 
										 power_limit->P_remainEngry*(power_limit->remainEnergy - power_limit->Min_remainEnergy);	
			}else{
				power_limit->set_power = power_limit->referee_max_power + 60;	
			}	
		break;
	}
}
/**********************************************************************************************************
*�� �� ��: set_powersate
*����˵��: ��������״̬
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
int set_powersate(Power_Limit_type *power_limit,SuperPower *super_power,int control_powerstate){
	
	if(control_powerstate == HalfCAP)
    {
        if(super_power->CAP_AD_H+0.5 < super_power->actual_vol && power_limit->Min_capVol-0.5 < super_power->actual_vol)//��ǰ��ѹ���ڷ���������С�ĵ��ݵ�ѹ���ܿ���
			super_power->PowerState_Set = power_limit->PowerState_Set;//��������ϵͳ�л��ź�
        else
			super_power->PowerState_Set = BAT;
		
		if(super_power->actual_vol < power_limit->Min_capVol-1){
			power_limit->LowVol_halfcap = 1;
		}else{
			power_limit->LowVol_halfcap = 0;
		}
    }
    
	else{
		if(control_powerstate == BAT)			//�����ֹرճ�������
		{
			if(super_power->PowerState == CAP)		//�մӵ����л�
			{
				if(power_limit->remainEnergy > 20)		//�����������ò��л�
					{
						super_power->PowerState_Set = BAT;
					}
			}
		}

	//AD_L AD_H�в�ֵ���տ�������ʱ�����ڵ��������裬˲�����4Vѹ��
		else if(control_powerstate == CAP)				//�����ֿ�����������
		{
			if(super_power->actual_vol<super_power->CAP_AD_L)  	//����ʵ�ʵ�ѹ<10V
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
*�� �� ��: PowerLimit
*����˵��: ��������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
/*
*calfromPID
*��pid�����㷨�з���ת�ٺ͵���ӳ���ϵ����M
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
			SumError += (P.PreError + P.LastError)/2;   //���λ�����߾���
		}
		
		if(ABS(SumError) >= P.IMax || ABS(PreError) > P.ErrorMax)//�����ֵ����
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
		power_limit->pidChassisWheelSpeed[i].SetPoint *= power_limit->k_s_min;//���ֵԽ��˥��
	}
	
}

void predict_old(Power_Limit_type *power_limit){
    // ������������֮������Ĺ���Ԥ��
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
	// ��̬�����
	cal_powerSet(power_limit,powerstate_cur);
	
	calfromPID(power_limit);//�����ת��ת��ϵ��
    double i_2=0, w_i=0, a, b, c,i_2_6020=0,w_i_6020=0;

    // ʵʱ����Ԥ��
	for(int i=0;i<4;i++){
		i_2 += (chassis.ChassisMotorCanReceive[i].Current*RM3508_CURRENT_RATIO * chassis.ChassisMotorCanReceive[i].Current*RM3508_CURRENT_RATIO);
		w_i += (chassis.ChassisMotorCanReceive[i].Current*RM3508_CURRENT_RATIO * chassis.ChassisMotorCanReceive[i].RealSpeed);
	}
	
	#if Mecanum == 2 
	// ʵʱ��ǵ������
	for(int i=0;i<4;i++){
		i_2_6020 += (chassis.SteerMotorsCanReceive[i].Current*RM6020_CURRENT_RATIO * chassis.SteerMotorsCanReceive[i].Current*RM6020_CURRENT_RATIO);
		w_i_6020 += (chassis.SteerMotorsCanReceive[i].Current*RM6020_CURRENT_RATIO * chassis.SteerMotorsCanReceive[i].RealSpeed);
	}
	power_limit->predict_p0 = RM3508_P0 + i_2_6020*RM3508_R + w_i_6020*power_limit->k_dynamic;
	#else
	power_limit->predict_p0 = RM3508_P0;
	#endif
	
	//��������,��matlab���
	power_limit->actual_i_2 = i_2;
	power_limit->actual_w_i = w_i;
	power_limit->actual_i_2_6020 = i_2_6020;
	power_limit->actual_w_i_6020 = w_i_6020;
	
	//���������Ϲ���
  power_limit->predict_power = RM3508_R * (i_2) + power_limit->k_dynamic * (w_i) + power_limit->predict_p0;

	//����k
	if(power_limit->predict_power>0){
		power_limit->k_dynamic = LowPass_SetK(power_limit->k_dynamic,
											(power_limit->actual_ina260_power - RM3508_P0 - RM3508_R * (i_2+i_2_6020))/(w_i+w_i_6020));
		power_limit->k_dynamic = LIMIT_MAX_MIN(power_limit->k_dynamic,RM3508_K_MAX,RM3508_K_MIN);
	}
		
	/***********************������ֵ���й���Ԥ�⣨������������Ч��Ҫ�ã�*****************************************/
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
	//���ֹ���Ԥ�����Ӷ�ǵ��
		for(int i=0;i<4;i++){
			i_2_6020 += (chassis.SteeringCurrentSend[i]*RM6020_CURRENT_RATIO * chassis.SteeringCurrentSend[i] *RM6020_CURRENT_RATIO);
			w_i_6020 += (chassis.SteeringCurrentSend[i]*RM6020_CURRENT_RATIO * chassis.SteerMotorsCanReceive[i].RealSpeed);
		}
		c += i_2_6020*RM3508_R + w_i_6020*power_limit->k_dynamic;
	#endif
	c += RM3508_P0;
	//��k_c=1ʱ��Ԥ�⹦������
    power_limit->predict_send_power = a + b + c;//����ת��Ԥ���쳣����

	/***********************����Ԥ��󳬹��������˥��*****************************************/
//	power_limit->set_power = 80;
//	power_limit->set_power -= power_limit->error_control;
	// ������˥��
	if(power_limit->predict_send_power > power_limit->set_power)
	{	
		// ģ��˥������
		if(b*b < 4*(c - power_limit->set_power)*a){//   b��С��4ac��û�н�
			power_limit->k_c = LIMIT_MAX_MIN(-b/(2*a) , 1.0,0.0);//   -b/(2a)
		}else{
			float32_t sqrt_result;
			arm_sqrt_f32(b * b - 4*(c - power_limit->set_power)*a,&sqrt_result);
			power_limit->k_c = LIMIT_MAX_MIN((-b + sqrt_result) / 2 / a,1.0 ,0.0);
		}
		
		//˥��
		for(int i=0;i<4;i++){
			power_limit->pidChassisWheelSpeed[i].SetPoint *= power_limit->k_c;//˥��
		}
	}
	
	power_limit->set_superpower = LIMIT_MAX_MIN(power_limit->set_power - (a*power_limit->k_c*power_limit->k_c+b*power_limit->k_c+c),22*8, 0);
	
	//PI����������Ŀ�����
	for(int i=0;i<4;i++){
		power_limit->Current_set[i] = PID_Calc(&(power_limit->pidChassisWheelSpeed[i]),power_limit->Speed_Now[i]);
	}
}

/**********************************************************************************************************
*�� �� ��: Chassis_Current_send()
*����˵��: ���̵������ͺ���
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/

#if Mecanum == 2  //����
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
*�� �� ��: Chassis_CurrentPid_Cal
*����˵��: ���̲���
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Chassis_Loop_Cal(void)
{
	int i=0;
	Chassis_Speed_Cal();//����xyw���ٶȼ���Ŀ���ٶ�ֵ,����xyw�����ٶ�
	
	motion_decomposition(chassis.Velocity_vectors);//�˶��ֽ⣬����ÿ�����ӵ�setpoint
	
	
	//�������ƣ�����ÿ�����ӵĵ���ֵ
	if(F405.Chassis_Flag == Chassis_Powerdown_Mode)
	{
				
		for(i=0;i<4;i++)
		{
			chassis.WheelCurrentSend[i] = 0;
			chassis.SteeringVolSend[i] = 0;
		}
		powerlimit.set_superpower = JudgeReceive.MaxPower;//ȫ�����
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
		
		//��������
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
		
		PowerLimit(&powerlimit,superpower.PowerState);//������������ֵ ��ȷ��������������ϵͳĿ��
		
		set_powersate(&powerlimit,&superpower,powerlimit.PowerState_control);//�����������ݿ����ź�superpower.PowerState_Set
	
		
		for(int i=0;i<4;i++){
			chassis.WheelCurrentSend[i] = powerlimit.Current_set[i];
		}
	}
	
	//���͵���
	Chassis_Current_send();
	
}

/**********************************************************************************************************
*�� �� ��: BuildF105
*����˵��: ����Ҫ�����ϲ���F105�ṹ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void BuildF105(void)
{
	
	if(JudgeReceive.robot_id < 10)
		F105.Sendmessage.RobotRed = 1;
	else
		F105.Sendmessage.RobotRed = 0;			//0Ϊ��ɫ��1Ϊ��ɫ
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
*�� �� ��: Chassis_Init
*����˵��: �������˶�������ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Chassis_Init(void)
{	
	ZeroCheck_Init();
	powerlimit.k_dynamic = RM3508_K;
#if Robot_ID  == 46  
	powerlimit.error_control = 5; //�����5w
	powerlimit.No_limited_Power = 250; 		//W �������������ƹ���
	
	powerlimit.Min_remainEnergy = 25;  		//15J ���޸�
	powerlimit.Add_HalfCAP_Power = 10;			//W �������ݶ��ܵĹ���
	powerlimit.Min_capVol = 16.5;			//V  �������ݱ���������������
	
	//������С��������
	powerlimit.Min_capEnergy = 0.5*CAP_C*(powerlimit.Min_capVol*powerlimit.Min_capVol);
#else
	powerlimit.error_control = 5; //�����5W
	powerlimit.No_limited_Power = 300; 		//W �������������ƹ���
	
	powerlimit.Min_remainEnergy = 20;  		//15J ���޸�
	powerlimit.Add_HalfCAP_Power = 10;			//W �������ݶ��ܵĹ���
	powerlimit.Min_capVol = 17.0;			//V  �������ݱ���������������
	
	
	//������С��������
	powerlimit.Min_capEnergy = 0.5*CAP_C*(powerlimit.Min_capVol*powerlimit.Min_capVol);	
#endif
	short i=0;
	for(i=0;i<4;i++)
	{
		//�����������Ƴ���ת��
		powerlimit.pidChassisWheelSpeed[i].deadband=0.0;
		powerlimit.pidChassisWheelSpeed[i].P = 20.0f;
		powerlimit.pidChassisWheelSpeed[i].I = 1.0f;		
		powerlimit.pidChassisWheelSpeed[i].D = 0.0f;
		powerlimit.pidChassisWheelSpeed[i].ErrorMax = 1000.0f;
		powerlimit.pidChassisWheelSpeed[i].IMax = 2500.0f;
		powerlimit.pidChassisWheelSpeed[i].SetPoint = 0.0f;	
		powerlimit.pidChassisWheelSpeed[i].OutMax = 16000.0f;
		
		//������
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
	
	//��ʼ�Ƕ�
	chassis.SteeringAnglePosition_init[0] = 5147;
	chassis.SteeringAnglePosition_init[1] = 3061;
	chassis.SteeringAnglePosition_init[2] = 4921;
	chassis.SteeringAnglePosition_init[3] = 326;
}
/**********************************************************************************************************
*�� �� ��: Chassis_task
*����˵��: ��������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
uint32_t Chassis_high_water;

extern uint8_t JudgeReveice_Flag;
//extern TaskHandle_t JudgeReceiveTask_Handler; //������
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
		
		//���ݳ�ŵ����
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
		//��������
		Chassis_Loop_Cal();
		BuildF105();
		Can2Send0(&F105);
		
		VOFA_Send();

		IWDG_Feed();//ι��		
		vTaskDelayUntil(&xLastWakeTime,xFrequency); 
		 
	#if INCLUDE_uxTaskGetStackHighWaterMark
		Chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
	#endif
	}
}
