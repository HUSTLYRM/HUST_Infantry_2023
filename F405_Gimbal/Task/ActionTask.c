/**********************************************************************************************************
 * @�ļ�     ActionTask.c
 * @˵��     ��������
 * @�汾  	 V1.0
 * @����     ��־��
 * @����     2020.1
 **********************************************************************************************************/
/**********************************************************************************************************
 * @�ļ�     ActionTask.c
 * @˵��     ģʽ�л�����
 * @�汾  	 V2.0
 * @����     ����
 * @����     2022.4
 **********************************************************************************************************/
#include "main.h"
/*--------------�ڲ�����-------------------*/
short Mouse_Key_Flag;
short waitFlag[10] = {10, 0, 0, 0, 0, 0, 0, 0, 0, 0};
short SpeedMode = 1; //Ħ�������ٵ�λ
short ctrl_rising_flag, pre_key_ctrl, v_rising_flag, pre_key_v, c_rising_flag, pre_key_c, pre_key_e, e_rising_flag, x_rising_flag, pre_key_x, Press_Key_x_Flag, v_rising_flag, pre_key_b;
short q_rising_flag, b_rising_flag, f_rising_flag, g_rising_flag, pre_key_f, mouse_Press_l_rising_flag, pre_mouse_l, r_rising_flag, pre_key_r, z_rising_flag, pre_key_z, Press_Key_z_Flag;
short pre_key_q, pre_key_g, pre_v_rising_flag; //��һv��������״̬
short v_high_flag;
char q_flag, f_flag, HighFreq_flag;
char k_slow;
uint32_t prese_ctrl_timer;
float prese_ctrl_interval=0,ctrl_interval=0;//ctrl�������
short ctrl_prese_countinue_flag=0;//��������
char ctrl_mode;
char smallBuff_flag;
short Turning_flag;		//С���ݱ�־
char Graphic_Init_flag; //ͼ�γ�ʼ����־
char Budan, Buff_flag;
short PC_Sendflag;
float Buff_Yaw_Motor;
char Buff_Init;
//���ָǶ�� ����ʱ��
short SteeringEngineDelay = 0;
/*---------------�ṹ��--------------------*/
Status_t Status;
/*--------------�ⲿ����-------------------*/
extern RC_Ctl_t RC_Ctl;
extern unsigned char magazineState;
extern Pid_Typedef PidBodanMotorPos, PidBodanMotorSpeed;
extern RobotInit_Struct Infantry;
extern F105_Typedef F105;
extern F405_typedef F405;
extern PC_Receive_t PC_Receive;
extern float Bodan_Pos;
extern Gimbal_Typedef Gimbal;
extern float MirocPosition; //���Ʋ�����ת
extern short FrictionWheel_speed;
extern RobotInit_Struct Infantry;
extern float Onegrid;
extern PCSendData pc_send_data;
extern ShootTask_typedef Shoot;
/**********************************************************************************************************
*�� �� ��: Status_Act
*����˵��: �����������������ֵ��̣���̨�����������
					 ���̣� 1)  Chassis_Act_Cal(Remote rc,Key key)                                 ��������ģʽ
									2)  Chassis_SelfProtect_Cal(Remote rc,Key key)                         ���ұ���ģʽ��С���ݣ�
									3)	Chassis_Solo_Cal(Remote rc,Key key)																 ����ģʽ
									4)  Chassis_Powerdown_Cal()                                            ��������ģʽ

					 ��̨�� 1)  Gimbal_Act_Cal(Remote rc,Mouse mouse,PC_Recv_t *Pc_RecvData)       ��̨����ģʽ
									2)  Gimbal_Armor_Cal(PC_Recv_t *Pc_Recv, RC_Ctl_t *RC_clt)             �������ģʽ
									3)  Gimbal_ShenFu_Cal(PC_Recv_t *Pc_Recv, RC_Ctl_t *RC_clt)            �������ģʽ
									4)  Gimbal_Powerdown_Cal()                                             ��̨����ģʽ

					 ���   1)  Shoot_Check_Cal();                                                 ��¼ģʽʹ��
							2)  Shoot_Fire_Cal();                                                  �������ģʽ
									3)  Shoot_Armor_Cal();                                                 �������ģʽ
									4)  Shoot_ShenFu_Cal();                                                ��������ģʽ
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Status_Act(void)
{
	SetInputMode(RC_Ctl.rc);
	switch (Status.ControlMode)
	{
	case Control_RC_Mode:
		Remote_Process(RC_Ctl.rc);
		break;
	case Control_MouseKey_Mode:
		Mouse_Key_Process(RC_Ctl);
		break;
	case Control_Powerdown_Mode:
		Powerdown_Process();
		break;
	default:
		break;
	}

	if (Mouse_Key_Flag == 3)
	{
		MouseKey_Act_Cal(RC_Ctl);
	}
	F405Can1Send(&F405); //����405��Ϣ
}

/**********************************************************************************************************
 *�� �� ��: SetInputMode
 *����˵��: ����ģʽѡ��
 *��    ��: rc
 *�� �� ֵ: ��
 **********************************************************************************************************/
void SetInputMode(Remote rc)
{
	switch (rc.s1)
	{
	case 1:
		Status.ControlMode = Control_RC_Mode;
		break;
	case 3:
		Status.ControlMode = Control_MouseKey_Mode;
		break;
	case 2:
		Status.ControlMode = Control_Powerdown_Mode;
		SteeringEngine_Set(Infantry.MagOpen);

		break;
	}
	//	Tx2_Off_Test(rc);	//ң��������tx2�ػ��ж�
}

/**********************************************************************************************************
 *�� �� ��: Powerdown_Process
 *����˵��: �ϵ�ģʽ
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
void Powerdown_Process()
{
	if (Mouse_Key_Flag != 1)
	{
		Mouse_Key_Flag = 1;
		Budan = 1;
		q_flag = 0;
		f_flag = 0;
		F405.SuperPowerLimit = BAT;
		HighFreq_flag = 0;
		Buff_Init = 0;
	}
	Status.ChassisMode = Chassis_Powerdown_Mode;
	Status.GimbalMode = Gimbal_Powerdown_Mode;
	Status.ShootMode = Shoot_Powerdown_Mode;
}

///**********************************************************************************************************
//*�� �� ��: Tx2_Off_Test
//*����˵��: Tx2�ػ�ָ���жϣ����Ҳ��˲������£�������ģʽ����ҡ�����ڿ������͹ػ�ָ��
//*��    ��: ��
//*�� �� ֵ: ��
//**********************************************************************************************************/
// void Tx2_Off_Test(Remote rc)
//{
//	if(rc.s1 == 2 && rc.ch0 < 500 && rc.ch2 > 1500)
//		Tx2_Off_times++;		//������
//	else
//		{
//			Tx2_Off_times = 0;
//			PC_Sendflag = 0;
//		}
//	if(Tx2_Off_times > 500)
//		PC_Sendflag = Tx2_Off;
//}

/**********************************************************************************************************
 *�� �� ��: Remote_Process
 *����˵��: ң����ģʽѡ��
 *��    ��: rc
 *�� �� ֵ: ��
 **********************************************************************************************************/

void Remote_Process(Remote rc)
{
	if (Mouse_Key_Flag != 2)
	{
		Mouse_Key_Flag = 2;
		Budan = 0;
	}

	if (rc.s2 == 1) //��¼���ģʽ
	{
		Status.GimbalMode = Gimbal_Act_Mode;
		Status.ChassisMode = Chassis_Powerdown_Mode;
		Status.ShootMode = Shoot_Check_Mode;
		SteeringEngine_Set(Infantry.MagOpen);
	}
	
	
	if (rc.s2 == 3) //����ģʽ
	{
		Buff_Init = 0;
		Status.GimbalMode = Gimbal_Act_Mode;
		Status.ChassisMode = Chassis_Act_Mode;//Chassis_Powerdown_Mode;
		Status.ShootMode = Shoot_Powerdown_Mode;
		SteeringEngine_Set(Infantry.MagClose);
	}
	
	if (rc.s2 == 2) //С����ģʽ
	{
		Status.GimbalMode = Gimbal_Act_Mode;
		Status.ChassisMode = Chassis_SelfProtect_Mode;
		Status.ShootMode = Shoot_Powerdown_Mode;
		SteeringEngine_Set(Infantry.MagClose);
	}
}

/**********************************************************************************************************
*�� �� ��: MouseKey_Act_Cal
*����˵��: ����ģʽ
		   w,s,a,d          				ǰ�����Һ���
		   q				                ���飬�Լ��򵯣���ס��
		   e             	                �л����ȼ�
		   r          		                �л����ո�
		   shift+f			                ���ӵ���
		   f                                ���ٵ���
		   g				                ���̷�תһ�񣨰�һ�£�
		   shift+z			                ���(�Ȱ���shift,�ٰ�һ��z����/�˳�,Ȼ��Ϳ��ɿ�shift��������)
		   shift+x                          С��(�Ȱ���shift,�ٰ�һ��x����/�˳�,Ȼ��Ϳ��ɿ�shift��������)
		   c                                �ߵ���Ƶ�л�
		   v                                ������ʼ��
		   b				                ˢ��UI
		   shift                            ʹ�ó������ݣ���ס��
		   ctrl                             С����(��ס)     �Զ��ڱ�ģʽ���������ν��룬�ٰ������˳���
		   mouse.press_r������Ҽ���    	����ס�����飬�������Լ���
		   mouse.press_l����������   		��������(����)/��������
		   ������         		        ����΢��

*��    ��: RC_Ctl
*�� �� ֵ: ��
**********************************************************************************************************/
extern float Inte_z;
extern float GimbalPitchPos;
extern ShootTask_typedef Shoot;
extern int Shoot_Init_flag;
float k_onegrid = 0.01f;
float k_single = 0.9f;
short aaaa = 500;
char ARMOR_FLAG = 0;
char Last_shift;
char shift_press_flag, shift_flag;
//extern char switchPriority;
extern FuzzyPID FuzzyAidPidPitchPos;
extern Pid_Typedef PidPitchAidSpeed;
extern Pid_Typedef PidPitchAidPos[2];
extern uint8_t ControlMode;
int intshort[4];
short pitchcurrent;

void MouseKey_Act_Cal(RC_Ctl_t RC_Ctl)
{
	static int waitb = 0;

	/******************************���̷�ת g��*****************************************/
	g_rising_flag = RC_Ctl.key.g - pre_key_g;
	pre_key_g = RC_Ctl.key.g;
	if (g_rising_flag == 1)
	{
		Shoot.ReverseRotation = 1;
		Shoot.ShootContinue = 0;
		g_rising_flag = 0;
	}

	/******************************ͼ�ν����ʼ�� b��*****************************************/
	b_rising_flag = RC_Ctl.key.b - pre_key_b;
	pre_key_b = RC_Ctl.key.b;
	if (RC_Ctl.key.b == 1)
	{


        F405.Graphic_Init_Flag = 1;
        Graphic_Init_flag = 1;
    }

		else
		{
//			waitb++;
//			if (waitb > 5)
//			{
				F405.Graphic_Init_Flag = 0;
				Graphic_Init_flag = 0;
//				waitb = 0;
//			}
		}

	/******************************���ұ���ģʽ(С����) ctrl��*****************************************/		
	if (Status.GimbalMode != Gimbal_BigBuf_Mode && Status.GimbalMode != Gimbal_SmlBuf_Mode)
	{
		ctrl_rising_flag = RC_Ctl.key.ctrl - pre_key_ctrl;
		pre_key_ctrl = RC_Ctl.key.ctrl;
		if (ctrl_rising_flag == 1){
			Turning_flag++;
		}
		
		ctrl_interval += GetDeltaT(&prese_ctrl_timer);
		if (ctrl_rising_flag == 1){
			if(ctrl_interval<0.5){//�������εļ��С��500ms
				ctrl_prese_countinue_flag++;
			}else{
				ctrl_prese_countinue_flag=1;
			}
			ctrl_interval = 0;
		}else if(ctrl_interval>1.0){
			ctrl_prese_countinue_flag=1;
		}
#if Robot_ID == 46	//���֣��������εı�־λ	 
		if(ctrl_prese_countinue_flag == 3){
			ctrl_prese_countinue_flag=1;
			ctrl_mode= ctrl_mode==1?0:1;
		}
#else
		ctrl_mode = 0;
#endif
		if (RC_Ctl.key.ctrl == 1) //��û�б�����ģʽ����һֱ���ŵ����
		{
			if (Status.ChassisMode == Chassis_Act_Mode || Status.ChassisMode == Chassis_SelfProtect_Mode)
				Status.ChassisMode = Chassis_SelfProtect_Mode;
		}else if(ctrl_mode==1){//��������
			if (Status.ChassisMode == Chassis_Act_Mode || Status.ChassisMode == Chassis_SelfProtect_Mode){
				Status.ChassisMode = Chassis_SelfProtect_Mode;
				Status.GimbalMode = Gimbal_Test_Mode;
			}
		}
		else if (Status.ChassisMode == Chassis_SelfProtect_Mode)
		{
			Status.ChassisMode = Chassis_Act_Mode;
			Status.GimbalMode = Gimbal_Act_Mode;
		}
	}
	/******************************�л����ȼ� e��*****************************************/
	e_rising_flag = RC_Ctl.key.e - pre_key_e;
	pre_key_e = RC_Ctl.key.e;
    if(RC_Ctl.key.e == 1)
    {
        pc_send_data.change_priority_flag = 1;
    }
    else
    {
        pc_send_data.change_priority_flag = 0;
    }

	/********************************�������ݿ���**********************************************/
	if (RC_Ctl.key.shift == 1)
	{
		F405.SuperPowerLimit = CAP; //ʹ�ó�������
	}
	else
	{
		F405.SuperPowerLimit = BAT; //��ʹ�ó�������
	}

	/****************************** �������д�ģʽ q��*****************************************/
    
    q_rising_flag = RC_Ctl.key.q - pre_key_q;
    pre_key_q = RC_Ctl.key.q;
	if (Status.ShootMode != Shoot_Powerdown_Mode)
	{
		if (q_rising_flag == 1 /*&& (Status.GimbalMode == Gimbal_Armor_Mode
                                || Status.GimbalMode == Gimbal_BigBuf_Mode
                                || Status.GimbalMode == Gimbal_SmlBuf_Mode)*/
                                    && Status.ShootMode == Shoot_Fire_Mode)
		{
//			Status.GimbalMode = Gimbal_Armor_Mode;
			Status.ShootMode = Shoot_Tx2_Mode;
			q_flag = 1;
		}
		else if (q_rising_flag == 1 && Status.ShootMode == Shoot_Tx2_Mode && Status.GimbalMode != Gimbal_Powerdown_Mode)
		{
//			Status.GimbalMode = Gimbal_Act_Mode;
			Status.ShootMode = Shoot_Fire_Mode;
			q_flag = 0;
		}
	}

	/******************************�ߵ���Ƶ�л� c��*****************************************/
	c_rising_flag = RC_Ctl.key.c - pre_key_c;
	pre_key_c = RC_Ctl.key.c;
	if (c_rising_flag == 1)
	{
		HighFreq_flag = !HighFreq_flag;
	}

	/******************************���ո� r��*****************************************/
    r_rising_flag = RC_Ctl.key.r - pre_key_r;
    pre_key_r = RC_Ctl.key.r;
	static char steer_open_flag = 0;
    if(r_rising_flag == 1)
    {
       if(steer_open_flag == 0){
		   steer_open_flag = 1;
		   Budan = 1;
		   GimbalPitchPos-=15;
	   }else{
		   steer_open_flag = 0;
		   Budan = 0;
		   GimbalPitchPos+=15;
	   }
    }
	
	/******************************������ʼ�� v��*****************************************/
	if (RC_Ctl.key.v == 1)
	{
		Inte_z = 0;
	}

	/******************************�������� f+r��*****************************************/
    
    f_rising_flag = RC_Ctl.key.f - pre_key_f;
    pre_key_f = RC_Ctl.key.f;
    if(f_rising_flag == 1 && RC_Ctl.key.shift !=1)
    {
        Infantry.High_FrictionSpeed -= 100;
        Infantry.Medium_FrictionSpeed -= 100;
        Infantry.Low_FrictionSpeed -= 100;
    }else if(f_rising_flag == 1 && RC_Ctl.key.shift ==1){//SHIFT+F�ӵ���
		Infantry.High_FrictionSpeed += 100;
        Infantry.Medium_FrictionSpeed += 100;
        Infantry.Low_FrictionSpeed += 100;
	}
	
	
	///******************************���ģʽ z��*****************************************/
	z_rising_flag = RC_Ctl.key.z - pre_key_z;
	pre_key_z = RC_Ctl.key.z;
	if (z_rising_flag == 1 && RC_Ctl.key.shift == 1)
	{
		if (Status.GimbalMode == Gimbal_BigBuf_Mode) //
		{
			Status.GimbalMode = Gimbal_Act_Mode;
//            Status.ShootMode = Shoot_Fire_Mode;
			Buff_Init = 0;
		}
		else
		{
			if (Buff_Init == 0)
			{
				Buff_Yaw_Motor = Gimbal.Yaw.Gyro;
				Buff_Init = 1;
			}
			Status.GimbalMode = Gimbal_BigBuf_Mode;
            //Status.ShootMode = Shoot_Tx2_Mode;
		}
	}

	//		/******************************Small  Buff ��С�� x��*****************************************/
	x_rising_flag = RC_Ctl.key.x - pre_key_x;
	pre_key_x = RC_Ctl.key.x;
	if (x_rising_flag == 1 && RC_Ctl.key.shift == 1)
	{
		if (Status.GimbalMode != Gimbal_SmlBuf_Mode)
		{
			if (Buff_Init == 0)
			{
				Buff_Yaw_Motor = Gimbal.Yaw.Gyro;
				Buff_Init = 1;
			}
			Status.GimbalMode = Gimbal_SmlBuf_Mode;
            //Status.ShootMode = Shoot_Tx2_Mode;
		}
		else
		{
			Buff_Init = 0;
			Status.GimbalMode = Gimbal_Act_Mode;
//            Status.ShootMode = Shoot_Fire_Mode;
		}
	}

	/******************************��갴�����*******************************************/
	mouse_Press_l_rising_flag = RC_Ctl.mouse.press_l - pre_mouse_l;
	pre_mouse_l = RC_Ctl.mouse.press_l;
    
    if (Status.ShootMode != Shoot_Fire_Mode && Status.ShootMode != Shoot_Powerdown_Mode && RC_Ctl.mouse.press_l == 1)
    {
        Status.ShootMode = Shoot_Fire_Mode;
    }//�ֶ������������
    
	if (Status.ShootMode == Shoot_Fire_Mode)
	{
		if (RC_Ctl.mouse.press_l == 1 && Shoot.ReverseRotation == 0)
		{
			waitFlag[5]++;
			if ((waitFlag[5] < 50) && (waitFlag[5] > 0)) //�̰�     //��ʱ���Ե�һ��
			{
				if (mouse_Press_l_rising_flag == 1)
				{
					waitFlag[5] = 0;
					if (Shoot.HeatControl.IsShootAble == 1) // Heat_Limit
					{
						if (ABS(Bodan_Pos - PidBodanMotorPos.SetPoint) < PluckThreholdPos && F405.Fric_Flag == 1)
						{
							MirocPosition -= Onegrid;
							Shoot.ShootContinue = 0;
						}
					}
				}
			}
			if (waitFlag[5] > 100) //����
			{

				if (Shoot.HeatControl.IsShootAble == 1)
				{
					if (ABS(Bodan_Pos - PidBodanMotorPos.SetPoint) < PluckThreholdPos && F405.Fric_Flag == 1)
					{
						Shoot.ShootContinue = 1;
					}
				}
			}
		}
		else if (RC_Ctl.mouse.press_l == 0 && Shoot.ReverseRotation == 0) //����
		{
			waitFlag[5] = 0;
			if (Shoot.ShootContinue == 1 || ABS(Bodan_Pos - PidBodanMotorPos.SetPoint) < 1000)
			{
				PidBodanMotorPos.SetPoint = Bodan_Pos;
			}
			Shoot.ShootContinue = 0;
		}
	}
	/******************************����������ƣ����飩 �Ҽ�*****************************************/
	if (RC_Ctl.mouse.press_r == 1 && (Status.GimbalMode != Gimbal_BigBuf_Mode && Status.GimbalMode != Gimbal_SmlBuf_Mode))
	{
		Status.GimbalMode = Gimbal_Armor_Mode;
//        if(RC_Ctl.mouse.press_l != 1)
//        Status.ShootMode = Shoot_Tx2_Mode;
	}
	else if (/*q_flag == 0 && */Status.GimbalMode == Gimbal_Armor_Mode)
	{
		Status.GimbalMode = Gimbal_Act_Mode;
//        Status.ShootMode = Shoot_Fire_Mode;
	}
}

/**********************************************************************************************************
 *�� �� ��: Mouse_Key_Process
 *����˵��: ����ģʽ��ʼ��
 *��    ��: RC_Ctl
 *�� �� ֵ: ��
 **********************************************************************************************************/
void Mouse_Key_Process(RC_Ctl_t RC_Ctl)
{
	/******************************���β�����ʼ*****************************************/
	if (Mouse_Key_Flag != 3)
	{
		SpeedMode = 1;
		Mouse_Key_Flag = 3;
		magazineState = 0x00; // Close
		SteeringEngine_Set(Infantry.MagClose);
		Status.GimbalMode = Gimbal_Act_Mode;
		Status.ChassisMode = Chassis_Act_Mode;
		Status.ShootMode = Shoot_Fire_Mode;
		Budan = 0;
	}

	//���Ʋ������ص��ո�
	if (Budan)
	{
		SteeringEngine_Set(Infantry.MagOpen);
	}
	else
	{
		SteeringEngine_Set(Infantry.MagClose);
	}

	//  ����Ħ���ֿ���
	if (RC_Ctl.rc.s2 == 2)
	{
		Status.ShootMode = Shoot_Powerdown_Mode;
	}
	else
	{
		if (Status.ShootMode != Shoot_Tx2_Mode)
			Status.ShootMode = Shoot_Fire_Mode;
	}
}

/**********************************************************************************************************
 *�� �� ��: ModeChoose_task
 *����˵��: ģʽѡ������
 *��    ��: pvParameters
 *�� �� ֵ: ��
 **********************************************************************************************************/
uint32_t ModeChoose_high_water;
extern char Chassis_ID;
void ModeChoose_task(void *pvParameters)
{

	while (1)
	{

		Status_Act();
		IWDG_Feed();
		vTaskDelay(5);

#if INCLUDE_uxTaskGetStackHighWaterMark
		ModeChoose_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
	}
}
