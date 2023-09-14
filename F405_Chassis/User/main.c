/**********************************************************************************************************
 * @�ļ�     main.c
 * @˵��     ���ļ�
 * @�汾  	 V1.0
 * @����     ��־��
 * @����     2021.4
**********************************************************************************************************/
/**********************************************************************************************************
 * @�ļ�     main.c
 * @˵��     ���ļ�
 * @�汾  	 V2.0
 * @����     ����
 * @����     2022.5
**********************************************************************************************************/
/**********************************************************************************************************
 * @�ļ�     infantry 2023
 * @˵��     ���ļ�
 * @�汾  	 V3.0
 * @����     ���κ�
 * @���Է�����
		1.�³����ȴ�Ӳ���������շ�������ע�͵����������ֹ�賵��ȷ���õ����CAN id������can1����can2�ϣ�can��FIFO��������
		2.Ӳ��ͨ�˺����ȱ궨Ӳ�����������ݵ�ADC����ϵ���������PID���������ϵ�����۲�Ԥ�⹦���Ƿ����׼ȷ��
		3.�鿴ÿ�����������init�����ڵĲ����Ƿ���Ҫ����
 * @����     2023.7
**********************************************************************************************************/
#include "main.h"
roboDisconnect Robot_Disconnect;
char Judge_Lost;
extern ChassisSpeed_t chassisSpeed;
extern Power_Limit_type powerlimit;
extern JudgeReceive_t JudgeReceive;
extern enum CHARGESTATE_Typedef ChargeState;
extern TaskHandle_t ChassisTask_Handler; //������
extern TaskHandle_t PowerControlTask_Handler; //������
extern SD_Saver_t SD_saver;
/**********************************************************************************************************
*�� �� ��: main
*����˵��: ������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
int main()
{
	BSP_Init();
	startTast();
	vTaskStartScheduler();
	while(1)
	{
		
	}
}

/**********************************************************************************************************
*�� �� ��: System_Config
*����˵��: ϵͳ��ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void BSP_Init(void)
{
	while(SysTick_Config(168000));	
	SuperPower_Configuration();//ADC��ʼ��Ҫ���ڴ���֮ǰ����Ȼ���ڲ��ܽ�������
	DAC1_Init();
	CAN1_Configuration();
	CAN2_Configuration();
	VOFA_USART_Configuration();
	UART4_Configuration();
	COUNTER_Configuration();
	TIM2_Configuration();
	TIM4_Configuration();
	IWDG_Config(IWDG_Prescaler_64 ,625);
	i2c_init();
	ChargeIO_Configuration();
	Charge_Off;
	ChargeState = ChargeOff;
	delay_ms(300);//�����ذ��ʼ����ɣ���ֹ���ذ��ʼ������������̷��ʹ�������
}
/**********************************************************************************************************
*�� �� ��: Offline_Check_task
*����˵��: ���߼������(�������ȼ�̫�ͣ�������Ŵ���ͨ�Żᵼ�����ݷ�����ȥ)
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Judge_Rst()
{
 JudgeReceive.MaxPower=60;
 JudgeReceive.remainEnergy=40;
}
/**********************************************************************************************************
*�� �� ��: Offline_Check_task
*����˵��: ���߼������(�������ȼ�̫�ͣ�������Ŵ���ͨ�Żᵼ�����ݷ�����ȥ)
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
uint32_t Offline_Check_high_water;
extern ext_student_interactive_header_data_t custom_grapic_draw;
extern uint8_t seq;
void Offline_Check_task(void *pvParameters)
{
	static char ChassisSuspend[4];
	vTaskDelay(3000); 
   while (1) {
		/*���ذ���߼��*/
		if(Robot_Disconnect.F405Disconnect>100)
		{
			SD_saver.number_error[Drop_F405]++;
			F405_Rst();
		}
		Robot_Disconnect.F405Disconnect++;
		
		/*����ϵͳ���߼��*/
		if(Robot_Disconnect.JudgeDisconnect>100)
		{
			SD_saver.number_error[Drop_Judge]++;
			Judge_Rst();
			Judge_Lost=1;
		}else
		{
		  Judge_Lost=0;
		}
		Robot_Disconnect.JudgeDisconnect++;
		
			/*���̵�����߼��*/
	for(int i=0;i<4;i++)
	{
		if(ABS(Robot_Disconnect.ChassisDisconnect[i])>100)
		{
			if(i==0)SD_saver.number_error[Drop_ChassisMotor1]++;
			if(i==1)SD_saver.number_error[Drop_ChassisMotor2]++;
			if(i==2)SD_saver.number_error[Drop_ChassisMotor3]++;
			if(i==3)SD_saver.number_error[Drop_ChassisMotor4]++;
			ChassisSuspend[i]=1;
			
		}else
		{
			if(ChassisSuspend[i])
			{
			ChassisSuspend[i]=0;
			break;
			}
		}
		
		if(ABS(Robot_Disconnect.steerDisconnect[i])>100)
		{
			if(i==0)SD_saver.number_error[Drop_SteerMotor1]++;
			if(i==1)SD_saver.number_error[Drop_SteerMotor2]++;
			if(i==2)SD_saver.number_error[Drop_SteerMotor3]++;
			if(i==3)SD_saver.number_error[Drop_SteerMotor4]++;
		}
		Robot_Disconnect.ChassisDisconnect[i]++;
		Robot_Disconnect.steerDisconnect[i]++;
	}	
	
	/*�������ݵ��߼��*/

	if(Robot_Disconnect.SuperPowerDisconnect>100)
	{
		SD_saver.number_error[Drop_SuperPower]++;
	}
	Robot_Disconnect.SuperPowerDisconnect++;	
	
	if(powerlimit.LowVol_halfcap){
		SD_saver.number_error[Low_vol]++;
	}
	
	SD_saver.total_number_1000hz++;
	IWDG_Feed();//ι��

	vTaskDelay(1); 
		 
#if INCLUDE_uxTaskGetStackHighWaterMark
        Offline_Check_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}


