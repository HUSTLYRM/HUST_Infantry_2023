/**********************************************************************************************************
 * @�ļ�     PowerControlTask.c
 * @˵��     ���ʿ���
 * @�汾  	 V1.0
 * @����     ��־��
 * @����     2020.1
**********************************************************************************************************/
/**********************************************************************************************************
 * @�ļ�     PowerControlTask.c
 * @˵��     ���ʿ���
 * @�汾  	 V2.0
 * @����     ����
 * @����     2022.5
**********************************************************************************************************/
/**********************************************************************************************************
 * @�ļ�     PowerControlTask.c
 * @˵��     ��������+�������ݿ���
 * @�汾  	 V3.0
 * @����     ���κ�
 * @����     2023.5
 * @˵����1. superpower.SUPERPOWER_SAMPLE_RATIO ���Ӳ���ֵ��ʵ��ֵת��ϵ������Ϊ���ݷ�ѹ��Ƶ��²��̶���ÿ�������궨һ�£�������ֵʹ��superpower.actual_vol = ����ʵ��ֵ
		  2. �������ݵĿ���Ƶ�ʿ��Ը�����Ҫ����
 
**********************************************************************************************************/

#include "main.h"
#include "stm32f4xx_dac.h"
SuperPower superpower;

float temperate,temp3v3;
float I_Set = 0;
Pid_Typedef ChargeCtl;
char Chassis_Run_Flag;
enum CHARGESTATE_Typedef ChargeState = ChargeOff;
extern unsigned short ADC_ConvertedValue[90];
extern INA260 INA260_1,INA260_2;
extern F405_typedef F405;
extern Power_Limit_type powerlimit;
extern JudgeReceive_t JudgeReceive;
extern ChassisSpeed_t chassisSpeed;
extern F105_Typedef F105;
extern int P_State;
extern short CAP_CrossoverFlag;
extern roboDisconnect Robot_Disconnect;


/**********************************************************************************************************
*�� �� ��: ѡ��ŵ��豸
*����˵��: 
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Bat_use(){
	CAP_off;   
	Bat_on;
	superpower.PowerState = BAT;
}
void CAP_use(){
	CAP_on;   
	Bat_off;
	superpower.PowerState = CAP;
}

/**********************************************************************************************************
*�� �� ��: ChargeIO_Configuration
*����˵��: ��ŵ�IO�ڳ�ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void ChargeIO_Configuration(void)
{
	GPIO_InitTypeDef gpio_init;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC, ENABLE);
	/**********************���ʹ��***********************************/
	gpio_init.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;
	gpio_init.GPIO_Mode=GPIO_Mode_OUT;
	gpio_init.GPIO_Speed=GPIO_Low_Speed;
	GPIO_Init(GPIOA,&gpio_init);	
	GPIO_ResetBits(GPIOA,GPIO_Pin_6);		//��ʼ��ʱ�����
	GPIO_SetBits(GPIOA,GPIO_Pin_7);	//����Diode Mode
	/***********************�ŵ�ѡ��ʹ��***********************/
	gpio_init.GPIO_Pin=GPIO_Pin_4|GPIO_Pin_5;
	gpio_init.GPIO_Mode=GPIO_Mode_OUT;
	gpio_init.GPIO_Speed=GPIO_Low_Speed;
	GPIO_Init(GPIOC,&gpio_init);	
	Bat_on;
	CAP_off;
}


/**********************************************************************************************************
*�� �� ��: ADC_Filter
*����˵��: ����ת��(�õ�����ʵ�ʵ�ѹ)
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void ADC_Filter()
{
    float temp_temperature;

    superpower.SUM_UCAP = 0;
    superpower.SUM_UDCDC = 0;
    superpower.SUM_TEMP = 0;
    for (int i = 0; i < ADC_SAMPLE_NUM; i++)
    {
        superpower.SUM_UCAP += ADC_ConvertedValue[i * 3];
        superpower.SUM_UDCDC += ADC_ConvertedValue[i * 3 + 1];
        superpower.SUM_TEMP += ADC_ConvertedValue[i * 3 + 2];
    }

    // /1000.0f��ʾ��mVת��V   // *825 >> 10 / 30 ��Ч�� *3300 / 2^12 ��Ч��
    superpower.UCAP = (superpower.SUM_UCAP * 0.0268554688f) / 1000.0f;
    superpower.UDCDC = (superpower.SUM_UDCDC * 0.0268554688f) / 1000.0f;
    temp_temperature = (superpower.SUM_TEMP * 0.0268554688f) / 1000.0f;
    superpower.temperate = (temp_temperature - ADC_V25) / ADC_AVG_SLOPE + ADC_TEMP_BASE;
    superpower.actual_vol = superpower.UCAP * superpower.SUPERPOWER_SAMPLE_RATIO;
}

/**********************************************************************************************************
*�� �� ��: Charge_Set
*����˵��: ����������
*��    ��: I_Set
*�� �� ֵ: ��
**********************************************************************************************************/
float last_i_set=0.0f;
float I_Set_Jump;
float Actual_Iset;
void Charge_Set(float i_Set)
{
	//����б�º�������
//	if(fabs(i_Set-last_i_set)>I_Set_Jump)
//	{
//	i_Set=(i_Set-last_i_set)*0.2+last_i_set;
//	}
//	���ݳ��IC���ԣ����ݵ�����Ӧ����Ϊ10mŷķ���ɵõ����¼��㹫ʽ����Ϊ��������
//  �������ݵ�ѹ�嵽Vccʱ�����IC��ؽ�������ģʽ�����ٵ��ݵ�ѹ��ʧ
	superpower.DAC_Set = i_Set*0.2f/3.3f*4096;    
	superpower.DAC_Set = LIMIT_MAX_MIN(superpower.DAC_Set,4096,0);
	DAC_SetChannel2Data(DAC_Align_12b_R,superpower.DAC_Set);
	last_i_set=i_Set;
}

/**********************************************************************************************************
*�� �� ��: PidInit
*����˵��: ���PID��ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void super_power_Init(){
	
	Bat_use();
	superpower.max_charge_i = 5.0;//�������� A
	
	superpower.SUPERPOWER_SAMPLE_RATIO = 8.25; //����ת��ϵ����ʹ��superpower.actual_vol�͵���ʵ�ʵ�������ͬ���³�Ҫ��һ��
	superpower.CAP_AD_H = 14.0;
	superpower.CAP_AD_L = 10.0;
	
	superpower.Max_change_fre = 2.5;//��������л�Ƶ�� HZ
	
	ChargeCtl.SetPoint = 40000;
	ChargeCtl.P = 0.5;
	ChargeCtl.I = 0;
	ChargeCtl.IMax = 0;
	ChargeCtl.OutMax = superpower.max_charge_i*1000;
}

/**********************************************************************************************************
*�� �� ��: ChargeControl
*����˵��: �������ݿ���
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
float test_current;
int t;
void ChargeControl(void)
{
	/******************������*******************/
	//ChargeCal(); 
	if(superpower.actual_vol<22.5 && powerlimit.set_superpower>10){
		superpower.i_set = LIMIT_MAX_MIN(powerlimit.set_superpower/superpower.actual_vol,superpower.max_charge_i, 0.0);
	}else{
		superpower.i_set = 0;
	}
	Charge_Set(superpower.i_set);
}

void chargetest(void)
{
	if(F405.SuperPowerLimit > 0)
	{
		CAP_use();
	}
}

/**********************************************************************************************************
*�� �� ��: PowerControl_task
*����˵��: ��������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
uint32_t PowerControl_high_water;
float readVol;
void PowerControl_task(void *pvParameters)
{
	portTickType xLastWakeTime;
	
	super_power_Init();
	TickType_t control_power_last=0;
	superpower.change_timer = 0;
	while (1) {
		/******************���ݻ�ȡ*******************/
		// ��ѹ�仯����������Ҫ����ѹ�����ԴӲ���ϵͳ��
		INA_READ_Vol();			
		INA_READ_Current();
		INA_READ_Power();
		ADC_Filter();
		if(superpower.actual_vol>3)
		{
		Robot_Disconnect.SuperPowerDisconnect=0;
		}
		/*****************�������ݿ���****************/
		//HalfCAPCal();
		ChargeControl();
		//chargetest();
		
		//Max_change_freƵ�ʿ��Ƶ�Դ�л�
		superpower.change_timer += ((double)(xTaskGetTickCount()-control_power_last))/1000;
		if(superpower.change_timer>(1/superpower.Max_change_fre) || F405.SuperPowerLimit==CAP){//�л�Ƶ������or�ֶ��л�
			superpower.change_timer = 0;
			if(superpower.PowerState_Set == CAP) CAP_use();
			else Bat_use();
		}
		control_power_last = xTaskGetTickCount();
		IWDG_Feed();
		
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
	}
}
