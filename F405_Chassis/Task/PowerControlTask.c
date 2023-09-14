/**********************************************************************************************************
 * @文件     PowerControlTask.c
 * @说明     功率控制
 * @版本  	 V1.0
 * @作者     陈志鹏
 * @日期     2020.1
**********************************************************************************************************/
/**********************************************************************************************************
 * @文件     PowerControlTask.c
 * @说明     功率控制
 * @版本  	 V2.0
 * @作者     戴军
 * @日期     2022.5
**********************************************************************************************************/
/**********************************************************************************************************
 * @文件     PowerControlTask.c
 * @说明     超级电容+被动电容控制
 * @版本  	 V3.0
 * @作者     郭嘉豪
 * @日期     2023.5
 * @说明：1. superpower.SUPERPOWER_SAMPLE_RATIO 板子采样值到实际值转换系数，因为电容分压设计导致不固定，每个车都标定一下，调整该值使得superpower.actual_vol = 电容实际值
		  2. 被动电容的开关频率可以根据需要设置
 
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
*函 数 名: 选择放电设备
*功能说明: 
*形    参: 无
*返 回 值: 无
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
*函 数 名: ChargeIO_Configuration
*功能说明: 充放电IO口初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ChargeIO_Configuration(void)
{
	GPIO_InitTypeDef gpio_init;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC, ENABLE);
	/**********************充电使能***********************************/
	gpio_init.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;
	gpio_init.GPIO_Mode=GPIO_Mode_OUT;
	gpio_init.GPIO_Speed=GPIO_Low_Speed;
	GPIO_Init(GPIOA,&gpio_init);	
	GPIO_ResetBits(GPIOA,GPIO_Pin_6);		//初始化时不充电
	GPIO_SetBits(GPIOA,GPIO_Pin_7);	//拉高Diode Mode
	/***********************放电选择使能***********************/
	gpio_init.GPIO_Pin=GPIO_Pin_4|GPIO_Pin_5;
	gpio_init.GPIO_Mode=GPIO_Mode_OUT;
	gpio_init.GPIO_Speed=GPIO_Low_Speed;
	GPIO_Init(GPIOC,&gpio_init);	
	Bat_on;
	CAP_off;
}


/**********************************************************************************************************
*函 数 名: ADC_Filter
*功能说明: 数据转换(得到电容实际电压)
*形    参: 无
*返 回 值: 无
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

    // /1000.0f表示从mV转到V   // *825 >> 10 / 30 等效于 *3300 / 2^12 等效于
    superpower.UCAP = (superpower.SUM_UCAP * 0.0268554688f) / 1000.0f;
    superpower.UDCDC = (superpower.SUM_UDCDC * 0.0268554688f) / 1000.0f;
    temp_temperature = (superpower.SUM_TEMP * 0.0268554688f) / 1000.0f;
    superpower.temperate = (temp_temperature - ADC_V25) / ADC_AVG_SLOPE + ADC_TEMP_BASE;
    superpower.actual_vol = superpower.UCAP * superpower.SUPERPOWER_SAMPLE_RATIO;
}

/**********************************************************************************************************
*函 数 名: Charge_Set
*功能说明: 充电电流设置
*形    参: I_Set
*返 回 值: 无
**********************************************************************************************************/
float last_i_set=0.0f;
float I_Set_Jump;
float Actual_Iset;
void Charge_Set(float i_Set)
{
	//加入斜坡函数处理
//	if(fabs(i_Set-last_i_set)>I_Set_Jump)
//	{
//	i_Set=(i_Set-last_i_set)*0.2+last_i_set;
//	}
//	根据充电IC特性，根据电流感应电阻为10m欧姆，可得到如下计算公式，此为最大充电电流
//  超级电容电压冲到Vcc时，充电IC变回进入休眠模式，减少电容电压流失
	superpower.DAC_Set = i_Set*0.2f/3.3f*4096;    
	superpower.DAC_Set = LIMIT_MAX_MIN(superpower.DAC_Set,4096,0);
	DAC_SetChannel2Data(DAC_Align_12b_R,superpower.DAC_Set);
	last_i_set=i_Set;
}

/**********************************************************************************************************
*函 数 名: PidInit
*功能说明: 充电PID初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void super_power_Init(){
	
	Bat_use();
	superpower.max_charge_i = 5.0;//最大充电电流 A
	
	superpower.SUPERPOWER_SAMPLE_RATIO = 8.25; //采样转换系数，使得superpower.actual_vol和电容实际电表测量相同，新车要调一下
	superpower.CAP_AD_H = 14.0;
	superpower.CAP_AD_L = 10.0;
	
	superpower.Max_change_fre = 2.5;//电容最大切换频率 HZ
	
	ChargeCtl.SetPoint = 40000;
	ChargeCtl.P = 0.5;
	ChargeCtl.I = 0;
	ChargeCtl.IMax = 0;
	ChargeCtl.OutMax = superpower.max_charge_i*1000;
}

/**********************************************************************************************************
*函 数 名: ChargeControl
*功能说明: 超级电容控制
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
float test_current;
int t;
void ChargeControl(void)
{
	/******************充电控制*******************/
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
*函 数 名: PowerControl_task
*功能说明: 底盘任务
*形    参: 无
*返 回 值: 无
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
		/******************数据获取*******************/
		// 电压变化较慢，不需要读电压，可以从裁判系统读
		INA_READ_Vol();			
		INA_READ_Current();
		INA_READ_Power();
		ADC_Filter();
		if(superpower.actual_vol>3)
		{
		Robot_Disconnect.SuperPowerDisconnect=0;
		}
		/*****************超级电容控制****************/
		//HalfCAPCal();
		ChargeControl();
		//chargetest();
		
		//Max_change_fre频率控制电源切换
		superpower.change_timer += ((double)(xTaskGetTickCount()-control_power_last))/1000;
		if(superpower.change_timer>(1/superpower.Max_change_fre) || F405.SuperPowerLimit==CAP){//切换频率满足or手动切换
			superpower.change_timer = 0;
			if(superpower.PowerState_Set == CAP) CAP_use();
			else Bat_use();
		}
		control_power_last = xTaskGetTickCount();
		IWDG_Feed();
		
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
	}
}
