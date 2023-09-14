#ifndef __POWERCONTROLTASK_H
#define __POWERCONTROLTASK_H

#define Bat_on	GPIO_SetBits(GPIOC,GPIO_Pin_5)
#define Bat_off	GPIO_ResetBits(GPIOC,GPIO_Pin_5)
#define CAP_on	GPIO_SetBits(GPIOC,GPIO_Pin_4)
#define CAP_off	GPIO_ResetBits(GPIOC,GPIO_Pin_4)
#define Charge_On	GPIO_SetBits(GPIOA,GPIO_Pin_6)
#define Charge_Off	GPIO_ResetBits(GPIOA,GPIO_Pin_6)


#define POWER_CHARGE_VOL_MIN 18.0f //开始线性衰减
#define POWER_CHARGE_VOL_MAX 21.0f //衰减到0
//功率限制
enum POWERSTATE_Typedef
{
	BAT = 0,
	CAP,
	HalfCAP
};
enum CHARGESTATE_Typedef
{
	ChargeOn = 0,
	ChargeOff
};

typedef struct SuperPower
{
	enum POWERSTATE_Typedef PowerState;//当前实际的能量状态 cap bat
	enum POWERSTATE_Typedef PowerState_Set;//被动电容时期望的能量状态 cap bat
	
    float actual_vol; //实际电容电压(乘采样比)
	float actual_power;//实际功率
		//电压采集变量
    unsigned int SUM_UCAP;  //电容电压
    unsigned int SUM_UDCDC; //充电电压
    unsigned int SUM_TEMP;  //温度

    float UCAP;
    float UDCDC;
    float temperate;
	
	int MaxChargePower; //最大充电功率，最小充电功率
	
    float i_set;             //充电电流
	short DAC_Set;
    
	float ina260_vol;//由ina测算的电容电压校准adc采样的值 能源是电容模式有效
	
	double change_timer;//当前实际时间
	//配置参数
	float CAP_AD_L, CAP_AD_H;//cap模式下切换电容的阈值
	float SUPERPOWER_SAMPLE_RATIO;//采样转换系数，新车要调一下
	float max_charge_i;//最大充电电流 A
	float Max_change_fre; //电容最大切换频率 HZ
	
    /* data */
} SuperPower;

void ChargeIO_Configuration(void);
void ADC_Filter(void);
void Pid_ChargeCal_Init(void);
void ChargeCal(void);
void Charge_Set(float I_Set);
void ChargeControl(void);
void chargetest(void);
void SuperPowerControl(void);
void Rc_k_Normalized_processing(void);

//热量控制
void HeatControl(void);
void HeatUpdate(void);

//发送结构体配置
void BuildF105(void);

//任务函数
void PowerControl_task(void *pvParameters);

#endif

