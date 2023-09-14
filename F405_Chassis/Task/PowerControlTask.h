#ifndef __POWERCONTROLTASK_H
#define __POWERCONTROLTASK_H

#define Bat_on	GPIO_SetBits(GPIOC,GPIO_Pin_5)
#define Bat_off	GPIO_ResetBits(GPIOC,GPIO_Pin_5)
#define CAP_on	GPIO_SetBits(GPIOC,GPIO_Pin_4)
#define CAP_off	GPIO_ResetBits(GPIOC,GPIO_Pin_4)
#define Charge_On	GPIO_SetBits(GPIOA,GPIO_Pin_6)
#define Charge_Off	GPIO_ResetBits(GPIOA,GPIO_Pin_6)


#define POWER_CHARGE_VOL_MIN 18.0f //��ʼ����˥��
#define POWER_CHARGE_VOL_MAX 21.0f //˥����0
//��������
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
	enum POWERSTATE_Typedef PowerState;//��ǰʵ�ʵ�����״̬ cap bat
	enum POWERSTATE_Typedef PowerState_Set;//��������ʱ����������״̬ cap bat
	
    float actual_vol; //ʵ�ʵ��ݵ�ѹ(�˲�����)
	float actual_power;//ʵ�ʹ���
		//��ѹ�ɼ�����
    unsigned int SUM_UCAP;  //���ݵ�ѹ
    unsigned int SUM_UDCDC; //����ѹ
    unsigned int SUM_TEMP;  //�¶�

    float UCAP;
    float UDCDC;
    float temperate;
	
	int MaxChargePower; //����繦�ʣ���С��繦��
	
    float i_set;             //������
	short DAC_Set;
    
	float ina260_vol;//��ina����ĵ��ݵ�ѹУ׼adc������ֵ ��Դ�ǵ���ģʽ��Ч
	
	double change_timer;//��ǰʵ��ʱ��
	//���ò���
	float CAP_AD_L, CAP_AD_H;//capģʽ���л����ݵ���ֵ
	float SUPERPOWER_SAMPLE_RATIO;//����ת��ϵ�����³�Ҫ��һ��
	float max_charge_i;//�������� A
	float Max_change_fre; //��������л�Ƶ�� HZ
	
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

//��������
void HeatControl(void);
void HeatUpdate(void);

//���ͽṹ������
void BuildF105(void);

//������
void PowerControl_task(void *pvParameters);

#endif

