#include "main.h"

float Input[4] = {0};
float Output[4] = {0};
#define K_Low_setSteer  0.08

#if Mecanum == 2 //����
#define K_Low_setchassis  0.03   //0.05   ��ֹƵ��f=1000a/2pi=160a
#else
#define K_Low_setchassis  0.01   //0.05   ��ֹƵ��f=1000a/2pi=160a
#endif

#define K_Low_setwheel    0.6

float Flow[4];
const float Gains[6] = {
 0.01016679592,   0.1177062541,   0.3721269369,   0.3721269369,   0.1177062541,
    0.01016679592
};

float Buffer[4][ORDER+1];        //������ʷ����
/**
  * @brief  ���뵱ǰ�����ͣ�����FIR�˲�����
  * @param  Input: �˲�����ĵ�ǰֵ     
  * @retval �˲�������ֵ
  */
void Fir(float Input[],float Output[4])
{
		unsigned int Index;                //�±�����
		for(int i = 0;i < 4;i ++)
		{
			for(Index=ORDER;Index>0;Index--) Buffer[i][Index]=Buffer[i][Index-1];
			Buffer[i][0]=Input[i];
			//�������
			for(Index=0;Index<ORDER+1;Index++)
			{
				Output[i]+=Gains[Index]*Buffer[i][Index];
			}
			Flow[i] = Output[i];
			Output[i] = 0;
		}
        
}

/**
  * @brief  �����ٶ��趨�ĵ�ͨ�˲�
  * @param  
  * @retval �˲�������ֵ
  */
float LowPass_SetChassis(float old,float In)
{
    return (1-K_Low_setchassis)*(old)+K_Low_setchassis*In;   
}

/**
  * @brief  ���Ӷ����趨�ĵ�ͨ�˲�
  * @param  
  * @retval �˲�������ֵ
  */
float LowPass_SetSteer(float old,float In)
{
    return (1-K_Low_setSteer)*(old)+K_Low_setSteer*In;   
}

/**
* @brief  �������ӵ������͵ĵ�ͨ�˲�
  * @param  
  * @retval �˲�������ֵ
  */
float LowPass_SetWheel(float In,float past)
{
  past=(1-K_Low_setwheel)*past+K_Low_setwheel*In;
  return past;	
}

/**
  * @brief  ���ʸ��µ�ͨ
  * @param  
  * @retval �˲�������ֵ
  */
float K_Low_SetK = 0.5;
float LowPass_SetK(float old,float In)
{
    return (1-K_Low_SetK)*(old)+K_Low_SetK*In;   
}

float K_Low_SetK1 = 0.005;
float LowPass_SetK1(float old,float In)
{
    return (1-K_Low_SetK1)*(old)+K_Low_SetK1*In;   
}


float K_Low_SetK5 = 0.5;
float LowPass_SetK5(float old,float In)
{
    return (1-K_Low_SetK5)*(old)+K_Low_SetK5*In;   
}