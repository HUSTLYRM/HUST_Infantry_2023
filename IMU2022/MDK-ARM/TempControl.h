#ifndef _TEMPCONTROL_H_
#define _TEMPCONTROL_H_


typedef struct PID{
		float SetPoint;			//�趨Ŀ��ֵ
	
		float ActualValue;  //ʵ��ֵ

    float DeadZone;
		float P;						//��������
		float I;						//���ֳ���
		float D;						//΢�ֳ���
		
		float LastError;		//ǰ�����
		float PreError;			//��ǰ���
		float SumError;			//�������
	
		float IMax;					//��������
		
		float POut;					//�������
		float IOut;					//�������
		float DOut;					//΢�����
	  float DOut_last;    //��һ��΢�����
		float OutMax;       //�޷�
	  float Out;          //�����
		float Out_last;     //��һ�����
		
		float I_U;          //���ٻ�������
		float I_L;          //���ٻ�������
		
		float RC_DF;        //����ȫ΢���˲�ϵ��
		
}Pid_Typedef;

float PID_Calc(Pid_Typedef * P);
void Temp_init();
void TempControl();
#endif