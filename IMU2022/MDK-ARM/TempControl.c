#include "main.h"
#define TEMP_CONTROL_LP_COEF 0.05
Pid_Typedef Temp;
extern TIM_HandleTypeDef htim2;
short SetPWM;
void Temp_init()
{
	Temp.SetPoint = 45.0f;
	Temp.P = 4000.0f;
	Temp.I = 1.9f;
	Temp.D = 10.0f;
	Temp.I_L = 1.5f;
	Temp.I_U = 3.0f;
	Temp.RC_DF = 0.05f;
	Temp.IMax = 2500.0f;
	Temp.OutMax = 9600.0f;
	Temp.ActualValue = 25.0f;
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
}

void TempControl()
{
	Temp.ActualValue = icm20602_get_temp();
	PID_Calc(&Temp);
	if (Temp.Out < 0.0f)
	{
		Temp.Out = 0.0f;
	}
	SetPWM = (short)Temp.Out;
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, SetPWM);
}

/**********************************************************************************************************
 *�� �� ��: PID_Calc
 *����˵��: PID+�����Ż�
 *��    ��: PID_Struct *P  PID�����ṹ��
 *        ActualValue    PID���㷴����
 *�� �� ֵ: PID�����������ֵ
 **********************************************************************************************************/
float PID_Calc(Pid_Typedef *P)
{
	P->LastError = P->PreError;
	P->PreError = P->SetPoint - P->ActualValue;
	if ((fabs(P->PreError) < P->DeadZone)) //��������
	{
		P->PreError = 0.0f;
	}

	//΢������
	float DM = P->D * (P->Out - P->Out_last); //΢������

	//���ٻ���   (���ַ���)
	if (fabs(P->PreError) < P->I_L)
	{

		P->SumError += (P->PreError + P->LastError) / 2;
		P->SumError = LIMIT_MAX_MIN(P->SumError, P->IMax, -P->IMax);
	}
	else if (fabs(P->PreError) < P->I_U)
	{
		//���λ���
		P->SumError += (P->PreError + P->LastError) / 2 * (P->PreError - P->I_L) / (P->I_U - P->I_L);
		P->SumError = LIMIT_MAX_MIN(P->SumError, P->IMax, -P->IMax);
	}

	P->POut = P->P * P->PreError;

	P->IOut = P->I * P->SumError;

	//����ȫ΢��
	P->DOut_last = P->DOut;
	P->DOut = DM * P->RC_DF + P->DOut_last * (1 - P->RC_DF);

	P->Out_last = P->Out;
	P->Out = LIMIT_MAX_MIN(P->POut + P->IOut + P->DOut, P->OutMax, -P->OutMax);

	return P->Out;
}
