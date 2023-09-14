/**********************************************************************************************************
 * @�ļ�     madgwick.c
 * @˵��     ͨ�������Ƽ���Ƕ�
 * @�汾  	 V1.0
 * @����
 * @����     2019.01


 * @�ļ�     Kalman_Filter.c
 * @˵��     �������˲�
 * @�汾  	 V2.0
 * @����     ע�����޶�
 * @����     2022.01
**********************************************************************************************************/
#include "main.h"

float dt = 0.005;

float Q_angle_pitch_roll = 0.001; // ����������Э����
float Q_gyro_pitch_roll = 0.003;  // 1 ����������Э���� ����������Э����Ϊһ��һ�����о���
float R_angle_pitch_roll = 10;	  // 5000;// ����������Э���� �Ȳ���ƫ��

char C_0_pitch_roll = 1; //�۲���󣬼�״̬ת�ƾ���ϵ��

float Q_bias_pitch_roll, Angle_err_pitch_roll;			//����
float PCt_0_pitch_roll, PCt_1_pitch_roll, E_pitch_roll; //�в���״̬ת�ƺ���������Э����
float K_0_pitch_roll, K_1_pitch_roll, t_0_pitch_roll, t_1_pitch_roll;
float PP_pitch_roll[2][2] = {{0, 0}, {0, 0}};
extern uint8_t alige_flag;
float PitchRoll_angle_est;
float K_bias = 1;
extern float pure_gyro_pitch;
extern float splicePitch; //��ϽǶ�
extern float acc_inti_norm;
extern float icm_accel[3]; //���ٶȼ�����

/**************************************************************************
�������ܣ����׿������˲�
��ڲ��������ٶȵõ��ĽǶ�ֵ�����ٶ�
����  ֵ����
**************************************************************************/
float Kalman_Filter_pitch_roll(float Accel, float Gyro, float *angle)
{

	PitchRoll_angle_est = *angle + (Gyro - Q_bias_pitch_roll) * dt / 3.1415926f * 180.0f; //�������

	PP_pitch_roll[0][0] += (Q_angle_pitch_roll - PP_pitch_roll[0][1] - PP_pitch_roll[1][0] + PP_pitch_roll[1][1] * dt) * dt; // Pk-����������Э����
	PP_pitch_roll[0][1] -= PP_pitch_roll[1][1] * dt;																		 // =����������Э����
	PP_pitch_roll[1][0] -= PP_pitch_roll[1][1] * dt;
	PP_pitch_roll[1][1] += Q_gyro_pitch_roll * dt;

	Angle_err_pitch_roll = Accel - PitchRoll_angle_est; //�в�

	PCt_0_pitch_roll = C_0_pitch_roll * PP_pitch_roll[0][0]; //׼�����²в�Э����
	PCt_1_pitch_roll = C_0_pitch_roll * PP_pitch_roll[1][0];

	E_pitch_roll = R_angle_pitch_roll + C_0_pitch_roll * PCt_0_pitch_roll; //�в�Э����

	K_0_pitch_roll = PCt_0_pitch_roll / E_pitch_roll; //���¿���������
	K_1_pitch_roll = PCt_1_pitch_roll / E_pitch_roll;

	t_0_pitch_roll = PCt_0_pitch_roll; //׼�����º������Э����
	t_1_pitch_roll = C_0_pitch_roll * PP_pitch_roll[0][1];

	PP_pitch_roll[0][0] -= K_0_pitch_roll * t_0_pitch_roll; //����������Э����
	PP_pitch_roll[0][1] -= K_0_pitch_roll * t_1_pitch_roll;
	PP_pitch_roll[1][0] -= K_1_pitch_roll * t_0_pitch_roll;
	PP_pitch_roll[1][1] -= K_1_pitch_roll * t_1_pitch_roll;

	*angle = PitchRoll_angle_est + K_0_pitch_roll * Angle_err_pitch_roll; //�Ƕ����ź������
	Q_bias_pitch_roll += K_1_pitch_roll * Angle_err_pitch_roll;			  //���ٶ�Ʈ�������������
	return (Gyro - Q_bias_pitch_roll);									  //���ֵ(�������)���ٶ�
}

float dt_yaw = 0.004;
///////////////////////////////////////////////////////
float Q_angle_yaw = 0.1; // ����������Э����
float Q_gyro_yaw = 0.02; // 0.003 ����������Э���� ����������Э����Ϊһ��һ�����о���
float R_angle_yaw = 0.7; // ����������Э���� �Ȳ���ƫ��

char C_0_yaw = 1;
float Q_bias_yaw, Angle_err_yaw, Bias_err_yaw;
float PCt_0_yaw, PCt_1_yaw, E_yaw;
float K_0_yaw, K_1_yaw, t_0_yaw, t_1_yaw;
float PP_yaw[2][2] = {{0, 0}, {0, 0}};
extern float pure_gyro_pitch;
float test_bias = 0;
/**************************************************************************
�������ܣ����׿������˲�
��ڲ��������ٶȵõ��ĽǶ�ֵ�����ٶ�
����  ֵ����
**************************************************************************/
float Kalman_Filter_yaw(float Accel, float Gyro, float *angle)
{
	// test_bias = (Gyro - Q_bias_yaw)*1000;
	*angle += (Gyro - Q_bias_yaw) * dt_yaw / 3.1415926f * 180.0f; //�������

	PP_yaw[0][0] += (Q_angle_yaw - PP_yaw[0][1] - PP_yaw[1][0] + PP_yaw[1][1] * dt_yaw) * dt_yaw; // Pk-����������Э����΢�ֵĻ���
	PP_yaw[0][1] -= PP_yaw[1][1] * dt_yaw;														  // =����������Э����
	PP_yaw[1][0] -= PP_yaw[1][1] * dt_yaw;
	PP_yaw[1][1] += Q_gyro_yaw * dt_yaw;

	Bias_err_yaw = Accel - *angle - Angle_err_yaw; //�в��ƫ��
	Angle_err_yaw = Accel - *angle;				   //�в�    zk-�������

	PCt_0_yaw = C_0_yaw * PP_yaw[0][0];
	PCt_1_yaw = C_0_yaw * PP_yaw[1][0];

	E_yaw = R_angle_yaw + C_0_yaw * PCt_0_yaw;

	K_0_yaw = PCt_0_yaw / E_yaw;
	K_1_yaw = PCt_1_yaw / E_yaw;

	t_0_yaw = PCt_0_yaw;
	t_1_yaw = C_0_yaw * PP_yaw[0][1];

	PP_yaw[0][0] -= K_0_yaw * t_0_yaw; //����������Э����
	PP_yaw[0][1] -= K_0_yaw * t_1_yaw;
	PP_yaw[1][0] -= K_1_yaw * t_0_yaw;
	PP_yaw[1][1] -= K_1_yaw * t_1_yaw;

	*angle += K_0_yaw * Angle_err_yaw;			   //�������
	Q_bias_yaw += K_bias * K_1_yaw * Bias_err_yaw; //�������
	return (Gyro - Q_bias_yaw);					   //���ֵ(�������)��΢��=���ٶ�
}
