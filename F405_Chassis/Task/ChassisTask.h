#ifndef __CHASSISTASK_H__
#define __CHASSISTASK_H__
#include "main.h"
#include "PowerControlTask.h"
#include "ZeroCheckTask.h"


#define YES 1
#define NO  0


//��������궨��
#define RM3508_CURRENT_RATIO (20.0/16384.0)  
#define RM3508_RPM_RAD (PI/30)


#define RM6020_CURRENT_RATIO (20.0/25000.0)  
#define RM6020_RPM_RAD (PI/30)

#define GM6020_MAX_VOLTAGE 30000 //����͵�ѹֵ,���ֵΪ30000
#define GM6020_MIN_VOLTAGE -GM6020_MAX_VOLTAGE

#define GM6020_MAX_CURRENT 16000
#define GM6020_MIN_CURRENT -GM6020_MAX_CURRENT
#define Vol2Current(x,y) (((x) - (y)*76.0)/1.35f) //���루��ѹ��ת�٣��������Ӧ�ĵ���

//���ʿ��ƺ궨��
#if Robot_ID == 46 //����
#define RM3508_R 0.2314
#define RM3508_K 0.001538
#define RM3508_P0 23.0
//K�Ķ�̬��Χ
#define RM3508_K_MAX 0.00168
#define RM3508_K_MIN 0.00142

#define CAP_C (60/9)  //�������������ֵ

#elif Robot_ID == 44
#define RM3508_R 0.1231
#define RM3508_K 0.001685
#define RM3508_P0 12.75
#define RM3508_K_MAX 0.0017
#define RM3508_K_MIN 0.00145
#define CAP_C (55/9)  //�������������ֵ

#elif Robot_ID == 45 || Robot_ID == 47
#define RM3508_R 0.0893
#define RM3508_K 0.00166
#define RM3508_P0 12.21
#define RM3508_K_MAX 0.001663
#define RM3508_K_MIN 0.001656
#define CAP_C (55/9)  //�������������ֵ
#endif


//����
#define SMALLTOP_CIRCLE   45 //�����ξ���45��  arctan(��/��) ��Ϊͷ��β
#define DEG2R(x) ((x)*PI /180.0f)
#define R2DEG(x) ((x)*180.0f /PI)

void Key_Num_Check(void);
void Chassis_Mode_Special_Control(void);
void Chassis_CurrentPid_Cal(void);
void Current_Filter_Excu(void);
void Current_Set_Jump(void);

void Pid_ChassisWheelInit(void);
void pid_test_cal(void);
void Chassis_task(void *pvParameters);


typedef struct{
	enum POWERSTATE_Typedef PowerState_control;//�û������л�������״̬bat cap halfcap
	enum POWERSTATE_Typedef PowerState_Set;//��������ʱ����������״̬ cap bat
	
	//SD��¼���ܷ�������Ĵ����־λ
	char LowVol_halfcap;//�����ģʽ�µ�����͵�ѹ
	
	
	float actual_i_2;//1KHZ
	float actual_w_i;//1KHZ
	float actual_ina260_power;
	float actual_i_2_ref;//10HZ��������
	float actual_w_i_ref;//10HZ��������
	float actual_referee_power;
	
	float actual_i_2_6020;
	float actual_w_i_6020;
	
	float predict_p0;//��̬���ʣ��Ƕ���ʱ����rm3508_p0������ʱ�����˶��ֹ�����Ϊp0
	float predict_power;//��ǰ������ת����ϵĹ���
	float predict_power_10;//��ǰ������ת����ϵĹ��� 10HZ
	
	float predict_power_old;//��ǰ������ת����ϵĹ���
	float predict_power_old_10;//��ǰ������ת����ϵĹ��� 10HZ
	
	float predict_send_power;//����Ŀ��ת�ٺ�Ԥ��Ĺ���
	float predict_send_power_other;//����Ŀ�������Ԥ��Ĺ���
	
	float set_power;//��̬��������
	float set_superpower;//��繦��
	double k_c;//ת��˥��
	double k_s_min;//ת�����ֵ˥��
	short referee_max_power;//����ϵͳ�����
	
	short remainEnergy;//����ϵͳʣ������
	
	short Current_set[4];//����Ŀ�����
	short Speed_Now[4];//��ǰ���ת��rpm
	short Current_Last[4];//��һʱ�̵������
	char Motor_lost[4];//�����֡
	
	double K[4];//����ת��ת������
	double M[4];//����ת��ת������
	double k_s[4];
	double k_dynamic;//��̬kֵ
	double k_dynamic_1;
	double k_dynamic_5;
	
	
	float capEnergy;//��������
	float Min_capEnergy;//��С��������
	float HalfCAP_Power;
	//��̬���ò���
	
	Pid_Typedef pidChassisWheelSpeed[4];//ת�ٿ���
	
	float No_limited_Power;//û������ʱ�ܵĹ���
	float Add_HalfCAP_Power;//��������ʱ���ܵĹ���
	float P_capEngry;//������������ϵ��
	float Min_capVol;//��С���ݵ�ѹ
	float P_remainEngry;//������������ϵ��
	float Min_remainEnergy;//��Сʣ������
	float error_control;//�������
}Power_Limit_type;




enum STEERING_MODE_Typedef{
	Stright,Small_top_move, Small_top_avoid,Recovery
};

typedef struct {
	float32_t v;//m/s
	float32_t yaw;//rad
}Velocity_vector_type;

typedef struct {
	Velocity_vector_type Velocity_vector;//�ٶ�
	Velocity_vector_type Velocity_vector_last;//��һ���ٶȣ�
}WheelVelocity_type;


typedef struct {
	//�û����ò���
	float SteeringAnglePosition_init[4];//����Ƕȳ�ʼλ��
	short Self_Protect_Limit;			//С����ת������
	enum STEERING_MODE_Typedef Steeringwheel_Mode;//����ģʽ
	
	
	float k_xy,k_w;//�ٶ����
	float sgn;
	float carSpeedw;//Ŀ��ת��
	
	//���ֶ���
	float car_yaw;//ƽ�Ʒ���
	float car_V;//ƽ���ٶ�
	double yaw_diff_rec;//���̺���̨�нǽ���ֵ
	
	
	
	//����ÿ��������ӵ�Ŀ��ֵ
	Velocity_vector_type Velocity_vectors[4];//�ٶ�
	Velocity_vector_type Velocity_vectors_last[4];//��һ���ٶȣ�
	
	
	Pid_Typedef	PIDSteeringCurrent[4];
	Pid_Typedef	PIDSteeringAngleSpeed[4]; //��ǿ���
	Pid_Typedef	PIDSteeringAnglePosition[4];
	float Current_set[4];
	
	//�˸����̵������ֵ
	short WheelCurrentSend[4];
	short SteeringVolSend[4];
	//ͨ����ѹ���ͼ���ĵ�������ֵ
	float SteeringCurrentSend[4];
	
	//���̵������ֵ
	Motor6020Receive_Typedef SteerMotorsCanReceive[4];
	RM820RReceive_Typedef ChassisMotorCanReceive[4];
	//���̵������ֵ
	Motor6020Data_Typedef SteerMotorsDecode[4];
	
	
}chassis_type;//���ֿ��ƽṹ��

void ChassisSteer_cal(chassis_type *chassis_);
void Method_Check(void);
void Chassis_Power_Control_Init(void);
void Chassis_Mode_Special_Control(void);
void controlSteering_wheel(chassis_type *chassis_, float car_yaw, float car_V, float w_speed);
#endif
