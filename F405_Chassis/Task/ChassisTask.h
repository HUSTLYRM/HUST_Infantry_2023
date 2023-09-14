#ifndef __CHASSISTASK_H__
#define __CHASSISTASK_H__
#include "main.h"
#include "PowerControlTask.h"
#include "ZeroCheckTask.h"


#define YES 1
#define NO  0


//电机参数宏定义
#define RM3508_CURRENT_RATIO (20.0/16384.0)  
#define RM3508_RPM_RAD (PI/30)


#define RM6020_CURRENT_RATIO (20.0/25000.0)  
#define RM6020_RPM_RAD (PI/30)

#define GM6020_MAX_VOLTAGE 30000 //最大发送电压值,最大值为30000
#define GM6020_MIN_VOLTAGE -GM6020_MAX_VOLTAGE

#define GM6020_MAX_CURRENT 16000
#define GM6020_MIN_CURRENT -GM6020_MAX_CURRENT
#define Vol2Current(x,y) (((x) - (y)*76.0)/1.35f) //输入（电压，转速），输出对应的电流

//功率控制宏定义
#if Robot_ID == 46 //舵轮
#define RM3508_R 0.2314
#define RM3508_K 0.001538
#define RM3508_P0 23.0
//K的动态范围
#define RM3508_K_MAX 0.00168
#define RM3508_K_MIN 0.00142

#define CAP_C (60/9)  //超级电容组的容值

#elif Robot_ID == 44
#define RM3508_R 0.1231
#define RM3508_K 0.001685
#define RM3508_P0 12.75
#define RM3508_K_MAX 0.0017
#define RM3508_K_MIN 0.00145
#define CAP_C (55/9)  //超级电容组的容值

#elif Robot_ID == 45 || Robot_ID == 47
#define RM3508_R 0.0893
#define RM3508_K 0.00166
#define RM3508_P0 12.21
#define RM3508_K_MAX 0.001663
#define RM3508_K_MIN 0.001656
#define CAP_C (55/9)  //超级电容组的容值
#endif


//舵轮
#define SMALLTOP_CIRCLE   45 //正方形就是45°  arctan(长/宽) 长为头到尾
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
	enum POWERSTATE_Typedef PowerState_control;//用户案件切换的能量状态bat cap halfcap
	enum POWERSTATE_Typedef PowerState_Set;//被动电容时期望的能量状态 cap bat
	
	//SD记录可能发生问题的错误标志位
	char LowVol_halfcap;//半电容模式下低于最低电压
	
	
	float actual_i_2;//1KHZ
	float actual_w_i;//1KHZ
	float actual_ina260_power;
	float actual_i_2_ref;//10HZ采样数据
	float actual_w_i_ref;//10HZ采样数据
	float actual_referee_power;
	
	float actual_i_2_6020;
	float actual_w_i_6020;
	
	float predict_p0;//静态功率，非舵轮时仅是rm3508_p0，舵轮时叠加了舵轮功率作为p0
	float predict_power;//当前电流和转速拟合的功率
	float predict_power_10;//当前电流和转速拟合的功率 10HZ
	
	float predict_power_old;//当前电流和转速拟合的功率
	float predict_power_old_10;//当前电流和转速拟合的功率 10HZ
	
	float predict_send_power;//发送目标转速后预测的功率
	float predict_send_power_other;//发送目标电流后预测的功率
	
	float set_power;//动态功率上限
	float set_superpower;//充电功率
	double k_c;//转速衰减
	double k_s_min;//转速最大值衰减
	short referee_max_power;//裁判系统最大功率
	
	short remainEnergy;//裁判系统剩余能量
	
	short Current_set[4];//求解的目标电流
	short Speed_Now[4];//当前电机转速rpm
	short Current_Last[4];//上一时刻电机电流
	char Motor_lost[4];//电机丢帧
	
	double K[4];//电流转速转换常数
	double M[4];//电流转速转换常数
	double k_s[4];
	double k_dynamic;//动态k值
	double k_dynamic_1;
	double k_dynamic_5;
	
	
	float capEnergy;//电容能量
	float Min_capEnergy;//最小电容能量
	float HalfCAP_Power;
	//静态配置参数
	
	Pid_Typedef pidChassisWheelSpeed[4];//转速控制
	
	float No_limited_Power;//没有限制时跑的功率
	float Add_HalfCAP_Power;//被动电容时多跑的功率
	float P_capEngry;//电容能量利用系数
	float Min_capVol;//最小电容电压
	float P_remainEngry;//缓冲能量利用系数
	float Min_remainEnergy;//最小剩余能量
	float error_control;//控制误差
}Power_Limit_type;




enum STEERING_MODE_Typedef{
	Stright,Small_top_move, Small_top_avoid,Recovery
};

typedef struct {
	float32_t v;//m/s
	float32_t yaw;//rad
}Velocity_vector_type;

typedef struct {
	Velocity_vector_type Velocity_vector;//速度
	Velocity_vector_type Velocity_vector_last;//上一次速度；
}WheelVelocity_type;


typedef struct {
	//用户配置参数
	float SteeringAnglePosition_init[4];//舵向角度初始位置
	short Self_Protect_Limit;			//小陀螺转速限制
	enum STEERING_MODE_Typedef Steeringwheel_Mode;//底盘模式
	
	
	float k_xy,k_w;//速度配比
	float sgn;
	float carSpeedw;//目标转速
	
	//舵轮独有
	float car_yaw;//平移方向
	float car_V;//平移速度
	double yaw_diff_rec;//底盘和云台夹角接收值
	
	
	
	//设置每组底盘轮子的目标值
	Velocity_vector_type Velocity_vectors[4];//速度
	Velocity_vector_type Velocity_vectors_last[4];//上一次速度；
	
	
	Pid_Typedef	PIDSteeringCurrent[4];
	Pid_Typedef	PIDSteeringAngleSpeed[4]; //舵角控制
	Pid_Typedef	PIDSteeringAnglePosition[4];
	float Current_set[4];
	
	//八个底盘电机发送值
	short WheelCurrentSend[4];
	short SteeringVolSend[4];
	//通过电压发送计算的电流控制值
	float SteeringCurrentSend[4];
	
	//底盘电机接收值
	Motor6020Receive_Typedef SteerMotorsCanReceive[4];
	RM820RReceive_Typedef ChassisMotorCanReceive[4];
	//底盘电机解码值
	Motor6020Data_Typedef SteerMotorsDecode[4];
	
	
}chassis_type;//舵轮控制结构体

void ChassisSteer_cal(chassis_type *chassis_);
void Method_Check(void);
void Chassis_Power_Control_Init(void);
void Chassis_Mode_Special_Control(void);
void controlSteering_wheel(chassis_type *chassis_, float car_yaw, float car_V, float w_speed);
#endif
