/**********************************************************************************************************
 * @文件     madgwick.c
 * @说明     通过磁力计计算角度
 * @版本  	 V1.0
 * @作者
 * @日期     2019.01


 * @文件     Kalman_Filter.c
 * @说明     卡尔曼滤波
 * @版本  	 V2.0
 * @作者     注释与修订
 * @日期     2022.01
**********************************************************************************************************/
#include "main.h"

float dt = 0.005;

float Q_angle_pitch_roll = 0.001; // 过程噪声的协方差
float Q_gyro_pitch_roll = 0.003;  // 1 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
float R_angle_pitch_roll = 10;	  // 5000;// 测量噪声的协方差 既测量偏差

char C_0_pitch_roll = 1; //观测矩阵，即状态转移矩阵系数

float Q_bias_pitch_roll, Angle_err_pitch_roll;			//后验
float PCt_0_pitch_roll, PCt_1_pitch_roll, E_pitch_roll; //残差与状态转移后的先验误差协方差
float K_0_pitch_roll, K_1_pitch_roll, t_0_pitch_roll, t_1_pitch_roll;
float PP_pitch_roll[2][2] = {{0, 0}, {0, 0}};
extern uint8_t alige_flag;
float PitchRoll_angle_est;
float K_bias = 1;
extern float pure_gyro_pitch;
extern float splicePitch; //组合角度
extern float acc_inti_norm;
extern float icm_accel[3]; //加速度计数据

/**************************************************************************
函数功能：简易卡尔曼滤波
入口参数：加速度得到的角度值、角速度
返回  值：无
**************************************************************************/
float Kalman_Filter_pitch_roll(float Accel, float Gyro, float *angle)
{

	PitchRoll_angle_est = *angle + (Gyro - Q_bias_pitch_roll) * dt / 3.1415926f * 180.0f; //先验估计

	PP_pitch_roll[0][0] += (Q_angle_pitch_roll - PP_pitch_roll[0][1] - PP_pitch_roll[1][0] + PP_pitch_roll[1][1] * dt) * dt; // Pk-先验估计误差协方差
	PP_pitch_roll[0][1] -= PP_pitch_roll[1][1] * dt;																		 // =先验估计误差协方差
	PP_pitch_roll[1][0] -= PP_pitch_roll[1][1] * dt;
	PP_pitch_roll[1][1] += Q_gyro_pitch_roll * dt;

	Angle_err_pitch_roll = Accel - PitchRoll_angle_est; //残差

	PCt_0_pitch_roll = C_0_pitch_roll * PP_pitch_roll[0][0]; //准备更新残差协方差
	PCt_1_pitch_roll = C_0_pitch_roll * PP_pitch_roll[1][0];

	E_pitch_roll = R_angle_pitch_roll + C_0_pitch_roll * PCt_0_pitch_roll; //残差协方差

	K_0_pitch_roll = PCt_0_pitch_roll / E_pitch_roll; //更新卡尔曼增益
	K_1_pitch_roll = PCt_1_pitch_roll / E_pitch_roll;

	t_0_pitch_roll = PCt_0_pitch_roll; //准备更新后验估计协方差
	t_1_pitch_roll = C_0_pitch_roll * PP_pitch_roll[0][1];

	PP_pitch_roll[0][0] -= K_0_pitch_roll * t_0_pitch_roll; //后验估计误差协方差
	PP_pitch_roll[0][1] -= K_0_pitch_roll * t_1_pitch_roll;
	PP_pitch_roll[1][0] -= K_1_pitch_roll * t_0_pitch_roll;
	PP_pitch_roll[1][1] -= K_1_pitch_roll * t_1_pitch_roll;

	*angle = PitchRoll_angle_est + K_0_pitch_roll * Angle_err_pitch_roll; //角度最优后验估计
	Q_bias_pitch_roll += K_1_pitch_roll * Angle_err_pitch_roll;			  //角速度飘移量，后验估计
	return (Gyro - Q_bias_pitch_roll);									  //输出值(后验估计)角速度
}

float dt_yaw = 0.004;
///////////////////////////////////////////////////////
float Q_angle_yaw = 0.1; // 过程噪声的协方差
float Q_gyro_yaw = 0.02; // 0.003 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
float R_angle_yaw = 0.7; // 测量噪声的协方差 既测量偏差

char C_0_yaw = 1;
float Q_bias_yaw, Angle_err_yaw, Bias_err_yaw;
float PCt_0_yaw, PCt_1_yaw, E_yaw;
float K_0_yaw, K_1_yaw, t_0_yaw, t_1_yaw;
float PP_yaw[2][2] = {{0, 0}, {0, 0}};
extern float pure_gyro_pitch;
float test_bias = 0;
/**************************************************************************
函数功能：简易卡尔曼滤波
入口参数：加速度得到的角度值、角速度
返回  值：无
**************************************************************************/
float Kalman_Filter_yaw(float Accel, float Gyro, float *angle)
{
	// test_bias = (Gyro - Q_bias_yaw)*1000;
	*angle += (Gyro - Q_bias_yaw) * dt_yaw / 3.1415926f * 180.0f; //先验估计

	PP_yaw[0][0] += (Q_angle_yaw - PP_yaw[0][1] - PP_yaw[1][0] + PP_yaw[1][1] * dt_yaw) * dt_yaw; // Pk-先验估计误差协方差微分的积分
	PP_yaw[0][1] -= PP_yaw[1][1] * dt_yaw;														  // =先验估计误差协方差
	PP_yaw[1][0] -= PP_yaw[1][1] * dt_yaw;
	PP_yaw[1][1] += Q_gyro_yaw * dt_yaw;

	Bias_err_yaw = Accel - *angle - Angle_err_yaw; //残差的偏移
	Angle_err_yaw = Accel - *angle;				   //残差    zk-先验估计

	PCt_0_yaw = C_0_yaw * PP_yaw[0][0];
	PCt_1_yaw = C_0_yaw * PP_yaw[1][0];

	E_yaw = R_angle_yaw + C_0_yaw * PCt_0_yaw;

	K_0_yaw = PCt_0_yaw / E_yaw;
	K_1_yaw = PCt_1_yaw / E_yaw;

	t_0_yaw = PCt_0_yaw;
	t_1_yaw = C_0_yaw * PP_yaw[0][1];

	PP_yaw[0][0] -= K_0_yaw * t_0_yaw; //后验估计误差协方差
	PP_yaw[0][1] -= K_0_yaw * t_1_yaw;
	PP_yaw[1][0] -= K_1_yaw * t_0_yaw;
	PP_yaw[1][1] -= K_1_yaw * t_1_yaw;

	*angle += K_0_yaw * Angle_err_yaw;			   //后验估计
	Q_bias_yaw += K_bias * K_1_yaw * Bias_err_yaw; //后验估计
	return (Gyro - Q_bias_yaw);					   //输出值(后验估计)的微分=角速度
}
