/**
 ******************************************************************************
 * File Name          : TIM.c
 * Description        : This file provides code for the configuration
 *                      of the TIM instances.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN 0 */
#define T 0.005f
extern Pid_Typedef Temp;
/* USER CODE END 0 */

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* TIM2 init function */
void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 47;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim2);
}
/* TIM3 init function */
void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 47;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *tim_baseHandle)
{

  if (tim_baseHandle->Instance == TIM2)
  {
    /* USER CODE BEGIN TIM2_MspInit 0 */

    /* USER CODE END TIM2_MspInit 0 */
    /* TIM2 clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();
    /* USER CODE BEGIN TIM2_MspInit 1 */

    /* USER CODE END TIM2_MspInit 1 */
  }
  else if (tim_baseHandle->Instance == TIM3)
  {
    /* USER CODE BEGIN TIM3_MspInit 0 */

    /* USER CODE END TIM3_MspInit 0 */
    /* TIM3 clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();
    /* USER CODE BEGIN TIM3_MspInit 1 */

    /* USER CODE END TIM3_MspInit 1 */
  }
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (timHandle->Instance == TIM2)
  {
    /* USER CODE BEGIN TIM2_MspPostInit 0 */

    /* USER CODE END TIM2_MspPostInit 0 */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM2 GPIO Configuration
    PA2     ------> TIM2_CH3
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USER CODE BEGIN TIM2_MspPostInit 1 */

    /* USER CODE END TIM2_MspPostInit 1 */
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *tim_baseHandle)
{

  if (tim_baseHandle->Instance == TIM2)
  {
    /* USER CODE BEGIN TIM2_MspDeInit 0 */

    /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();
    /* USER CODE BEGIN TIM2_MspDeInit 1 */

    /* USER CODE END TIM2_MspDeInit 1 */
  }
  else if (tim_baseHandle->Instance == TIM3)
  {
    /* USER CODE BEGIN TIM3_MspDeInit 0 */

    /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();
    /* USER CODE BEGIN TIM3_MspDeInit 1 */

    /* USER CODE END TIM3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void delay_us(short us)
{
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  __HAL_TIM_ENABLE(&htim3);
  while (__HAL_TIM_GET_COUNTER(&htim3) < us)
    ;
}

/************************************************************************************/

//陀螺仪纯粹积分得到的角度
float pure_gyro_yaw, pure_gyro_yaw_LP;
float pure_gyro_roll;

//加速度计计算出来的角度，假设只有重力加速度
float pure_acc_pitch, pure_acc_pitch_last;
float pure_acc_roll, pure_acc_roll_last;
float acc_inti_norm = 0;
float acc_norm;

//两轴角加速度,用于互补滤波补偿
float pitch_w_acc;
float last_w_pitch;
float k_com = 0.002f;

/*************************************************************************************/

// yaw轴互补滤波参数
float k_filter = 0.02f;    //互补滤波系数
float combYawAngle = 0.0f; //互补滤波角度

// pitch互补滤波参数
float k_filter_pitch_roll = 0.0f;
float combAngle_pitch, comAngle_pitch_LP;

// roll轴互补滤波参数
float combAngle_roll = 0.0f;

/***********************************************************************************/

//卡尔曼滤波结果
float yaw_angle, yaw_angle_dot;
float pitch_angle, pitch_angle_dot;
float roll_angle, roll_angle_dot;

/**********************************************************************************/
//动态静态切换参数
double spliceThrehold_pitch = 0.06; //步兵
float splicePitch = 0;              //组合角度
float spliceRoll = 0;               //组合角度
double last_pitch_angle = 0;
double last_pitch_angle2 = 0;
short spliceFlag_pitch = 0;
short spliceCnt_pitch = 0;

double spliceThrehold_yaw = 0.05; //
float spliceYaw = 0;              //组合角度
double last_yaw_angle = 0;
double last_yaw_angle2 = 0;
short spliceFlag = 0;
short spliceCnt = 0;
short breakCnt = 0;
// yaw 时间参数
short break_gyro_yaw = 50;   //避免抖动
short comb_time_yaw = 200;   //进入互补滤波范围
float kalman_time_yaw = 500; //进入卡尔曼滤波范围
float yaw_mag_bias = 0;
/***********************************************************************************/

float icm_gyro[3];
float icm_gyro_LP[3];
float yaw_w_cal, pitch_w_cal, roll_w_cal, yaw_w_cal_LP;
float icm_accel[3];

short gyro_raw[3];
short acc_raw[3];

/***********************************************************************************/

float inte_velocity_y, inte_velocity_x;

/***********************************************************************************/
int8_t angle_init_flag = 1;
extern int8_t IMU_init_flag;
extern float gyro_bias[3];
extern float tempeature;
int32_t ms4 = 0;
int32_t symbol = 0;

int32_t testcan1 = 0;
int32_t testcan2 = 0;

void HAL_SYSTICK_Callback(void)
{
  if (IMU_init_flag == 1)
  {
    static int blink = 0;
    ms4++;
    blink++; //闪烁表示正常工作中
    if (blink > 500)
      LED_ON;
    if (blink > 1000)
    {
      LED_OFF;
      blink = 0;
    }
    //	if(ms4 >= 2)//每4ms解算一次 250Hz
    //	{
    //	  ms4 = 0;
    //      ms4++;
    icm20602_get_gyro_adc(gyro_raw);
    icm20602_get_accel_adc(acc_raw);

    for (uint8_t index = 0; index < 3; index++)
    {
      icm_gyro[index] = (float)gyro_raw[index] * 1.0f / 16.3835f;
      icm_accel[index] = (float)acc_raw[index] / 8192.0f * 9.80f;
    }
    acc_norm = sqrt(icm_accel[0] * icm_accel[0] + icm_accel[1] * icm_accel[1] + icm_accel[2] * icm_accel[2]);
    //		icm20602_get_accel(icm_accel);//读取加速度计数据
    //		icm20602_get_gyro(icm_gyro); //读取陀螺仪数据

    //		//计算加速度计测量角度
    //    pure_acc_pitch = -atan2((double)icm_accel[0], sqrt(icm_accel[2]*icm_accel[2]+icm_accel[1]*icm_accel[1])) / 3.1415926f * 180.0f;
    //    pure_acc_roll  =  atan2((double)icm_accel[1], icm_accel[2]) / 3.1415926f * 180.0f;
    //
    //
    //
    //  if(angle_init_flag)
    //	{
    //    combAngle_pitch=splicePitch=pitch_angle = pure_acc_pitch_last = pure_acc_pitch;
    //		combAngle_roll=spliceRoll =pure_gyro_roll =roll_angle  = pure_acc_roll_last = pure_acc_roll ;
    //		angle_init_flag=0;
    //  }

    //		acc_norm = icm_accel[0]*icm_accel[0]+icm_accel[1]*icm_accel[1]+icm_accel[2]*icm_accel[2];
    //		if((acc_inti_norm<90)||(acc_inti_norm>105))
    //		{//通常是90-105
    //			acc_inti_norm = acc_norm;
    //		}
    //
    //		k_filter_pitch_roll = -k_com*fabsf(acc_norm-acc_inti_norm ) +0.02f ;//+ k_com_w*fabsf(pitch_w_acc);//自适应参数
    //
    //		k_filter_pitch_roll = LIMIT_MAX_MIN(k_filter_pitch_roll,0.02f,0.0f);
    //
    //		//互补滤波
    //		combAngle_pitch = (combAngle_pitch + icm_gyro[1]*T/3.1415926f * 180.0f)*(1 - k_filter_pitch_roll) + pure_acc_pitch*k_filter_pitch_roll;
    ////	combAngle_roll  = (combAngle_roll  + icm_gyro[0]*T/3.1415926f * 180.0f)*(1 - k_filter_pitch_roll) + pure_acc_roll *k_filter_pitch_roll;
    //
    //
    //    yaw_w_cal = icm_gyro[2] *cos(combAngle_pitch*3.1415926f/180.0f)* cos(combAngle_roll*3.1415926f/180.0f) + icm_gyro[0] * (-sin(combAngle_pitch*3.1415926f/180.0f)) + \
//                          icm_gyro[1] * sin(combAngle_roll*3.1415926f/180.0f)*cos(combAngle_pitch*3.1415926f/180.0f);
    //

    //
    //		pure_gyro_yaw   += yaw_w_cal * T / 3.1415926f * 180.0f;
    //		//yaw轴互补滤波，为了保持连续性，角度采取增量形式
    //
    //
    //		combYawAngle = combYawAngle + yaw_w_cal*T/3.1415926f * 180.0f*(1 - k_filter);
    //
    //
    //		//卡尔曼，调的不太理想，暂时先用互补滤波
    //		pitch_angle_dot=Kalman_Filter_pitch_roll(pure_acc_pitch, icm_gyro[1],&pitch_angle);
    ////		roll_angle_dot =Kalman_Filter_pitch_roll(pure_acc_roll , icm_gyro[0],&roll_angle);
    //		yaw_angle_dot  =Kalman_Filter_yaw(pure_gyro_yaw, icm_gyro[2],&yaw_angle);////没啥意义，自我安慰

    //
    //	//动态时YAW轴采用校准后的陀螺仪数据，静止时采用卡尔曼滤波后的数据
    //		if((yaw_w_cal>spliceThrehold_yaw)||(yaw_w_cal<-spliceThrehold_yaw))
    //		{//如果在动态情况下
    //			if(breakCnt>break_gyro_yaw)       //旋转一段时间后完全用陀螺仪数据
    //			{
    //				spliceFlag = 1;
    //				spliceYaw += yaw_w_cal * T / 3.1415926f * 180.0f;
    //				spliceCnt = 0;
    //			}
    //			else                             //旋转较短一段时间内，使用互补滤波数据
    //			{
    //				spliceFlag = -1;
    //				spliceYaw += (combYawAngle - last_yaw_angle);
    //			}
    //			breakCnt ++;
    //		}
    //		else
    //		{
    //			//静态情况下
    //			spliceCnt ++;
    //			if(spliceCnt>comb_time_yaw&&spliceCnt<kalman_time_yaw)//
    //			{//维持一段时间静态使用互补数据避免频繁跳动
    //				spliceFlag = -1;
    //				spliceYaw += (combYawAngle - last_yaw_angle);
    //			}
    //			else if(spliceCnt>=kalman_time_yaw)
    //			{//长时间静态则使用卡尔曼滤波数据
    //				spliceFlag = -2;
    //				spliceYaw += (yaw_angle - last_yaw_angle2);
    //			 //spliceYaw += yaw_angle_dot;
    //			}
    //			else
    //			{//短时间静态先用陀螺仪数据
    //				spliceFlag = 1;
    //				spliceYaw += yaw_w_cal * T / 3.1415926f * 180.0f;
    //
    //			}
    //			breakCnt = 0;
    //		}
    //
    //		if( spliceYaw > 180.0F)
    //		{
    //		 spliceYaw -= 360.0f;
    //		}
    //		else if( spliceYaw < -180.0F)
    //		{
    //		 spliceYaw += 360.0f;
    //		}
    //
    //		last_yaw_angle = combYawAngle;
    //		last_yaw_angle2 = yaw_angle;

    // CANSendMag(combAngle_pitch, icm_gyro[1],0x100);//避免加速度计带来的点头现象
    // CANSendMag( spliceYaw, yaw_w_cal,0x101);//未经过过零处理

    testcan1 = CANSendMagRaw(gyro_raw, 0x100);
    testcan2 = CANSendMagRaw(acc_raw, 0x101);

    //		CANSendMag(pure_acc_roll , icm_gyro[0],0x133);
    //		CANSendMag(tempeature     ,  tempeature               ,0x011);

    //	}
  }
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
