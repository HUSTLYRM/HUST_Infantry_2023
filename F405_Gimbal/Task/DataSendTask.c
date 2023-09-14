/**********************************************************************************************************
 * @文件     DataSendTask.c
 * @说明     数据发送
 * @版本  	 V1.0
 * @作者     黄志雄
 * @日期     2019.10
 **********************************************************************************************************/
/**********************************************************************************************************
 * @文件     DataSendTask.c
 * @说明     数据发送
 * @版本  	 V2.0
 * @作者     戴军
 * @日期     2022.3
 **********************************************************************************************************/
#include "main.h"
/*--------------内部变量-------------------*/
F405_typedef F405;
/*--------------外部变量-------------------*/
extern Gimbal_Typedef Gimbal;
extern short PC_Sendflag;
extern Status_t Status;
extern F105_Typedef F105;
extern unsigned char magazineState;
extern float Theta_chassis;
/**********************************************************************************************************
 *函 数 名: ChassisCan1Send
 *功能说明: 把xyw向速度发送至B板 Can1Send0
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
void ChassisCan1Send(short *carSpeedx, short *carSpeedy, short *carSpeedw)
{
	CanTxMsg tx_message;
	tx_message.IDE = CAN_ID_STD;
	tx_message.RTR = CAN_RTR_DATA;
	tx_message.DLC = 0x08;
	tx_message.StdId = 0x101;
    
    F405.Yaw_100 = (short)(Theta_chassis/3.1415926f*180 * 100);

	memcpy(&tx_message.Data[0], carSpeedx, 2);
	memcpy(&tx_message.Data[2], carSpeedy, 2);
	memcpy(&tx_message.Data[4], carSpeedw, 2);
	memcpy(&tx_message.Data[6], &F405.Yaw_100, 2);
	CAN_Transmit(CAN1, &tx_message);
}

/**********************************************************************************************************
 *函 数 名: F405Can1Send
 *功能说明: 与B板通信	Can1Send1
 *形    参: 超级电容使用允许位  小陀螺模式标志位  大陀螺模式标志位  单挑模式标志位
 *返 回 值: 无
 **********************************************************************************************************/
//extern short armor_state;
extern char SelfProtect_Cross_Flag;
extern char k_slow;
extern char HighFreq_flag;
extern PCRecvData pc_recv_data;
extern char q_flag;


void F405Can1Send(F405_typedef *F405_Send)
{
	CanTxMsg tx_message;
	tx_message.IDE = CAN_ID_STD;
	tx_message.RTR = CAN_RTR_DATA;
	tx_message.DLC = 0x08;
	tx_message.StdId = 0x102;
//	if (magazineState == 0x01)
//		F405_Send->Mag_Flag = 1;
//	else
//		F405_Send->Mag_Flag = 0;
    if(Status.ShootMode == Shoot_Tx2_Mode)
        F405.AutoFire_Flag = 1;
    else
        F405.AutoFire_Flag = 0;
    
	F405_Send->Pitch_100 = (short)(Gimbal.Pitch.Gyro * 100);
	F405_Send->Send_Pack1 = ((F405_Send->AutoFire_Flag & 0x01) << 0) |
							((F405_Send->Laser_Flag & 0x01) << 1) |
							((F405_Send->Graphic_Init_Flag & 0x01) << 2) |
							((HighFreq_flag & 0x01) << 3) |
							((F405_Send->Fric_Flag & 0x01) << 4) |
							((pc_recv_data.enemy_id & 0x07) << 5);
	memcpy(&tx_message.Data[0], &F405_Send->SuperPowerLimit, 1);
	memcpy(&tx_message.Data[1], &F405_Send->Chassis_Flag, 1);
	memcpy(&tx_message.Data[2], &F405_Send->Pitch_100, 2);
	memcpy(&tx_message.Data[4], &F405_Send->Gimbal_Flag, 1);
	memcpy(&tx_message.Data[5], &F405_Send->Send_Pack1, 1);
	CAN_Transmit(CAN1, &tx_message);
}

/**********************************************************************************************************
 *函 数 名: YawCan1Send
 *功能说明: 发送yaw电流值
 *形    参: yaw电流值
 *返 回 值: 无
 **********************************************************************************************************/
extern RobotInit_Struct Infantry;
void YawCan1Send(short tempX)
{
	CanTxMsg tx_message;
	tx_message.IDE = CAN_ID_STD;
	tx_message.RTR = CAN_RTR_DATA;
	tx_message.DLC = 0x08;
	tx_message.StdId = 0x1FF;
	tempX = LIMIT_MAX_MIN(tempX, 30000, -30000);

	switch (Infantry.YawMotorID)
	{
	case 0x205:
		tx_message.Data[0] = (unsigned char)((tempX >> 8) & 0xff); // yaw
		tx_message.Data[1] = (unsigned char)(tempX & 0xff);
		break;
	case 0x206:
		tx_message.Data[2] = (unsigned char)((tempX >> 8) & 0xff); // yaw
		tx_message.Data[3] = (unsigned char)(tempX & 0xff);
		break;
	case 0x207:
		tx_message.Data[4] = (unsigned char)((tempX >> 8) & 0xff); // yaw
		tx_message.Data[5] = (unsigned char)(tempX & 0xff);
		break;
	case 0x208:
		tx_message.Data[6] = (unsigned char)((tempX>>8)&0xff);//yaw
		tx_message.Data[7] = (unsigned char)(tempX&0xff);
		break;
	default:
		break;
	}
	CAN_Transmit(CAN1, &tx_message);
}

/**********************************************************************************************************
 *函 数 名: PitchCan2Send
 *功能说明: 发送pitch电流值
 *形    参: pitch电流值
 *返 回 值: 无
 **********************************************************************************************************/
void PitchCan2Send(short tempX)
{
	CanTxMsg tx_message;
	tx_message.IDE = CAN_ID_STD;
	tx_message.RTR = CAN_RTR_DATA;
	tx_message.DLC = 0x08;
	tx_message.StdId = 0x1FF;
	tempX = LIMIT_MAX_MIN(tempX, 30000, -30000); // 30000

	switch (Infantry.PitchMotorID)
	{
	case 0x205:
		tx_message.Data[0] = (unsigned char)((tempX >> 8) & 0xff); // pitch
		tx_message.Data[1] = (unsigned char)(tempX & 0xff);
		break;
	case 0x206:
		tx_message.Data[2] = (unsigned char)((tempX >> 8) & 0xff); // pitch
		tx_message.Data[3] = (unsigned char)(tempX & 0xff);
		break;
	case 0x207:
		tx_message.Data[4] = (unsigned char)((tempX >> 8) & 0xff); // pitch
		tx_message.Data[5] = (unsigned char)(tempX & 0xff);
		break;
	case 0x208:
		tx_message.Data[6] = (unsigned char)((tempX >> 8) & 0xff); // pitch
		tx_message.Data[7] = (unsigned char)(tempX & 0xff);
		break;
	default:
		break;
	}
	CAN_Transmit(CAN2, &tx_message);
}

/**********************************************************************************************************
 *函 数 名: BodanCan1Send
 *功能说明: 发送拨弹电机电流值
 *形    参: 拨弹电机电流值
 *返 回 值: 无
 **********************************************************************************************************/
void BodanCan1Send(short tempX)
{
	CanTxMsg tx_message;
	tx_message.IDE = CAN_ID_STD;
	tx_message.RTR = CAN_RTR_DATA;
	tx_message.DLC = 0x08;
	tx_message.StdId = 0x200;
	tempX = LIMIT_MAX_MIN(tempX, 8000, -8000);

	switch (Infantry.BodanMotorID)
	{
	case 0x201:
		tx_message.Data[0] = (unsigned char)((tempX >> 8) & 0xff);
		tx_message.Data[1] = (unsigned char)(tempX & 0xff);
		break;
	case 0x202:
		tx_message.Data[2] = (unsigned char)((tempX >> 8) & 0xff);
		tx_message.Data[3] = (unsigned char)(tempX & 0xff);
		break;
	case 0x203:
		tx_message.Data[4] = (unsigned char)((tempX >> 8) & 0xff);
		tx_message.Data[5] = (unsigned char)(tempX & 0xff);
		break;
	case 0x204:
		tx_message.Data[6] = (unsigned char)((tempX >> 8) & 0xff);
		tx_message.Data[7] = (unsigned char)(tempX & 0xff);
		break;
	default:
		break;
	}
	CAN_Transmit(CAN1, &tx_message);
}

/**********************************************************************************************************
 *函 数 名: FrictionCan2Send
 *功能说明: 发送摩擦轮电机电流值
 *形    参: 摩擦轮电机电流值
 *返 回 值: 无
 **********************************************************************************************************/
void FrictionCan2Send(short tempX, short tempY)
{
	CanTxMsg tx_message;
	tx_message.IDE = CAN_ID_STD;
	tx_message.RTR = CAN_RTR_DATA;
	tx_message.DLC = 0x08;
	tx_message.StdId = 0x200;
	tempX = LIMIT_MAX_MIN(tempX, 9000, -9000);
	tempY = LIMIT_MAX_MIN(tempY, 9000, -9000);

	switch (Infantry.FricMotorID[0])
	{
	case 0x201:
		tx_message.Data[0] = (unsigned char)((tempX >> 8) & 0xff);
		tx_message.Data[1] = (unsigned char)(tempX & 0xff);
		break;
	case 0x202:
		tx_message.Data[2] = (unsigned char)((tempX >> 8) & 0xff);
		tx_message.Data[3] = (unsigned char)(tempX & 0xff);
		break;
	case 0x203:
		tx_message.Data[4] = (unsigned char)((tempX >> 8) & 0xff);
		tx_message.Data[5] = (unsigned char)(tempX & 0xff);
		break;
	case 0x204:
		tx_message.Data[6] = (unsigned char)((tempX >> 8) & 0xff);
		tx_message.Data[7] = (unsigned char)(tempX & 0xff);
		break;
	default:
		break;
	}
	switch (Infantry.FricMotorID[1])
	{
	case 0x201:
		tx_message.Data[0] = (unsigned char)((tempY >> 8) & 0xff);
		tx_message.Data[1] = (unsigned char)(tempY & 0xff);
		break;
	case 0x202:
		tx_message.Data[2] = (unsigned char)((tempY >> 8) & 0xff);
		tx_message.Data[3] = (unsigned char)(tempY & 0xff);
		break;
	case 0x203:
		tx_message.Data[4] = (unsigned char)((tempY >> 8) & 0xff);
		tx_message.Data[5] = (unsigned char)(tempY & 0xff);
		break;
	case 0x204:
		tx_message.Data[6] = (unsigned char)((tempY >> 8) & 0xff);
		tx_message.Data[7] = (unsigned char)(tempY & 0xff);
		break;
	default:
		break;
	}
	CAN_Transmit(CAN2, &tx_message);
}

/**********************************************************************************************************
 *函 数 名: Tx2_Off_CheckAndSet
 *功能说明: 发送指令给tx2使其关机
 *形    参: 指向发送数据的指针
 *返 回 值: 无
 **********************************************************************************************************/
void Tx2_Off_CheckAndSet(u8 *Buff)
{
	int i;
	if (PC_Sendflag == Tx2_Off)
		for (i = 0; i < 9; i++)
		{
			*Buff = '!';
			Buff++;
		}
}
/**********************************************************************************************************
 *函 数 名: TX2_task
 *功能说明: 通信任务
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
uint32_t TX2_high_water;
TickType_t LastWakeTime_PC;
void TX2_task(void *pvParameters)
{
	while (1)
	{
		LastWakeTime_PC = xTaskGetTickCount();
		SendtoPC();
		IWDG_Feed();
		vTaskDelayUntil(&LastWakeTime_PC, 1);

#if INCLUDE_uxTaskGetStackHighWaterMark
		TX2_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
	}
}
