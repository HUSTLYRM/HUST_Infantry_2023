/**********************************************************************************************************
 * @文件     DataReceive.c
 * @说明     接收函数
 * @版本  	 V1.0
 * @作者     黄志雄
 * @日期     2020.1
 **********************************************************************************************************/
/**********************************************************************************************************
 * @文件     DataReceive.c
 * @说明     接收函数
 * @版本  	 V2.0
 * @作者     戴军
 * @日期     2022.3
 **********************************************************************************************************/
#include "main.h"

RC_Ctl_t RC_Ctl;		  //遥控器数据
F105_Typedef F105;
PC_Receive_t PC_Receive;
BodanMotorReceive_Typedef BodanReceive;
char PitchMotor_ReceiveFlag;
short FrictionReceive[2];
float PitchMotorReceive, YawMotorReceive; // Pitch,Yaw电机角度
float yaw_gyro_temp, pitch_gyro_temp;

Disconnect Robot_Disconnect;

extern ZeroCheck_Typedef ZeroCheck_Pitch;
extern Gimbal_Typedef Gimbal;
extern volatile long run_time_check;
extern Status_t Status;
extern short KalMan_doneflag;
extern char Judge_Lost;
char Chassis_ID;

extern PCSendData pc_send_data;
extern uint8_t Remote_Receive_Flag;
extern uint8_t PC_ReceiveFlag;
extern volatile unsigned char sbus_rx_buffer[18];
extern unsigned char PCRecbuffer[PC_RECVBUF_SIZE];

unsigned char tempPC[PC_RECVBUF_SIZE]; //这里要改为全局变量，不然crc不通过
short crcNopass;
short buffindex;
uint8_t CoolBuffState;
/**********************************************************************************************************
 *函 数 名: Can1Receive0
 *功能说明: 功率板和拨弹电机通信
 *形    参: rx_message1
 *返 回 值: 无
 **********************************************************************************************************/
int error_cnt = 0;
extern RobotInit_Struct Infantry;
short YawMotorSpeed;
extern ShootTask_typedef Shoot;
void Can1Receive0(CanRxMsg rx_message1)
{
	switch (rx_message1.StdId)
	{
	case 0x094://裁判系统信息
		memcpy(&F105.JudgeReceive_info,rx_message1.Data,sizeof(JudgeReceive_Info_Typedef));
		Shoot.HeatControl.HeatUpdateFlag = 1;
        Robot_Disconnect.F105_DisConect=0;
		break;
	case 0x096://大符信息
		memcpy(&CoolBuffState, &rx_message1.Data[0], 1);
        Robot_Disconnect.F105_DisConect=0;
		break;
	case 0x100://底盘控制反馈
		memcpy(&F105.ChassisSpeedw, &rx_message1.Data[0], 2);
        Robot_Disconnect.F105_DisConect=0;
		break;

		//		case 0x209:
		//				 YawMotorReceive=rx_message1.Data[0]<<8 | rx_message1.Data[1];
		//		     YawMotorSpeed  =rx_message1.Data[2]<<8 | rx_message1.Data[3];
		//				 Robot_Disconnect.YawMotor_DisConnect=0;
		//		break;
		//		case 0x204:
		//			   BodanReceive.Angle=rx_message1.Data[0]<<8 | rx_message1.Data[1];
		//				 BodanReceive.RealSpeed=rx_message1.Data[2]<<8 | rx_message1.Data[3];
		//				 Robot_Disconnect.Pluck_DisConnect=0;
		//		break;
	}
}

/**********************************************************************************************************
 *函 数 名: Can1Receive1
 *功能说明: 功率板和拨弹电机通信
 *形    参: rx_message1
 *返 回 值: 无
 **********************************************************************************************************/
extern uint32_t cnt_last;
extern int diudiao;
extern double yuzhi;
void Can1Receive1(CanRxMsg rx_message1)
{
	if (rx_message1.StdId == Infantry.BodanMotorID)
	{
		BodanReceive.Angle = rx_message1.Data[0] << 8 | rx_message1.Data[1];
		BodanReceive.RealSpeed = rx_message1.Data[2] << 8 | rx_message1.Data[3];
		Robot_Disconnect.Pluck_DisConnect = 0;
	}else if(rx_message1.StdId == Infantry.YawMotorID){
		YawMotorReceive = rx_message1.Data[0] << 8 | rx_message1.Data[1];
		YawMotorSpeed = rx_message1.Data[2] << 8 | rx_message1.Data[3];
		Robot_Disconnect.YawMotor_DisConnect = 0;
		Robot_Disconnect.Pluck_DisConnect = 0;
		if (GetDeltaT(&cnt_last) > yuzhi)
		{
			diudiao++;
		}
		else
		{
			diudiao = 0;
		}
	}
}

/**********************************************************************************************************
 *函 数 名: Can1Receive1
 *功能说明: Yaw电机、拨弹电机角度接收
 *形    参: rx_message0
 *返 回 值: 无
 **********************************************************************************************************/
// extern RobotInit_Struct Infantry;
// short YawMotorSpeed;
// void Can1Receive1(CanRxMsg rx_message0)
//{
//	if(rx_message0.StdId == Infantry.YawMotorID)
//	{
//				 YawMotorReceive=rx_message0.Data[0]<<8 | rx_message0.Data[1];
//		     YawMotorSpeed  =rx_message0.Data[2]<<8 | rx_message0.Data[3];
//				 Robot_Disconnect.YawMotor_DisConnect=0;
//
//	}
//	else if(rx_message0.StdId == Infantry.BodanMotorID)
//	{
//          BodanReceive.Angle=rx_message0.Data[0]<<8 | rx_message0.Data[1];
//				 BodanReceive.RealSpeed=rx_message0.Data[2]<<8 | rx_message0.Data[3];
//				 Robot_Disconnect.Pluck_DisConnect=0;
//	}
// }

/**********************************************************************************************************
 *函 数 名: Can2Receive0
 *功能说明: Pitch电机、摩擦轮电机角度接收
 *形    参: rx_message0
 *返 回 值: 无
 **********************************************************************************************************/
short PitchMotorSpeed;
void Can2Receive0(CanRxMsg rx_message0)
{
	if (rx_message0.StdId == Infantry.PitchMotorID)
	{
		PitchMotorReceive = rx_message0.Data[0] << 8 | rx_message0.Data[1];
        PitchMotorSpeed = rx_message0.Data[2] << 8 | rx_message0.Data[3];
		Robot_Disconnect.PitchMotor_DisConnect = 0;
		if (!PitchMotor_ReceiveFlag)
		{
			ZeroCheck_Pitch.LastValue = PitchMotorReceive;
		}
		PitchMotor_ReceiveFlag = 1;
	}
	else if (rx_message0.StdId == Infantry.FricMotorID[0])
	{
		FrictionReceive[0] = rx_message0.Data[2] << 8 | rx_message0.Data[3];
		Robot_Disconnect.Friction_DisConnect[0] = 0;
	}
	else if (rx_message0.StdId == Infantry.FricMotorID[1])
	{
		FrictionReceive[1] = rx_message0.Data[2] << 8 | rx_message0.Data[3];
		Robot_Disconnect.Friction_DisConnect[1] = 0;
	}
}

/**********************************************************************************************************
 *函 数 名: Can2Receive1
 *功能说明: 陀螺仪数据接收
 *形    参: rx_message1
 *返 回 值: 无
 **********************************************************************************************************/
void Can2Receive1(CanRxMsg *rx_message1)
{
    	switch (rx_message1->StdId)
	{
	case 0x101:
	{
    memcpy(&IMUReceive.Acc[0], rx_message1->Data, 2);
    memcpy(&IMUReceive.Acc[1], &rx_message1->Data[2], 2);   
    memcpy(&IMUReceive.Acc[2], &rx_message1->Data[4], 2);
    global_debugger.imu_debugger[1].recv_msgs_num++;
    Robot_Disconnect.Gyro_DisConnect = 0;
    break;
    }
  
    case 0x100:
    {
    memcpy(&IMUReceive.Gyro[0], rx_message1->Data, 2);
    memcpy(&IMUReceive.Gyro[1], &rx_message1->Data[2], 2);
    memcpy(&IMUReceive.Gyro[2], &rx_message1->Data[4], 2);
    global_debugger.imu_debugger[0].recv_msgs_num++;
    Robot_Disconnect.Gyro_DisConnect = 0;
    break;
    }
  
    default:
    break;
    }
}

/**********************************************************************************************************
 *函 数 名: RemoteReceive
 *功能说明: 遥控器数据接收
 *形    参: rx_buffer[]
 *返 回 值: 无
 **********************************************************************************************************/
char temp1, temp2, temp3;
void RemoteReceive(volatile unsigned char rx_buffer[])
{
	RC_Ctl.rc.ch0 = (rx_buffer[0] | (rx_buffer[1] << 8)) & 0x07ff;								 //!< Channel 0
	RC_Ctl.rc.ch1 = ((rx_buffer[1] >> 3) | (rx_buffer[2] << 5)) & 0x07ff;						 //!< Channel 1
	RC_Ctl.rc.ch2 = ((rx_buffer[2] >> 6) | (rx_buffer[3] << 2) | (rx_buffer[4] << 10)) & 0x07ff; //!< Channel 2
	RC_Ctl.rc.ch3 = ((rx_buffer[4] >> 1) | (rx_buffer[5] << 7)) & 0x07ff;						 //!< Channel 3
	RC_Ctl.rc.s1 = ((rx_buffer[5] >> 4) & 0x0003);												 //!< Switch left
	RC_Ctl.rc.s2 = ((rx_buffer[5] >> 6) & 0x0003);
	RC_Ctl.mouse.x = rx_buffer[6] | (rx_buffer[7] << 8);   //!< Mouse X axis
	RC_Ctl.mouse.y = rx_buffer[8] | (rx_buffer[9] << 8);   //!< Mouse Y axis
	RC_Ctl.mouse.z = rx_buffer[10] | (rx_buffer[11] << 8); //!< Mouse Z axis
	RC_Ctl.mouse.press_l = rx_buffer[12];				   //!< Mouse Left Is Press ?
	RC_Ctl.mouse.press_r = rx_buffer[13];				   //!< Mouse Right Is Press ?
	RC_Ctl.key.w = rx_buffer[14] & 0x01;				   // KeyBoard value
	RC_Ctl.key.s = (rx_buffer[14] >> 1) & 0x01;
	RC_Ctl.key.a = (rx_buffer[14] >> 2) & 0x01;
	RC_Ctl.key.d = (rx_buffer[14] >> 3) & 0x01;
	RC_Ctl.key.shift = (rx_buffer[14] >> 4) & 0x01;
	RC_Ctl.key.ctrl = (rx_buffer[14] >> 5) & 0x01;
	RC_Ctl.key.q = (rx_buffer[14] >> 6) & 0x01;
	RC_Ctl.key.e = (rx_buffer[14] >> 7) & 0x01;
	RC_Ctl.key.r = (rx_buffer[15]) & 0x01;
	RC_Ctl.key.f = (rx_buffer[15] >> 1) & 0x01;
	RC_Ctl.key.g = (rx_buffer[15] >> 2) & 0x01;
	RC_Ctl.key.z = (rx_buffer[15] >> 3) & 0x01;
	RC_Ctl.key.x = (rx_buffer[15] >> 4) & 0x01;
	RC_Ctl.key.c = (rx_buffer[15] >> 5) & 0x01;
	RC_Ctl.key.v = (rx_buffer[15] >> 6) & 0x01;
	RC_Ctl.key.b = (rx_buffer[15] >> 7) & 0x01;
	temp1 = rx_buffer[16];
	temp2 = rx_buffer[17];
	if ((RC_Ctl.rc.ch0 - 1024 < 20) && (RC_Ctl.rc.ch0 - 1024 > -20))
		RC_Ctl.rc.ch0 = 1024;
	if ((RC_Ctl.rc.ch1 - 1024 < 20) && (RC_Ctl.rc.ch1 - 1024 > -20))
		RC_Ctl.rc.ch1 = 1024;
	if ((RC_Ctl.rc.ch2 - 1024 < 20) && (RC_Ctl.rc.ch2 - 1024 > -20))
		RC_Ctl.rc.ch2 = 1024;
	if ((RC_Ctl.rc.ch3 - 1024 < 20) && (RC_Ctl.rc.ch3 - 1024 > -20))
		RC_Ctl.rc.ch3 = 1024;

	Robot_Disconnect.RC_DisConnect = 0;
}

/**********************************************************************************************************
 *函 数 名: RC_Rst
 *功能说明: 遥控器数据复位
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
void RC_Rst(void)
{
	RC_Ctl.rc.ch0 = 1024;
	RC_Ctl.rc.ch1 = 1024;
	RC_Ctl.rc.ch2 = 1024;
	RC_Ctl.rc.ch3 = 1024;
	RC_Ctl.mouse.x = 0;
	RC_Ctl.mouse.y = 0;
	RC_Ctl.mouse.z = 0;
	RC_Ctl.mouse.press_l = 0;
	RC_Ctl.mouse.press_r = 0;

	RC_Ctl.key.w = 0;
	RC_Ctl.key.s = 0;
	RC_Ctl.key.a = 0;
	RC_Ctl.key.d = 0;
	RC_Ctl.key.q = 0;
	RC_Ctl.key.e = 0;
	RC_Ctl.key.r = 0;
	RC_Ctl.key.f = 0;
	RC_Ctl.key.shift = 0;
	RC_Ctl.key.ctrl = 0;

	RC_Ctl.rc.s1 = 2;
	RC_Ctl.rc.s2 = 2;
}

/**********************************************************************************************************
 *函 数 名: F105_Rst
 *功能说明: 功率板掉线复位
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
unsigned int Heat_LimitTick;
void F105_Rst()
{
	Heat_LimitTick++;
}

/**********************************************************************************************************
 *函 数 名: RCReceive_task
 *功能说明: 遥控器数据处理任务
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/

extern TaskHandle_t RCReceiveTask_Handler; //任务句柄
extern TaskHandle_t PCReceiveTask_Handler; //任务句柄
uint32_t RC_high_water;



void RCReceive_task()
{
	//	static BaseType_t DataReceive_Exit = pdFALSE ;  //使用通知实现计数信号量，便于不同中断信息处理

	while (1)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY); //若无通知更新，则不唤醒

		/********************************* 遥控器数据处理 *****************************************************/
		Remote_Receive_Flag = 0;
		RemoteReceive(sbus_rx_buffer); //解码函数
        #if INCLUDE_uxTaskGetStackHighWaterMark
		RC_high_water = uxTaskGetStackHighWaterMark(NULL);
        #endif
	}
}

///**********************************************************************************************************
// *函 数 名: PCReceive_task
// *功能说明: PC数据处理任务
// *形    参: 无
// *返 回 值: 无
// **********************************************************************************************************/
//uint32_t PC_high_water;
//void PCReceive_task()
//{
//	//	static BaseType_t DataReceive_Exit = pdFALSE ;  //使用通知实现计数信号量，便于不同中断信息处理

//	static unsigned char temptemp[2 * PC_RECVBUF_SIZE]; //用于PC数据处理的局部变量
//	short PackPoint, n;

//	while (1)
//	{
//		ulTaskNotifyTake(pdTRUE, portMAX_DELAY); //若无通知更新，则不唤醒,通知实现二值信号量

//		/********************************* PC数据处理 *******************************************************/

//		PC_ReceiveFlag = 0;

//		memcpy(temptemp + PC_RECVBUF_SIZE, PCRecbuffer, PC_RECVBUF_SIZE);
//		for (PackPoint = 0; PackPoint < PC_RECVBUF_SIZE; PackPoint++) //防止错位，不一定数组元素的第一个就为
//		{
//			if (temptemp[PackPoint] == '!')
//			{
//				for (n = 0; n < PC_RECVBUF_SIZE; n++)
//				{
//					tempPC[n] = temptemp[(n + PackPoint)];
//				}
//				crcNopass++;
//				if (Verify_CRC8_Check_Sum(tempPC, PC_RECVBUF_SIZE))
//				{
//					PCReceive(tempPC);
//				}
//				else
//				{
//					buffindex++;
//					buffindex = buffindex % 4;
//				}
//				break;
//			}
//		}
//		memcpy(temptemp, temptemp + PC_RECVBUF_SIZE, PC_RECVBUF_SIZE);

//		/********************************* 其他数据处理 *******************************************************/
//        #if INCLUDE_uxTaskGetStackHighWaterMark
//		PC_high_water = uxTaskGetStackHighWaterMark(NULL);
//        #endif
//	}
//}