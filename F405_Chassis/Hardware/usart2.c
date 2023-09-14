/**********************************************************************************************************
 * @文件     usart4.c
 * @说明     uart初始化,VOFA
 * @版本  	 V1.0
**********************************************************************************************************/
#include "main.h"

//unsigned char DataScopeSend_Buf[DataScopeSend_BufSize];
#define	VOFA_MAX_CHANNEL 10
#ifndef SD_READ_MODE
#define VOFA_SEND_SIZE (VOFA_MAX_CHANNEL*4+4)
#else
#define VOFA_SEND_SIZE 100
#endif
float 	VOFA_justfloat[VOFA_MAX_CHANNEL];
uint8_t VOFA_send_Data[VOFA_SEND_SIZE];
DMA_InitTypeDef   dma;
extern float INA_Power;
/**********************************************************************************************************
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void VOFA_USART_Configuration(void)
{
	USART_InitTypeDef usart;
	GPIO_InitTypeDef  gpio;
	NVIC_InitTypeDef  nvic;
	
			//写入结尾数据
	VOFA_send_Data[VOFA_MAX_CHANNEL*4] = 0x00;                    
	VOFA_send_Data[VOFA_MAX_CHANNEL*4+1] = 0x00;
	VOFA_send_Data[VOFA_MAX_CHANNEL*4+2] = 0x80;
	VOFA_send_Data[VOFA_MAX_CHANNEL*4+3] = 0x7f;
	
	nvic.NVIC_IRQChannel = VOFA_DMA_TX_IRQ;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	
		{
			VOFA_DMA_TX_AHBxClock_FUN(VOFA_DMA_TX_CLK,ENABLE);
			DMA_DeInit(VOFA_DMA_TX_STREAM);
			dma.DMA_Channel= VOFA_DMA_TX_CHANNEL;
			dma.DMA_PeripheralBaseAddr = (uint32_t)&(VOFA_USARTx->DR);
			dma.DMA_Memory0BaseAddr = (uint32_t)VOFA_send_Data;
			dma.DMA_DIR = DMA_DIR_MemoryToPeripheral;
			dma.DMA_BufferSize = (VOFA_SEND_SIZE);
			dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
			dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
			dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
			dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
			dma.DMA_Mode = DMA_Mode_Normal;
			dma.DMA_Priority = DMA_Priority_VeryHigh;
			dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
			dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
			dma.DMA_MemoryBurst = DMA_Mode_Normal;
			dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		
			DMA_Init(VOFA_DMA_TX_STREAM,&dma);
		  DMA_ClearFlag(VOFA_DMA_TX_STREAM, VOFA_DMA_FLAG_TCIF);  // clear all DMA flags
			DMA_ITConfig(VOFA_DMA_TX_STREAM,DMA_IT_TC,ENABLE);  //open DMA send inttrupt
			DMA_Cmd(VOFA_DMA_TX_STREAM,DISABLE);
		}	
	
	VOFA_USART_GPIO_APBxClock_FUN(VOFA_USART_GPIO_CLK,ENABLE);
	VOFA_USART_APBxClock_FUN(VOFA_USART_CLK,ENABLE);

	GPIO_PinAFConfig(VOFA_USART_TX_PORT,VOFA_USART_TX_SOURCE,VOFA_USART_TX_AF);
	GPIO_PinAFConfig(VOFA_USART_RX_PORT,VOFA_USART_RX_SOURCE,VOFA_USART_RX_AF); 

	gpio.GPIO_Pin = VOFA_USART_TX_PIN;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(VOFA_USART_TX_PORT,&gpio);
	
	gpio.GPIO_Pin = VOFA_USART_RX_PIN;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(VOFA_USART_RX_PORT,&gpio);

	usart.USART_BaudRate = VOFA_USART_BAUD_RATE;
	usart.USART_WordLength = USART_WordLength_8b;
	usart.USART_StopBits = USART_StopBits_1;
	usart.USART_Parity = USART_Parity_No;
	usart.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(VOFA_USARTx,&usart);
	
	USART_Cmd(VOFA_USARTx,ENABLE);
	
	USART_DMACmd(VOFA_USARTx,USART_DMAReq_Tx,ENABLE);	

	DMA_Cmd(VOFA_DMA_TX_STREAM,ENABLE);
}
void VOFA_DMA_TX_INT_FUN(void)
{
	if(DMA_GetITStatus(VOFA_DMA_TX_STREAM,VOFA_DMA_IT_STATUS  ))
	{
		DMA_ClearITPendingBit(VOFA_DMA_TX_STREAM,VOFA_DMA_IT_STATUS);
		DMA_Cmd(VOFA_DMA_TX_STREAM, DISABLE); 
  }
}

//将要输出的变量声明extern
extern JudgeReceive_t JudgeReceive;
extern float K_Power;
extern Power_Limit_type powerlimit;
extern short WheelCurrentSend[4];
extern int testspeed;
extern SuperPower superpower;
extern chassis_type chassis;//底盘控制结构体
extern INA260 INA260_1;
extern roboDisconnect Robot_Disconnect;
extern F405_typedef F405;
void VOFA_Send(void)
{
#ifdef  SD_READ_MODE //SD卡输出
	extern char ReadBuffer[];
	
	memcpy(VOFA_send_Data,ReadBuffer,VOFA_SEND_SIZE);
	
#else
	
	//需转为float型数据存储
	VOFA_justfloat[0]=(float)(powerlimit.set_power);
	VOFA_justfloat[1]=(float)(chassis.ChassisMotorCanReceive[0].RealSpeed);
	VOFA_justfloat[2]=(float)(superpower.PowerState);
	VOFA_justfloat[3]=(float)(superpower.actual_vol);
//	VOFA_justfloat[4]=(float)(powerlimit.remainEnergy);
//	VOFA_justfloat[5]=(float)(powerlimit.predict_power);
//	VOFA_justfloat[6]=(float)(powerlimit.actual_referee_power);
	VOFA_justfloat[4]=(float)(powerlimit.predict_p0 - 14);
	VOFA_justfloat[5]=(float)(powerlimit.predict_power - powerlimit.predict_p0 - 14);
//	
    //动态K采样
//    VOFA_justfloat[0]=(float)(powerlimit.actual_i_2);
//	VOFA_justfloat[1]=(float)(powerlimit.actual_w_i);
//	VOFA_justfloat[2]=(float)(powerlimit.actual_ina260_power);

    
	//拟合
	VOFA_justfloat[6]=(float)(chassis.ChassisMotorCanReceive[0].Current);
	VOFA_justfloat[7]=(float)(chassis.ChassisMotorCanReceive[1].Current);
	VOFA_justfloat[8]=(float)(chassis.ChassisMotorCanReceive[2].Current);
	VOFA_justfloat[9]=(float)(chassis.ChassisMotorCanReceive[3].Current);
//	VOFA_justfloat[4]=(float)(chassis.SteerMotorsCanReceive[0].Current);
//	VOFA_justfloat[5]=(float)(chassis.SteerMotorsCanReceive[1].Current);
//	VOFA_justfloat[6]=(float)(chassis.SteerMotorsCanReceive[2].Current);
//	VOFA_justfloat[7]=(float)(chassis.SteerMotorsCanReceive[3].Current);
	
	
	//对比
	
//	VOFA_justfloat[6]=(float)(Robot_Disconnect.motor_rec_timer[0]);
//	VOFA_justfloat[7]=(float)(Robot_Disconnect.motor_rec_timer[1]);
//	VOFA_justfloat[8]=(float)(Robot_Disconnect.motor_rec_timer[2]);
//	VOFA_justfloat[9]=(float)(Robot_Disconnect.motor_rec_timer[3]);
//	VOFA_justfloat[4]=(float)(Robot_Disconnect.motor_rec_timer[4]);
//	VOFA_justfloat[5]=(float)(Robot_Disconnect.motor_rec_timer[5]);
//	VOFA_justfloat[6]=(float)(Robot_Disconnect.motor_rec_timer[6]);
//	VOFA_justfloat[7]=(float)(Robot_Disconnect.motor_rec_timer[7]);
//	VOFA_justfloat[8]=(float)(powerlimit.remainEnergy);
//	for(int i=0;i<8;i++){
//		VOFA_justfloat[i] = LIMIT_MAX_MIN(VOFA_justfloat[i],2.0,0.0);
//	}
	
	
	//拷贝到传输变量
	memcpy(VOFA_send_Data, (uint8_t *)VOFA_justfloat, sizeof(VOFA_justfloat));
#endif	
	VOFA_DMA_TX_STREAM->NDTR = VOFA_SEND_SIZE; 
	DMA_Cmd(VOFA_DMA_TX_STREAM, ENABLE);   

}



