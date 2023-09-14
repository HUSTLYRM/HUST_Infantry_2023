/**********************************************************************************************************
 * @文件     usart4.c
 * @说明     uart初始化,陀螺仪
 * @版本  	 V1.0
**********************************************************************************************************/
#include "main.h"

#define H101_RECNUM 35
unsigned char F103REC_Buffer[H101_RECNUM];

/**********************************************************************************************************
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void UART4_Configuration(void)
{
{
    USART_InitTypeDef usart4;
		GPIO_InitTypeDef  gpio;
		NVIC_InitTypeDef  nvic;
	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);
	
		GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4);
		GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4); 
	
    gpio.GPIO_Pin = GPIO_Pin_11;
		gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOC,&gpio);

	  gpio.GPIO_Pin = GPIO_Pin_10;
		gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOC,&gpio);
		
		usart4.USART_BaudRate = 9600;
		usart4.USART_WordLength = USART_WordLength_8b;
		usart4.USART_StopBits = USART_StopBits_1;
		usart4.USART_Parity = USART_Parity_No;
		usart4.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
    usart4.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(UART4,&usart4);
		
		//USART_ITConfig(UART5, USART_IT_IDLE, ENABLE);  
		USART_Cmd(UART4,ENABLE);
		USART_DMACmd(UART4,USART_DMAReq_Rx,ENABLE);	
		
//		nvic.NVIC_IRQChannel = UART5_IRQn;
//    nvic.NVIC_IRQChannelPreemptionPriority = 1;
//    nvic.NVIC_IRQChannelSubPriority = 1;
//    nvic.NVIC_IRQChannelCmd = ENABLE;



		nvic.NVIC_IRQChannel = DMA1_Stream2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 2;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
		
		{
			DMA_InitTypeDef  dma;
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
			DMA_DeInit(DMA1_Stream2);
			dma.DMA_Channel= DMA_Channel_4;
			dma.DMA_PeripheralBaseAddr = (uint32_t)&(UART4->DR);
			dma.DMA_Memory0BaseAddr = (uint32_t)F103REC_Buffer;
			dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
			dma.DMA_BufferSize = H101_RECNUM;
			dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
			dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
			dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
			dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
			dma.DMA_Mode = DMA_Mode_Circular;
			dma.DMA_Priority = DMA_Priority_High;
			dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
			dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
			dma.DMA_MemoryBurst = DMA_Mode_Normal;
			dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
			DMA_Init(DMA1_Stream2,&dma);
			DMA_ITConfig(DMA1_Stream2,DMA_IT_TC,ENABLE);
			DMA_Cmd(DMA1_Stream2,ENABLE);
		}	
}
}


/**********************************************************************************************************
*函 数 名: DMA接收
*功能说明: UART4与F4
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/

void DMA1_Stream2_IRQHandler(void)
{	
	if(DMA_GetITStatus(DMA1_Stream2, DMA_IT_TCIF2))
	{
		//F103Receive(F103REC_Buffer);
		int n=0;
		for(n=0;n<H101_RECNUM-15;n++)
		{
			if(F103REC_Buffer[n] == 0x55)
			{
					decode_gyro(&F103REC_Buffer[n]);
			}
		} 
		DMA_ClearFlag(DMA1_Stream2, DMA_IT_TCIF2);
		DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);
	}
}
