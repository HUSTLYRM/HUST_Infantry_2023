#ifndef _IIC_H_
#define _IIC_H_

#include "main.h"
//F042F6: IIC初始化  PA1-SDA PA4-SCL
#define GPIO_PIN_SDA GPIO_PIN_1
#define GPIO_GROUP_SDA GPIOA

#define GPIO_PIN_SCL GPIO_PIN_4
#define GPIO_GROUP_SCL GPIOA


/*******************??????*************************/
typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;
/*******************?????**************************/

#define READ_SDA HAL_GPIO_ReadPin(GPIO_GROUP_SDA, GPIO_PIN_SDA)

void IIC_SCL(u8 flag);
void SDA_IN(void);
void SDA_OUT(void);
//IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口				 
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void IIC_Send_Byte(u8 txd);			//IIC发送一个字节
u8 IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
u8 IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	 

#endif