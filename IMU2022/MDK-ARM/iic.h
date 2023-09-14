#ifndef _IIC_H_
#define _IIC_H_

#include "main.h"
//F042F6: IIC��ʼ��  PA1-SDA PA4-SCL
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
//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	 

#endif