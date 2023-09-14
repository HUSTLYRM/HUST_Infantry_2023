/**********************************************************************************************************
 * @�ļ�     i2c.c
 * @˵��     ģ��i2cʵ��
 * @�汾  	 V1.0
 * @����     ����
 * @����     2019.01
 
  * @�ļ�     i2c.c
 * @˵��     ģ��i2cʵ��
 * @�汾  	 V2.0
 * @����     ����
 * @����     2022.01
**********************************************************************************************************/
#include "main.h"
void IIC_SCL(u8 flag)
{
	if(flag > 0)
		HAL_GPIO_WritePin(GPIO_GROUP_SCL, GPIO_PIN_SCL, GPIO_PIN_SET);
	else 
		HAL_GPIO_WritePin(GPIO_GROUP_SCL, GPIO_PIN_SCL, GPIO_PIN_RESET);
}

void IIC_SDA(u8 flag)
{
	if(flag > 0)
		HAL_GPIO_WritePin(GPIO_GROUP_SDA, GPIO_PIN_SDA, GPIO_PIN_SET);
	else 
		HAL_GPIO_WritePin(GPIO_GROUP_SDA, GPIO_PIN_SDA, GPIO_PIN_RESET);
}

void SDA_IN(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	/*Configure GPIO pins : PA1 */
	GPIO_InitStruct.Pin = GPIO_PIN_SDA;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIO_GROUP_SDA, &GPIO_InitStruct);
}

void SDA_OUT(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	/*Configure GPIO pins : PA1 */
	GPIO_InitStruct.Pin = GPIO_PIN_SDA;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIO_GROUP_SDA, &GPIO_InitStruct);
}

//����IIC��ʼ�ź�
void IIC_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SDA(1);	  	  
	IIC_SCL(1);
	delay_us(4);
 	IIC_SDA(0);//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL(0);//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL(0);
	IIC_SDA(0);//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL(1); 
	delay_us(4);			
	IIC_SDA(1);//����I2C���߽����ź�				   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
	IIC_SDA(1);delay_us(1);	   
	IIC_SCL(1);delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL(0);//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
void IIC_Ack(void)
{
	IIC_SCL(0);
	SDA_OUT();
	IIC_SDA(0);
	delay_us(2);
	IIC_SCL(1);
	delay_us(2);
	IIC_SCL(0);
}
//������ACKӦ��		    
void IIC_NAck(void)
{
	IIC_SCL(0);
	SDA_OUT();
	IIC_SDA(1);
	delay_us(2);
	IIC_SCL(1);
	delay_us(2);
	IIC_SCL(0);
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
		SDA_OUT(); 	    
    IIC_SCL(0);//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        IIC_SDA((txd&0x80)>>7);
        txd<<=1; 	  
		delay_us(2);   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL(1);     //SCLΪ�ߵ�ƽʱ��SDA��������Ч
		delay_us(2); 
		IIC_SCL(0);	
		delay_us(2);
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        IIC_SCL(0); 
        delay_us(2);
		IIC_SCL(1);
        receive<<=1;
        if(READ_SDA)receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}


