/**********************************************************************************************************
 * @�ļ�     qmc5883L.c
 * @˵��     ģ��i2c��ȡ����������
 * @�汾  	 V1.0
 * @����     ����/��һ��
 * @����     2019.01
 
 * @�汾   	 V2.0 
 * @������־	 ��������оƬ��ist8310��Ϊqmc5883L,���޸ļĴ�����
 * @���� 		 ��־��
 * @����		 2021.3
**********************************************************************************************************/
#include "main.h"

float mag_data[3];//����������
float mag_x_bias = 0;//����������У׼���ƫ��
float mag_y_bias = 0;
float mag_z_bias = 0;

float mag_x_scale = 1;//����������У׼�����������
float mag_y_scale = 1;
float mag_z_scale = 1;

extern short sys_flag;
u8 QMC5883L_reg1 = 0;//should be 0x1D
u8 QMC5883L_reg2 = 0;//should be 0x01

/**********************************************************************************************************
*�� �� ��: Init_QMC5883
*����˵��: QMC5883��ʼ��������ȡ�Ĵ���1��2�����ж��Ƿ��ȡ�ɹ���ʼ��
*��    ��: ��
*�� �� ֵ: 
			1----reg1����
			2----reg2����
			3----name����
**********************************************************************************************************/
u8 Init_QMC5883(void)
{
  unsigned char ID;
	//���оƬ�Ƿ����
	ID=QMC5883L_ReadOneByte(QMC5883L_ADDRESS,QMC5883L_ID);
	if(ID!=0XFF) 
	return 1;
	                                                 // ����2G  0x0D   8G 0X1D  
	QMC5883L_WriteOneByte(QMC5883L_ADDRESS,QMC5883L_CTR1,0x1D);
	QMC5883L_reg1 = QMC5883L_ReadOneByte(QMC5883L_ADDRESS, QMC5883L_CTR1);
	if(QMC5883L_reg1 != 0x1D)
		return 1;
	HAL_Delay(10);

	QMC5883L_WriteOneByte(QMC5883L_ADDRESS,QMC5883L_PFBR,0x01);
	QMC5883L_reg2 = QMC5883L_ReadOneByte(QMC5883L_ADDRESS, QMC5883L_PFBR);
	if(QMC5883L_reg2 != 0x01)
		return 2;
	
	return 0;
}
/**********************************************************************************************************
*�� �� ��: QMC5883L_ReadOneByte
*����˵��: i2c��ȡһλ�Ĵ�������
*��    ��: 
           address-������ַ
           reg-----�Ĵ�����ַ
*�� �� ֵ: �Ĵ�����ֵ
**********************************************************************************************************/
u8 QMC5883L_ReadOneByte(u8 address, u8 reg)
{ 
  u8 temp = 0;
  IIC_Start();
  IIC_Send_Byte(address); 
  IIC_Wait_Ack();
  IIC_Send_Byte(reg);     
  IIC_Wait_Ack();
  IIC_Start();
  IIC_Send_Byte(address + 1); //�������ģʽ 
  IIC_Wait_Ack();
  temp = IIC_Read_Byte(0);  //���Ĵ��� 3
  IIC_Stop();//����һ��ֹͣ���� 
  return temp;
}
/**********************************************************************************************************
*�� �� ��: QMC5883L_ReadBuffer
*����˵��: i2c��ȡ�Ĵ�������
*��    ��: 
           address-������ַ
           reg-----�Ĵ�����ַ
           buffer--����ָ��
           len-----����
*�� �� ֵ: ��
**********************************************************************************************************/
void QMC5883L_ReadBuffer(u8 address, u8 reg, u8 *buffer, u8 len)
{
	int i;
	IIC_Start();
	IIC_Send_Byte(address); 
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(address + 1);  //begin to read process
	IIC_Wait_Ack();
	for(i=0;i<len-1;i++)
	{
		*buffer = IIC_Read_Byte(1);   //send ack
		buffer ++;
	}
	*buffer = IIC_Read_Byte(0);       //don't send ack
	IIC_Stop();
}
/**********************************************************************************************************
*�� �� ��: QMC5883L_WriteOneByte
*����˵��: i2cд�Ĵ���
*��    ��: 
           address-������ַ
           reg-----�Ĵ�����ַ
           command-д�������
*�� �� ֵ: ��
**********************************************************************************************************/
void QMC5883L_WriteOneByte(u8 address,u8 reg,u8 command)
{ 
	IIC_Start();
  IIC_Send_Byte(address);  
	IIC_Wait_Ack();
  IIC_Send_Byte(reg);
  IIC_Wait_Ack();
  IIC_Send_Byte(command); 
  IIC_Wait_Ack();
  IIC_Stop();//����һ��ֹͣ����
}
/**********************************************************************************************************
*�� �� ��: QMC5883L_ReadProcess
*����˵��: �����ƶ�ȡ����
*��    ��: ��
*�� �� ֵ: 
			1 ��ȡ�ɹ�
**********************************************************************************************************/
short QMC5883L_ReadProcess(void)
{
	u8 buffer1[6];
	int16_t mag_temp[3];
	
	QMC5883L_ReadOneByte(QMC5883L_ADDRESS,QMC5883L_CTR1);
	QMC5883L_ReadBuffer(QMC5883L_ADDRESS, QMC5883L_R_XL, buffer1, 6);	//�Ĵ�����ַ������ȡ

	mag_temp[0] = buffer1[0] | (buffer1[1] << 8);
	mag_temp[1] = buffer1[2] | (buffer1[3] << 8);
	mag_temp[2] = buffer1[4] | (buffer1[5] << 8);
	
  mag_data[0]=mag_temp[0]*0.3f;
	mag_data[1]=mag_temp[1]*0.3f;
	mag_data[2]=mag_temp[2]*0.3f;
	
	
	
  //-32768
  //0.012207f
//  uncalibratedMagnetometer.axis.x=mag_temp[0]*0.012207f;
//	uncalibratedMagnetometer.axis.y=mag_temp[1]*0.012207f;
//	uncalibratedMagnetometer.axis.z=mag_temp[2]*0.012207f;
		//qmc5883 magnetometer calibration.
//	mag_data.mx = mag_temp[0] - mag_x_bias;
//	mag_data.my = mag_temp[1] - mag_y_bias;
//	mag_data.mz = mag_temp[2] - mag_z_bias;

	return 1;
}
/**********************************************************************************************************
*�� �� ��: qmc_offset_call()
*����˵��: ������У׼�����㲹��ֵ
*��    ��: ��
*�� �� ֵ: 
			1 ��ȡ�ɹ�
**********************************************************************************************************/
void qmc_offset_call()
{
  u8 buffer1[6];
	short flag=1;
	int16_t mag_temp[3];
	int32_t Xmax=0,Xmin=0,Ymax=0,Ymin=0,Zmax=0,Zmin=0;
	
	while(1)
	{
	QMC5883L_ReadOneByte(QMC5883L_ADDRESS,QMC5883L_CTR1);
	QMC5883L_ReadBuffer(QMC5883L_ADDRESS, QMC5883L_R_XL, buffer1, 6);	//�Ĵ�����ַ������ȡ

	mag_temp[0] = buffer1[0] | (buffer1[1] << 8);
	mag_temp[1] = buffer1[2] | (buffer1[3] << 8);
	mag_temp[2] = buffer1[4] | (buffer1[5] << 8);
	
	if(flag)
	{
	Xmax=Xmin=mag_temp[0];
	Ymax=Ymin=mag_temp[1];
	Zmax=Zmin=mag_temp[2];
	flag=0;
	}
	
	if(Xmax<mag_temp[0]) Xmax=mag_temp[0];
	if(Xmin>mag_temp[0]) Xmin=mag_temp[0];
	if(Ymax<mag_temp[1]) Ymax=mag_temp[1];
	if(Ymin>mag_temp[1]) Ymin=mag_temp[1];
	if(Zmax<mag_temp[2]) Zmax=mag_temp[2];
	if(Zmin>mag_temp[2]) Zmin=mag_temp[2];
	
	mag_x_bias=(Xmax+Xmin)/2;
	mag_y_bias=(Ymax+Ymin)/2;
	mag_z_bias=(Zmax+Zmin)/2;
	}

}