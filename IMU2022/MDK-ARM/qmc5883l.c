/**********************************************************************************************************
 * @文件     qmc5883L.c
 * @说明     模拟i2c读取磁力计数据
 * @版本  	 V1.0
 * @作者     刘垚/林一成
 * @日期     2019.01
 
 * @版本   	 V2.0 
 * @更新日志	 将磁力计芯片由ist8310改为qmc5883L,并修改寄存器库
 * @作者 		 陈志鹏
 * @日期		 2021.3
**********************************************************************************************************/
#include "main.h"

float mag_data[3];//磁力计数据
float mag_x_bias = 0;//磁力计数据校准后的偏差
float mag_y_bias = 0;
float mag_z_bias = 0;

float mag_x_scale = 1;//磁力计数据校准后的缩放因子
float mag_y_scale = 1;
float mag_z_scale = 1;

extern short sys_flag;
u8 QMC5883L_reg1 = 0;//should be 0x1D
u8 QMC5883L_reg2 = 0;//should be 0x01

/**********************************************************************************************************
*函 数 名: Init_QMC5883
*功能说明: QMC5883初始化，并读取寄存器1和2，并判断是否读取成功初始化
*形    参: 无
*返 回 值: 
			1----reg1错误
			2----reg2错误
			3----name错误
**********************************************************************************************************/
u8 Init_QMC5883(void)
{
  unsigned char ID;
	//检测芯片是否存在
	ID=QMC5883L_ReadOneByte(QMC5883L_ADDRESS,QMC5883L_ID);
	if(ID!=0XFF) 
	return 1;
	                                                 // 量程2G  0x0D   8G 0X1D  
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
*函 数 名: QMC5883L_ReadOneByte
*功能说明: i2c读取一位寄存器数据
*形    参: 
           address-器件地址
           reg-----寄存器地址
*返 回 值: 寄存器的值
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
  IIC_Send_Byte(address + 1); //进入接收模式 
  IIC_Wait_Ack();
  temp = IIC_Read_Byte(0);  //读寄存器 3
  IIC_Stop();//产生一个停止条件 
  return temp;
}
/**********************************************************************************************************
*函 数 名: QMC5883L_ReadBuffer
*功能说明: i2c读取寄存器数据
*形    参: 
           address-器件地址
           reg-----寄存器地址
           buffer--数组指针
           len-----长度
*返 回 值: 无
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
*函 数 名: QMC5883L_WriteOneByte
*功能说明: i2c写寄存器
*形    参: 
           address-器件地址
           reg-----寄存器地址
           command-写入的数据
*返 回 值: 无
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
  IIC_Stop();//产生一个停止条件
}
/**********************************************************************************************************
*函 数 名: QMC5883L_ReadProcess
*功能说明: 磁力计读取数据
*形    参: 无
*返 回 值: 
			1 读取成功
**********************************************************************************************************/
short QMC5883L_ReadProcess(void)
{
	u8 buffer1[6];
	int16_t mag_temp[3];
	
	QMC5883L_ReadOneByte(QMC5883L_ADDRESS,QMC5883L_CTR1);
	QMC5883L_ReadBuffer(QMC5883L_ADDRESS, QMC5883L_R_XL, buffer1, 6);	//寄存器地址连续读取

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
*函 数 名: qmc_offset_call()
*功能说明: 磁力计校准，计算补偿值
*形    参: 无
*返 回 值: 
			1 读取成功
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
	QMC5883L_ReadBuffer(QMC5883L_ADDRESS, QMC5883L_R_XL, buffer1, 6);	//寄存器地址连续读取

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