#ifndef  _QMC5883L_H_
#define  _QMC5883L_H_
#include "main.h"
typedef uint8_t  u8;
//QMC5883L internal reg addr

#define QMC5883L_ADDRESS	0x0D<<1		//7-bit address turns to 8-bit one

#define QMC5883L_R_XL 0x00
#define QMC5883L_R_XM 0x01
#define QMC5883L_R_YL 0x02
#define QMC5883L_R_YM 0x03
#define QMC5883L_R_ZL 0x04
#define QMC5883L_R_ZM 0x05

#define QMC5883L_CTR1 0x09	//设置测量模式		0x1D
#define QMC5883l_CTR2 0x0A	//0x00
#define QMC5883L_PFBR 0x0B	//0x01
#define QMC5883L_ID   0X0D  //读取ID 正常为0xFF


typedef struct
{
	float mx;
	float my;
	float mz;
}magnetometer_t;

u8 QMC5883L_ReadOneByte(u8 address, u8 reg);
u8 Init_QMC5883(void);
void QMC5883L_WriteOneByte(u8 address,u8 reg,u8 command);
void QMC5883L_ReadBuffer(u8 address, u8 reg, u8 *buffer, u8 len);
void qmc_offset_call();
short QMC5883L_ReadProcess(void);



#endif