#include "main.h"

Gyro gyro;
Motor motors[4];

extern float PitchMotorReceive,YawMotorReceive;//Pitch,Yaw电机角度
extern float yaw_gyro_temp,pitch_gyro_temp;
extern Gyro_Typedef GyroReceive;//陀螺仪数据
extern Gimbal_Typedef Gimbal;
extern Disconnect Robot_Disconnect;
extern RobotInit_Struct Infantry;

void decode_gyro(uint8_t *buf)
{
//    char check_sum = 0;
    float temp;
//    for (int i = 0; i < 10; i++)
//    {
//        check_sum += buf[i];
//    }
//    if ((int8_t)check_sum != buf[10])
//    {
//        return;
//    }
    if (buf[0] == 0x55)
    {
        switch (buf[1])
        {
					case 0x51:
							gyro.a_x = (float)((buf[3] << 8) | buf[2]) * 0.004785f;
							gyro.a_y = (float)((buf[5] << 8) | buf[4]) * 0.004785f;
							gyro.a_z = (float)((buf[7] << 8) | buf[6]) * 0.004785f;
							gyro.a_t = (float)((buf[9] << 8) | buf[8]) * 0.01f;
							break;
					case 0x52:
							gyro.w_x = (float)((buf[3] << 8) | buf[2]) * 0.06104f;
							gyro.w_y = (float)((buf[5] << 8) | buf[4]) * 0.06104f;
							gyro.w_z = (float)((buf[7] << 8) | buf[6]) * 0.06104f;
							gyro.w_t = (float)((buf[9] << 8) | buf[8]) * 0.01f;
					    
					if(ABS(gyro.w_z)<30){
							GyroReceive.GZ = gyro.w_z;
							GyroReceive.GY = gyro.w_y;
							GyroReceive.GY*= Infantry.gyro_pn;
					}
							break;
					case 0x53:
							gyro.x_roll = (float)((buf[3] << 8) | buf[2]) * 0.005493f;
							gyro.x_pitch = (float)((buf[5] << 8) | buf[4]) * 0.005493f;
							temp = (float)((buf[7] << 8) | buf[6]) * 0.005493f;
							gyro.x_yaw = temp + round((gyro.x_yaw - temp) / 360.0f) * 360.0f;
							gyro.x_t = (float)((buf[9] << 8) | buf[8]) * 0.01f;
					
							pitch_gyro_temp = gyro.x_pitch;
							 //程序安全
							 if(ABS(pitch_gyro_temp)<50.0f)
							 {
							 GyroReceive.PITCH = pitch_gyro_temp*Infantry.gyro_pn;
							 }
							 else
							 {
							 GyroReceive.PITCH = Gimbal.Pitch.MotorTransAngle;
							 }
							 
							 //程序安全
							 yaw_gyro_temp = gyro.x_yaw;
							 if(yaw_gyro_temp < 180.0f && yaw_gyro_temp > -180.0f)
							 {
								GyroReceive.YAW = yaw_gyro_temp;
								Robot_Disconnect.Gyro_DisConnect=0;
								}
							break;
					default:
							break;
        }
    }
}
