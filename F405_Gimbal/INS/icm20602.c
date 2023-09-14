/**
 ******************************************************************************
 * @file    icm20602.c
 * @brief   imu数据解码与处理
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "icm20602.h"
#include "main.h"
#if Robot_ID == 44
float GxOFFSET =-0.00218699244;
float GyOFFSET =0.0105982339;//注意由于云台imu安装问题，实际解算出来的pitch和roll是反的
float GzOFFSET =-0.00629083067;
float gNORM =9.6937418;
#elif Robot_ID == 47
float GxOFFSET =0.0355444364;
float GyOFFSET =0.0100790961;//注意由于云台imu安装问题，实际解算出来的pitch和roll是反的
float GzOFFSET =0.00344608445;
float gNORM =9.50813961;
#endif
/**
 * @brief IMU初始化
 *
 * @param[in] IMUReceive
 * @param[in] IMUData
 */
void ICM20602_init(IMU *IMUReceive, IMU_Data_t *IMUData)
{
    if (CALIBRATE)
        Calibrate_IMU_Offset(IMUReceive, IMUData);
    else
    {
        IMUData->GyroOffset[0] = GxOFFSET;
        IMUData->GyroOffset[1] = GyOFFSET;
        IMUData->GyroOffset[2] = GzOFFSET;
        IMUData->gNorm = gNORM;
        IMUData->AccelScale = GRAVITY / IMUData->gNorm;
    }
}
/**
 * @brief 接收数据解码
 *
 * @param[in] IMUReceive 转换前的数据
 * @param[out] IMUData  进行转换后的数据
 */
void ICM_20602_Decode(IMU *IMUReceive, IMU_Data_t *IMUData)
{
    //解码
    IMU IMUReceive_temp = *IMUReceive;
    for (unsigned char i = 0; i < 3; i++)
    {
        IMUData->Gyro[i] = (float)IMUReceive_temp.Gyro[i] * GYRO_SCALE;
        IMUData->Accel[i] = (float)IMUReceive_temp.Acc[i] * ACC_SCALE;
    }
}

/**
 * @brief 零飘校准(IMU简单标定)
 *
 * @param[in] IMUReceive
 * @param[in] IMUData
 */
void Calibrate_IMU_Offset(IMU *IMUReceive, IMU_Data_t *icm_20602)
{
    static float startTime;
    static uint16_t CaliTimes = MAX_CALIB_TIMES; // 需要获得的数
    uint32_t caliCount = 0;
    float gyroMax[3], gyroMin[3], gyroDiff[3];
    float gNormTemp, gNormMax, gNormMin, gNormDiff;

    startTime = GetTime_s();
    do
    {
        if (GetTime_s() - startTime > 10)
        {
            // 校准超时
            icm_20602->GyroOffset[0] = GxOFFSET;
            icm_20602->GyroOffset[1] = GyOFFSET;
            icm_20602->GyroOffset[2] = GzOFFSET;
            icm_20602->gNorm = gNORM;
            break;
        }

        Delay(5000);
        icm_20602->gNorm = 0;
        icm_20602->GyroOffset[0] = 0;
        icm_20602->GyroOffset[1] = 0;
        icm_20602->GyroOffset[2] = 0;

        for (uint16_t i = 0; i < CaliTimes; i++)
        {
            ICM_20602_Decode(IMUReceive, icm_20602);
            gNormTemp = sqrtf(icm_20602->Accel[0] * icm_20602->Accel[0] +
                              icm_20602->Accel[1] * icm_20602->Accel[1] +
                              icm_20602->Accel[2] * icm_20602->Accel[2]);
            icm_20602->gNorm += gNormTemp;

            for (uint8_t j = 0; j < 3; j++)
            {
                icm_20602->GyroOffset[j] += icm_20602->Gyro[j];
            }
            caliCount++;

            // 记录数据极差
            if (i == 0)
            {
                gNormMax = gNormTemp;
                gNormMin = gNormTemp;
                for (uint8_t j = 0; j < 3; j++)
                {
                    gyroMax[j] = icm_20602->Gyro[j];
                    gyroMin[j] = icm_20602->Gyro[j];
                }
            }
            else
            {
                if (gNormTemp > gNormMax)
                    gNormMax = gNormTemp;
                if (gNormTemp < gNormMin)
                    gNormMin = gNormTemp;
                for (uint8_t j = 0; j < 3; j++)
                {
                    if (icm_20602->Gyro[j] > gyroMax[j])
                        gyroMax[j] = icm_20602->Gyro[j];
                    if (icm_20602->Gyro[j] < gyroMin[j])
                        gyroMin[j] = icm_20602->Gyro[j];
                }
            }

            // 数据差异过大认为收到外界干扰，需重新校准
            gNormDiff = gNormMax - gNormMin;
            for (uint8_t j = 0; j < 3; j++)
                gyroDiff[j] = gyroMax[j] - gyroMin[j];

            if (gNormDiff > G_NORM_MAX_MIN_DIFF_THRESH ||
                gyroDiff[0] > GYRO_DIFF_THRESH ||
                gyroDiff[1] > GYRO_DIFF_THRESH ||
                gyroDiff[2] > GYRO_DIFF_THRESH)
                break;
            Delay(1000); // us
        }

        // 取平均值得到标定结果
        icm_20602->gNorm /= (float)caliCount;
        for (uint8_t i = 0; i < 3; i++)
            icm_20602->GyroOffset[i] /= (float)caliCount;

    } while (gNormDiff > G_NORM_MAX_MIN_DIFF_THRESH ||
             fabsf(icm_20602->gNorm - GRAVITY) > 0.5f ||
             gyroDiff[0] > GYRO_DIFF_THRESH ||
             gyroDiff[1] > GYRO_DIFF_THRESH ||
             gyroDiff[2] > GYRO_DIFF_THRESH ||
             fabsf(icm_20602->GyroOffset[0]) > GYRO_OFFSET_MAX ||
             fabsf(icm_20602->GyroOffset[1]) > GYRO_OFFSET_MAX ||
             fabsf(icm_20602->GyroOffset[2]) > GYRO_OFFSET_MAX ||
             caliCount < MIN_CALIB_TIMES);

    // 根据标定结果校准加速度计标度因数
    icm_20602->AccelScale = GRAVITY / icm_20602->gNorm;
}
