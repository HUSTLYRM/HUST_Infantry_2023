/**
 ******************************************************************************
 * @file    GimbalEstimateTask.c
 * @brief   云台位姿估计任务
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "GimbalEstimateTask.h"
#include "RtosTaskCheck.h"

/* IMU */
IMU IMUReceive;
IMU_Data_t IMUData;

/**
 * @brief 云台位姿估计任务
 * @param[in] void
 */
uint32_t GimbalEstimate_high_water;
void GimbalEstimate_task(void *pvParameters)
{
    portTickType xLastWakeTime;
    const portTickType xFrequency = 1; // 1kHZ

    //适当延时
    vTaskDelay(50);

    while (1)
    {
      TaskCheck(INSTask);
        xLastWakeTime = xTaskGetTickCount();

        INS_Task(&IMUReceive, &IMUData);

        /*  喂狗 */
//        xEventGroupSetBits(xCreatedEventGroup, GIMBAL_ESTIMATE_BIT); //标志位置一

        /*  延时  */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
#if INCLUDE_uxTaskGetStackHighWaterMark
				GimbalEstimate_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}
