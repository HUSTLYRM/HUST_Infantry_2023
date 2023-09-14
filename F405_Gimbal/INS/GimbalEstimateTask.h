#ifndef GIMBAL_ESTIMATE_TASK_H
#define GIMBAL_ESTIMATE_TASK_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "icm20602.h"
#include "counter.h"
#include "ins.h"

#include "iwdg.h"

void GimbalEstimate_task(void *pvParameters);

extern IMU IMUReceive;
extern IMU_Data_t IMUData;

#endif // !GIMBAL_ESTIMATE_TASK_H
