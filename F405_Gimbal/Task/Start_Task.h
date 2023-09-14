#ifndef START_TASK_H
#define START_TASK_H
#include "main.h"
#include "GimbalEstimateTask.h"

void CPU_task(void *pvParameters);
void start_task(void *pvParameters);
void startTast(void);
void Robot_Stop(void);
void Robot_Recover(void);
void Shoot_Stop(void);
void Shoot_Recover(void);
void PC_Rst(void);

/*  根据创建的任务列举Task Handle */
enum TASK_LIST
{
    GIMBAL_ESTIMATE_TASK,
    RC_TASK,
    TX2_TASK,
    MODECHOOSE_TASK,
    CPU_TASK,
    OFFLINE_TASK,
    SDCARD_TASK,
    GIMBAL_TASK,
    CHASSIS_TASK,
    CONTROL_TASK,
    SHOOT_TASK,
    SD_TASK,
    TASK_NUM,
};

#endif
