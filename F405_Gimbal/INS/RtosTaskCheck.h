#ifndef __RTOSTASKCHECK_H__
#define __RTOSTASKCHECK_H__

#include "stm32f4xx.h"

#define ChassisTask 0x01
#define GimbalTask 0x02
#define ModeChooseTask 0x04
#define OfflineCheckTask 0x08
#define PCReceiveTask 0x10
#define RCReceiveTask 0x20
#define ShootTask 0x40
#define TX2Task 0x80
#define INSTask 0x100

typedef struct
{
  uint8_t Chassis_task_RunFlag;
  uint8_t Gimbal_task_RunFlag;
  uint8_t Shoot_task_RunFlag;
  uint8_t ModeChoose_task_RunFlag;
  uint8_t TX2_task_RunFlag;
  uint8_t Offline_Check_task_RunFlag;
  uint8_t RCReceive_task_RunFlag;
  uint8_t PCReceive_task_RunFlag;
  uint8_t INS_task_RunFlag;
}TaskRunFlag_t;

typedef struct
{
  uint32_t Chassis_task_RunTime;
  uint32_t Gimbal_task_RunTime;
  uint32_t Shoot_task_RunTime;
  uint32_t ModeChoose_task_RunTime;
  uint32_t TX2_task_RunTime;
  uint32_t Offline_Check_task_RunTime;
  uint32_t RCReceive_task_RunTime;
  uint32_t PCReceive_task_RunTime;
  uint32_t INS_task_RunTime;
}TaskRunTime_t;

extern TaskRunFlag_t TaskRunFlag;
extern TaskRunTime_t TaskRunTime;

void TaskCheck(uint16_t Task);

#endif // !RTOSTASKCHECK_H

