#include "RtosTaskCheck.h"
TaskRunFlag_t TaskRunFlag;
TaskRunTime_t TaskRunTime;
uint16_t Temp = 0x01;
void TaskCheck(uint16_t Task)
{
    Temp = 0x001;
    for (uint16_t i = 1 ; i<=9 ; i++)
    {
        if(Temp & Task)
        {
            switch (i)
            {
            case 1:
                TaskRunFlag.Chassis_task_RunFlag = 1;
                TaskRunFlag.Gimbal_task_RunFlag = 0;
                TaskRunFlag.ModeChoose_task_RunFlag = 0;
                TaskRunFlag.Offline_Check_task_RunFlag = 0;
                TaskRunFlag.PCReceive_task_RunFlag = 0;
                TaskRunFlag.RCReceive_task_RunFlag = 0;
                TaskRunFlag.Shoot_task_RunFlag = 0;
                TaskRunFlag.TX2_task_RunFlag = 0;
                TaskRunFlag.INS_task_RunFlag = 0;

                TaskRunTime.Chassis_task_RunTime++;
                break;

            case 2:
                TaskRunFlag.Chassis_task_RunFlag = 0;
                TaskRunFlag.Gimbal_task_RunFlag = 1;
                TaskRunFlag.ModeChoose_task_RunFlag = 0;
                TaskRunFlag.Offline_Check_task_RunFlag = 0;
                TaskRunFlag.PCReceive_task_RunFlag = 0;
                TaskRunFlag.RCReceive_task_RunFlag = 0;
                TaskRunFlag.Shoot_task_RunFlag = 0;
                TaskRunFlag.TX2_task_RunFlag = 0;
                TaskRunFlag.INS_task_RunFlag = 0;

                TaskRunTime.Gimbal_task_RunTime++;
                break;

            case 3:
                TaskRunFlag.Chassis_task_RunFlag = 0;
                TaskRunFlag.Gimbal_task_RunFlag = 0;
                TaskRunFlag.ModeChoose_task_RunFlag = 1;
                TaskRunFlag.Offline_Check_task_RunFlag = 0;
                TaskRunFlag.PCReceive_task_RunFlag = 0;
                TaskRunFlag.RCReceive_task_RunFlag = 0;
                TaskRunFlag.Shoot_task_RunFlag = 0;
                TaskRunFlag.TX2_task_RunFlag = 0;
                TaskRunFlag.INS_task_RunFlag = 0;

                TaskRunTime.ModeChoose_task_RunTime++;
                break;

            case 4:
                TaskRunFlag.Chassis_task_RunFlag = 0;
                TaskRunFlag.Gimbal_task_RunFlag = 0;
                TaskRunFlag.ModeChoose_task_RunFlag = 0;
                TaskRunFlag.Offline_Check_task_RunFlag = 1;
                TaskRunFlag.PCReceive_task_RunFlag = 0;
                TaskRunFlag.RCReceive_task_RunFlag = 0;
                TaskRunFlag.Shoot_task_RunFlag = 0;
                TaskRunFlag.TX2_task_RunFlag = 0;
                TaskRunFlag.INS_task_RunFlag = 0;

                TaskRunTime.Offline_Check_task_RunTime++;
                break;

            case 5:
                TaskRunFlag.Chassis_task_RunFlag = 0;
                TaskRunFlag.Gimbal_task_RunFlag = 0;
                TaskRunFlag.ModeChoose_task_RunFlag = 0;
                TaskRunFlag.Offline_Check_task_RunFlag = 0;
                TaskRunFlag.PCReceive_task_RunFlag = 1;
                TaskRunFlag.RCReceive_task_RunFlag = 0;
                TaskRunFlag.Shoot_task_RunFlag = 0;
                TaskRunFlag.TX2_task_RunFlag = 0;
                TaskRunFlag.INS_task_RunFlag = 0;

                TaskRunTime.PCReceive_task_RunTime++;
                break;

            case 6:
                TaskRunFlag.Chassis_task_RunFlag = 0;
                TaskRunFlag.Gimbal_task_RunFlag = 0;
                TaskRunFlag.ModeChoose_task_RunFlag = 0;
                TaskRunFlag.Offline_Check_task_RunFlag = 0;
                TaskRunFlag.PCReceive_task_RunFlag = 0;
                TaskRunFlag.RCReceive_task_RunFlag = 1;
                TaskRunFlag.Shoot_task_RunFlag = 0;
                TaskRunFlag.TX2_task_RunFlag = 0;
                TaskRunFlag.INS_task_RunFlag = 0;

                TaskRunTime.RCReceive_task_RunTime++;
                break;

            case 7:
                TaskRunFlag.Chassis_task_RunFlag = 0;
                TaskRunFlag.Gimbal_task_RunFlag = 0;
                TaskRunFlag.ModeChoose_task_RunFlag = 0;
                TaskRunFlag.Offline_Check_task_RunFlag = 0;
                TaskRunFlag.PCReceive_task_RunFlag = 0;
                TaskRunFlag.RCReceive_task_RunFlag = 0;
                TaskRunFlag.Shoot_task_RunFlag = 1;
                TaskRunFlag.TX2_task_RunFlag = 0;
                TaskRunFlag.INS_task_RunFlag = 0;

                TaskRunTime.Shoot_task_RunTime++;
                break;

            case 8:
                TaskRunFlag.Chassis_task_RunFlag = 0;
                TaskRunFlag.Gimbal_task_RunFlag = 0;
                TaskRunFlag.ModeChoose_task_RunFlag = 0;
                TaskRunFlag.Offline_Check_task_RunFlag = 0;
                TaskRunFlag.PCReceive_task_RunFlag = 0;
                TaskRunFlag.RCReceive_task_RunFlag = 0;
                TaskRunFlag.Shoot_task_RunFlag = 0;
                TaskRunFlag.TX2_task_RunFlag = 1;
                TaskRunFlag.INS_task_RunFlag = 0;

                TaskRunTime.TX2_task_RunTime++;
                break;
            
            case 9:
                TaskRunFlag.Chassis_task_RunFlag = 0;
                TaskRunFlag.Gimbal_task_RunFlag = 0;
                TaskRunFlag.ModeChoose_task_RunFlag = 0;
                TaskRunFlag.Offline_Check_task_RunFlag = 0;
                TaskRunFlag.PCReceive_task_RunFlag = 0;
                TaskRunFlag.RCReceive_task_RunFlag = 0;
                TaskRunFlag.Shoot_task_RunFlag = 0;
                TaskRunFlag.TX2_task_RunFlag = 0;
                TaskRunFlag.INS_task_RunFlag = 1;

                TaskRunTime.INS_task_RunTime++;
                break;
            
            default:
                break;
            }
        }
        Temp = Temp << 1;
    }
}
