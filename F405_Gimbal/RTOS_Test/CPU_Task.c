/**
 ******************************************************************************
 * @file    CPU_Task.c
 * @brief   记录堆栈使用情况以及CPU用时
 ******************************************************************************
 * @attention
 ******************************************************************************
 */
#include "CPU_Task.h"
#include "Start_Task.h"
#include "debug.h"

#ifdef DEBUG_MODE_FREERTOS

uint8_t CPU_RunInfo1[CPU_INFO_LIST_LENGTH]; //保存任务运行时间信息 分别是：任务名 任务状态 优先级 剩余栈 任务序号
uint8_t CPU_RunInfo2[CPU_INFO_LIST_LENGTH]; //保存任务运行时间信息 分别是：任务名 运行计数  使用率
unsigned portBASE_TYPE Mark[TASK_NUM];      //观察任务堆栈使用情况
uint32_t CPU_high_water;

/* Task Handle  */
extern TaskHandle_t User_Tasks[TASK_NUM];

void CPU_task(void *pvParameters)
{

    while (1)
    {

#if INCLUDE_uxTaskGetStackHighWaterMark
        memset(CPU_RunInfo1, 0, CPU_INFO_LIST_LENGTH); //信息缓冲区清零

        vTaskList((char *)&CPU_RunInfo1); //获取任务运行时间信息

        memset(CPU_RunInfo2, 0, CPU_INFO_LIST_LENGTH); //信息缓冲区清零
        for (size_t i = 0; i < TASK_NUM; i++)
        {
            Mark[i] = uxTaskGetStackHighWaterMark(User_Tasks[i]);
            /* code */
        }
        // CPU任务占用剩余
        CPU_high_water = uxTaskGetStackHighWaterMark(NULL);
        vTaskGetRunTimeStats((char *)&CPU_RunInfo2);
#ifdef DEBUG_MODE
        PRINTF("---------------------------------------------\r\n");
        PRINTF("Name    State Priority     RemainStack     Sequence\r\n");
        PRINTF("%s", CPU_RunInfo1);
        PRINTF("---------------------------------------------\r\n");

        PRINTF("Name         RunCounts    UsingRate\r\n");
        PRINTF("%s", CPU_RunInfo2);
        PRINTF("---------------------------------------------\r\n\n");
#endif
        vTaskDelay(1000); /* 延时 500 个 tick */
#endif
    }
}

#endif // DEBUG
