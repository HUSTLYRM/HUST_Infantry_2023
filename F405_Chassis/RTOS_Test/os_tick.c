/**
 ******************************************************************************
 * @file    os_tick.c
 * @brief   FreeRTOS CPU运行情况计时
 ******************************************************************************
 * @attention
 ******************************************************************************
 */
#include "os_tick.h"
#include "stm32f4xx_dbgmcu.h"
/**
 * @brief FreeRTOS用于统计CPU运行情况的定时器配置
 * @param[in] void
 */
void OS_TICK_Configuration(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    OS_TICK_APBx_CLOCK_CMD(OS_TICK_RCC_APBx_TIMx, ENABLE);

    DBGMCU_Config(OS_TICK_DBGMCU_TIMx_STOP, ENABLE);

    /*  0.02ms */
    TIM_TimeBaseInitStructure.TIM_Period = 50 - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 84 - 1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(OS_TICK_TIMx, &TIM_TimeBaseInitStructure);

    TIM_ClearFlag(OS_TICK_TIMx, TIM_FLAG_Update);

    TIM_ITConfig(OS_TICK_TIMx, TIM_IT_Update, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = OS_TICK_TIMx_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(OS_TICK_TIMx, ENABLE);
}

/* global params */

volatile uint32_t CPU_RunTime = 0UL; // CPU计时

/**
 * @brief FreeRTOS用于统计CPU运行情况的定时器中断处理
 * @param[in] void
 */
void OS_TICK_IRQHandler(void)
{
    if (TIM_GetITStatus(OS_TICK_TIMx, TIM_IT_Update) != RESET)
    {
        CPU_RunTime++;
        TIM_ClearITPendingBit(OS_TICK_TIMx, TIM_FLAG_Update);
    }
}
