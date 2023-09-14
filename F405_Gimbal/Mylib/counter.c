/**
 ******************************************************************************
 * @file    counter.c
 * @brief   计时器，用于计算程序用时或者延时
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "main.h"

/*  计时参数定义  */

uint32_t LastCNT32; // 上一个64位值
SysTime_t SysTime;  //系统时间，秒，毫秒，微秒

/**
 * @brief 计数器配置
 * @param[in] void
 */
void COUNTER_Configuration(void)
{
    /* Params Init */
    LastCNT32 = 0;
    memcpy(&SysTime, &LastCNT32, sizeof(SysTime_t)); //初始化为0

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

    COUNTER_APBx_CLOCK_CMD(COUNTER_RCC_APBx_TIMx, ENABLE);

    //DBGMCU_Config(COUNTER_DBGMCU_TIMx_STOP, ENABLE);

    /*  1us */
    TIM_TimeBaseInitStructure.TIM_Period = UINT32_MAX;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 84 - 1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(COUNTER_TIMx, &TIM_TimeBaseInitStructure);

    TIM_Cmd(COUNTER_TIMx, ENABLE);
}

/**
 * @brief 计算两次调用之间的时间差
 * @param[in] cnt_last 上一次计数值
 */
float GetDeltaT(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = COUNTER_TIMx->CNT;
    float dt = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(COUNTER_SAMPLING));
    *cnt_last = cnt_now;

    return dt;
}

/**
 * @brief 计算两次调用之间的时间差(双精度)
 * @param[in] cnt_last 上一次计数值
 */
double GetDeltaT64(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = COUNTER_TIMx->CNT;
    double dt = ((uint32_t)(cnt_now - *cnt_last)) / ((double)(COUNTER_SAMPLING));
    *cnt_last = cnt_now;

    return dt;
}

/**
 * @brief 更新总时间 s/ms/us
 * @param[in] void
 */
void SysTimeUpdate(void)
{
    volatile uint32_t cnt_now = COUNTER_TIMx->CNT;
    static uint64_t CNT_TEMP1, CNT_TEMP2;
    uint32_t diff_cnt = cnt_now - LastCNT32;

    CNT_TEMP1 = (diff_cnt + SysTime.us) / 1000;             // ms
    CNT_TEMP2 = (diff_cnt + SysTime.us) - CNT_TEMP1 * 1000; // us
    SysTime.ms += CNT_TEMP1;
    SysTime.us = CNT_TEMP2;
    if (SysTime.ms >= 1000)
    {
        SysTime.s += 1;
        SysTime.ms -= 1000;
    }

    LastCNT32 = cnt_now;
}

/**
 * @brief 获取从程序开始到现在的时间 /s
 * @param[in] void
 */
float GetTime_s(void)
{
    SysTimeUpdate();

    float Timelinef32 = SysTime.s + SysTime.ms * 0.001f + SysTime.us * 0.000001f;

    return Timelinef32;
}

/**
 * @brief 获取从程序开始到现在的时间 /s
 * @param[in] void
 */
float GetTime_ms(void)
{
    SysTimeUpdate();

    float Timelinef32 = SysTime.s * 1000 + SysTime.ms + SysTime.us * 0.001f;

    return Timelinef32;
}

/**
 * @brief 获取从程序开始到现在的时间 /us
 * @param[in] void
 */
uint64_t GetTime_us(void)
{
    SysTimeUpdate();

    uint64_t Timelinef32 = SysTime.s * 1000000 + SysTime.ms * 1000 + SysTime.us;

    return Timelinef32;
}

/**
 * @brief 延时 /us
 * @param[in] void
 */
void Delay(uint32_t Delay)
{
    volatile uint32_t tickstart = COUNTER_TIMx->CNT;
    uint32_t wait = Delay;

    while ((COUNTER_TIMx->CNT - tickstart) < wait)
    {
    }
}

/**
 * @brief 计算到输入时间值到现在的时间
 * @param[in] cnt_last 上一次计数值
 */
float GetDeltaTtoNow(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = COUNTER_TIMx->CNT;
    float dt = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(COUNTER_SAMPLING));

    return dt;
}
