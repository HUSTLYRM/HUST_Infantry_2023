#ifndef _COUNTER_H
#define _COUNTER_H

#include <stm32f4xx.h>
#include <stm32f4xx_conf.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/* Params START */

#define COUNTER_APBx_CLOCK_CMD RCC_APB1PeriphClockCmd
#define COUNTER_RCC_APBx_TIMx RCC_APB1Periph_TIM2
#define COUNTER_TIMx TIM2
#define COUNTER_DBGMCU_TIMx_STOP DBGMCU_TIM2_STOP
/* TIM_TimeBaseInitStructure配置到源码处修改 */
#define COUNTER_SAMPLING 1000000
/* Params END */

typedef struct SysTime_t
{
    uint32_t s;
    uint32_t ms;
    uint32_t us;
} SysTime_t;

void COUNTER_Configuration(void);
float GetDeltaT(uint32_t *cnt_last);
double GetDeltaT64(uint32_t *cnt_last);
void SysTimeUpdate(void);
float GetTime_s(void);
float GetTime_ms(void);
uint64_t GetTime_us(void);
void Delay(uint32_t Delay);
float GetDeltaTtoNow(uint32_t *cnt_last);

#endif // !_COUNTER_H
