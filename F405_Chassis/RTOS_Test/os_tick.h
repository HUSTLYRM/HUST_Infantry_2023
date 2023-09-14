#ifndef __TIM7_H
#define __TIM7_H

#include <stm32f4xx.h>
#include <stm32f4xx_conf.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/* Params START */

#define OS_TICK_APBx_CLOCK_CMD RCC_APB1PeriphClockCmd
#define OS_TICK_RCC_APBx_TIMx RCC_APB1Periph_TIM7
#define OS_TICK_TIMx TIM7
#define OS_TICK_TIMx_IRQn TIM7_IRQn
#define OS_TICK_IRQHandler TIM7_IRQHandler
#define OS_TICK_DBGMCU_TIMx_STOP DBGMCU_TIM7_STOP
/* TIM_TimeBaseInitStructure配置到源码处修改 */
/* NVIC_InitStructure中断优先级配置到源码处修改 */

/* Params END */

void OS_TICK_Configuration(void);

#endif
