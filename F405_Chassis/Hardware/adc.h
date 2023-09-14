#ifndef _BSP_SUPER_POWER_H
#define _BSP_SUPER_POWER_H

#include <stm32f4xx.h>
#include <stm32f4xx_conf.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "tools.h"

#define ADC_NUM 3         //采集通道数目2+温度通道1
#define ADC_SAMPLE_NUM 30 //采样数量
#define ADC1_DR_Address (((u32)ADC1 + 0x4c))
#define ADC2_DR_Address (((u32)ADC2 + 0x4c))

#define ADC_AVG_SLOPE 0.0025f
#define ADC_V25 0.76f
#define ADC_TEMP_BASE 25.0f

// ADC
#define POWER_ADC_AHBxPeriphClockCmd RCC_AHB1PeriphClockCmd
#define POWER_ADC_RCC_AHBxPeriph_GPIOx RCC_AHB1Periph_GPIOA
#define POWER_ADC_GPIO_Pin_x GPIO_Pin_4
#define POWER_ADC_GPIOx GPIOA // ADC12_IN4
// ADC->DMA
#define POWER_ADC_DMA_AHBxPeriphClockCmd RCC_AHB1PeriphClockCmd
#define POWER_ADC_RCC_AHBxPeriph_DMAx RCC_AHB1Periph_DMA2
#define POWER_ADC_DMAx_Streamx DMA2_Stream0
#define POWER_ADC_ADCx_DR_Address ADC1_DR_Address
// ADC中通用ADC设置到特定位置设置
// 特定ADC配置
#define POWER_ADC_ADCx ADC1
#define POWER_ADC_ADC_RCC_APBxPeriphClockCmd RCC_APB2PeriphClockCmd
#define POWER_ADC_RCC_APBxPeriph_ADCx RCC_APB2Periph_ADC1
//一些通道的设置需要到特定位置修改

// DAC GPIO口
#define POWER_DAC_RCC_AHBxPeriphClockCmd RCC_AHB1PeriphClockCmd
#define POWER_DAC_RCC_AHBxPeriph_GPIOx RCC_AHB1Periph_GPIOA
#define POWER_DAC_GPIO_Pin_x1 GPIO_Pin_4 // ADC12_IN4
#define POWER_DAC_GPIO_Pin_x2 GPIO_Pin_5 // DAC_BQ
#define POWER_DAC_GPIOx GPIOA
// DAC
#define POWER_DAC_DAC_RCC_APBxPeriphClockCmd RCC_APB1PeriphClockCmd
#define POWER_DAC_DAC_RCC_APBxPeriph_DAC RCC_APB1Periph_DAC
#define POWER_DAC_Channel_x1 DAC_Channel_1
#define POWER_DAC_SetChannelx1Data DAC_SetChannel1Data
#define POWER_DAC_Channel_x2 DAC_Channel_2
#define POWER_DAC_SetChannelx2Data DAC_SetChannel2Data
//其它DAC设置到特定位置修改

//充放电IO口配置
#define POWER_CHARGE_RCC_AHBxPeriphClockCmd RCC_AHB1PeriphClockCmd
#define POWER_CHARGE_RCC_AHBxPeriph_GPIOx RCC_AHB1Periph_GPIOA
#define POWER_UNCHARGE_RCC_AHBxPeriph_GPIOx RCC_AHB1Periph_GPIOC
#define POWER_CHARGE_GPIO_Pin_x1 GPIO_Pin_6 // BQ_CE
#define POWER_CHARGE_GPIO_Pin_x2 GPIO_Pin_7 // Diode_Mode
#define POWER_CHARGE_GPIOx GPIOA
#define POWER_UNCHARGE_GPIO_Pin_x1 GPIO_Pin_4 // CAP_ON
#define POWER_UNCHARGE_GPIO_Pin_x2 GPIO_Pin_5 // BAT_ON
#define POWER_UNCHARGE_GPIOx GPIOC

#define Bat_on GPIO_SetBits(POWER_UNCHARGE_GPIOx, POWER_UNCHARGE_GPIO_Pin_x2)
#define Bat_off GPIO_ResetBits(POWER_UNCHARGE_GPIOx, POWER_UNCHARGE_GPIO_Pin_x2)
#define CAP_on GPIO_SetBits(POWER_UNCHARGE_GPIOx, POWER_UNCHARGE_GPIO_Pin_x1)
#define CAP_off GPIO_ResetBits(POWER_UNCHARGE_GPIOx, POWER_UNCHARGE_GPIO_Pin_x1)
#define Charge_On GPIO_SetBits(POWER_CHARGE_GPIOx, POWER_CHARGE_GPIO_Pin_x1)
#define Charge_Off GPIO_ResetBits(POWER_CHARGE_GPIOx, POWER_CHARGE_GPIO_Pin_x1)

void SuperPower_Configuration(void);

extern uint16_t ADC_ConvertedValue[ADC_NUM * 30];

#endif // !_BSP_SUPER_POWER_H
