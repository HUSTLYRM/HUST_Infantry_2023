#ifndef _GYRO_UART_H
#define _GYRO_UART_H

#include <stm32f4xx.h>
#include <stm32f4xx_conf.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "mySensors.h"

/* MOTOR1 */
#define GYRO_UART_GPIOx_CLOCK_CMD RCC_AHB1PeriphClockCmd
#define GYRO_UART_RCC_AxBxPeriph_GPIOx RCC_AHB1Periph_GPIOC
#define GYRO_UART_UARTx_CLOCK_CMD RCC_APB1PeriphClockCmd
#define GYRO_UART_RCC_AxBxPeriph_UARTx RCC_APB1Periph_UART4
#define GYRO_UART_GPIOx GPIOC
#define GYRO_UART_UARTx UART4
#define GYRO_UART_GPIO_AF_USARTx GPIO_AF_UART4
#define GYRO_UART_GPIO_PinSourcex1 GPIO_PinSource10
#define GYRO_UART_GPIO_PinSourcex2 GPIO_PinSource11
#define GYRO_UART_GPIO_Pin_x1 GPIO_Pin_10
#define GYRO_UART_GPIO_Pin_x2 GPIO_Pin_11
#define GYRO_UART_BaudRate 230400

#define GYRO_UART_RECV_USARTx_IRQHandler UART5_IRQHandler
#define GYRO_UART_RECV_USARTx_IRQn UART4_IRQn

#define GYRO_UART_RECV_DMAx_Streamx_IRQn DMA1_Stream2_IRQn
#define GYRO_UART_RECV_DMAx_Streamx DMA1_Stream2
#define GYRO_UART_RECV_RCC_AxBxPeriphClockCmd RCC_AHB1PeriphClockCmd
#define GYRO_UART_RECV_RCC_AxBxPeriph_DMAx RCC_AHB1Periph_DMA1
#define GYRO_UART_RECV_DMA_Channel_x DMA_Channel_4
#define GYRO_UART_RECV_DMAx_Streamx_IRQHandler DMA1_Stream2_IRQHandler
#define GYRO_UART_RECV_DMA_FLAG_TCIFx DMA_FLAG_TCIF2
#define GYRO_UART_RECV_DMA_IT_TCIFx DMA_IT_TCIF2

#define GYRO_RECV_BUFF_SIZE 11

void GYRO_UART_Configuration(void);

#endif // !_GYRO_UART_H
