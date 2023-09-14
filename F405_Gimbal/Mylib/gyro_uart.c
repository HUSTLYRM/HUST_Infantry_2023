/**
 ******************************************************************************
 * @file    GYRO_UART.c
 * @brief   485通信
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "gyro_uart.h"

/*  数据定义  */
uint8_t gyroRecvBuff[GYRO_RECV_BUFF_SIZE] = {0};

void GYRO_UART_Configuration(void)
{
    USART_InitTypeDef usart;
    GPIO_InitTypeDef gpio;
    NVIC_InitTypeDef nvic;

    /* 如果是通信频率不够，需要加入这句  */
    // USART_OverSampling8Cmd(GYRO_UART_RCC_AxBxPeriph_UARTx, ENABLE);

    GYRO_UART_GPIOx_CLOCK_CMD(GYRO_UART_RCC_AxBxPeriph_GPIOx, ENABLE);
    GYRO_UART_UARTx_CLOCK_CMD(GYRO_UART_RCC_AxBxPeriph_UARTx, ENABLE);

    GPIO_PinAFConfig(GYRO_UART_GPIOx, GYRO_UART_GPIO_PinSourcex1, GYRO_UART_GPIO_AF_USARTx);
    GPIO_PinAFConfig(GYRO_UART_GPIOx, GYRO_UART_GPIO_PinSourcex2, GYRO_UART_GPIO_AF_USARTx);

    gpio.GPIO_Pin = GYRO_UART_GPIO_Pin_x1 | GYRO_UART_GPIO_Pin_x2;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GYRO_UART_GPIOx, &gpio);

    USART_DeInit(GYRO_UART_UARTx);
    usart.USART_BaudRate = GYRO_UART_BaudRate;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(GYRO_UART_UARTx, &usart);

    USART_ITConfig(GYRO_UART_UARTx, USART_IT_IDLE, ENABLE);

    USART_Cmd(GYRO_UART_UARTx, ENABLE);

    USART_DMACmd(GYRO_UART_UARTx, USART_DMAReq_Rx, ENABLE);
    USART_DMACmd(GYRO_UART_UARTx, USART_DMAReq_Tx, ENABLE);
    USART_ClearFlag(GYRO_UART_UARTx, USART_FLAG_TC);

    nvic.NVIC_IRQChannel = GYRO_UART_RECV_USARTx_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    { // RX
        DMA_InitTypeDef dma;
        GYRO_UART_RECV_RCC_AxBxPeriphClockCmd(GYRO_UART_RECV_RCC_AxBxPeriph_DMAx, ENABLE);
        DMA_DeInit(GYRO_UART_RECV_DMAx_Streamx);
        while (DMA_GetCmdStatus(GYRO_UART_RECV_DMAx_Streamx) != DISABLE)
        {
        };
        dma.DMA_Channel = GYRO_UART_RECV_DMA_Channel_x;
        dma.DMA_PeripheralBaseAddr = (uint32_t) & (GYRO_UART_UARTx->DR);
        dma.DMA_Memory0BaseAddr = (uint32_t)gyroRecvBuff;
        dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
        dma.DMA_BufferSize = GYRO_RECV_BUFF_SIZE;
        dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
        dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        dma.DMA_Mode = DMA_Mode_Circular;
        dma.DMA_Priority = DMA_Priority_Medium;
        dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
        dma.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
        dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(GYRO_UART_RECV_DMAx_Streamx, &dma);
        DMA_ITConfig(GYRO_UART_RECV_DMAx_Streamx, DMA_IT_TC, ENABLE);
        DMA_Cmd(GYRO_UART_RECV_DMAx_Streamx, ENABLE);
    }
}

void GYRO_UART_RECV_USARTx_IRQHandler(void)
{
    if (USART_GetITStatus(GYRO_UART_UARTx, USART_IT_IDLE) != RESET)
    {
        (void)GYRO_UART_UARTx->SR;
        (void)GYRO_UART_UARTx->DR;
        DMA_Cmd(GYRO_UART_RECV_DMAx_Streamx, DISABLE);
        DMA_ClearFlag(GYRO_UART_RECV_DMAx_Streamx, GYRO_UART_RECV_DMA_FLAG_TCIFx);
        DMA_ClearITPendingBit(GYRO_UART_RECV_DMAx_Streamx, GYRO_UART_RECV_DMA_IT_TCIFx);

        decode_gyro(gyroRecvBuff);

        DMA_SetCurrDataCounter(GYRO_UART_RECV_DMAx_Streamx, GYRO_RECV_BUFF_SIZE);
        DMA_Cmd(GYRO_UART_RECV_DMAx_Streamx, ENABLE);
    }
}
