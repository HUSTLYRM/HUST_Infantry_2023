/**
 ******************************************************************************
 * @file    pc_uart.c
 * @brief   PC串口通信
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "pc_uart.h"
#include "pc_serial.h"

unsigned char PCbuffer[PC_RECVBUF_SIZE];
unsigned char SendToPC_Buff[PC_SENDBUF_SIZE];

/**
 * @brief 与PC通信的UART
 * @param[in] void
 */
void PC_UART_Configuration(void)
{
    USART_InitTypeDef usart;
    GPIO_InitTypeDef gpio;
    NVIC_InitTypeDef nvic;

    PC_UART_GPIOx_CLOCK_CMD(PC_UART_RCC_AxBxPeriph_GPIOx, ENABLE);
    PC_UART_UARTx_CLOCK_CMD(PC_UART_RCC_AxBxPeriph_UARTx, ENABLE);

    GPIO_PinAFConfig(PC_UART_GPIOx, PC_UART_GPIO_PinSourcex1, PC_UART_GPIO_AF_USARTx);
    GPIO_PinAFConfig(PC_UART_GPIOx, PC_UART_GPIO_PinSourcex2, PC_UART_GPIO_AF_USARTx);

    gpio.GPIO_Pin = PC_UART_GPIO_Pin_x1 | PC_UART_GPIO_Pin_x2;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(PC_UART_GPIOx, &gpio);

    USART_DeInit(PC_UART_UARTx);
    usart.USART_BaudRate = PC_UART_BaudRate;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(PC_UART_UARTx, &usart);
    USART_Cmd(PC_UART_UARTx, ENABLE);

    USART_ITConfig(PC_UART_UARTx, USART_IT_IDLE, ENABLE);
    USART_DMACmd(PC_UART_UARTx, USART_DMAReq_Rx, ENABLE);
    USART_DMACmd(PC_UART_UARTx, USART_DMAReq_Tx, ENABLE);

    nvic.NVIC_IRQChannel = PC_UART_RECV_USARTx_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    nvic.NVIC_IRQChannel = PC_UART_SEND_DMAx_Streamx_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    { // RX
        DMA_InitTypeDef dma;
        PC_UART_RECV_RCC_AxBxPeriphClockCmd(PC_UART_RECV_RCC_AxBxPeriph_DMAx, ENABLE);
        DMA_DeInit(PC_UART_RECV_DMAx_Streamx);
        while (DMA_GetCmdStatus(PC_UART_RECV_DMAx_Streamx) != DISABLE)
        {
        };
        dma.DMA_Channel = PC_UART_RECV_DMA_Channel_x;
        dma.DMA_PeripheralBaseAddr = (uint32_t) & (PC_UART_UARTx->DR);
        dma.DMA_Memory0BaseAddr = (uint32_t)PCbuffer;
        dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
        dma.DMA_BufferSize = PC_RECVBUF_SIZE;
        dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
        dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        dma.DMA_Mode = DMA_Mode_Circular;
        dma.DMA_Priority = DMA_Priority_VeryHigh;
        dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
        dma.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
        dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(PC_UART_RECV_DMAx_Streamx, &dma);
        DMA_ITConfig(PC_UART_RECV_DMAx_Streamx, DMA_IT_TC, ENABLE);
        DMA_Cmd(PC_UART_RECV_DMAx_Streamx, ENABLE);
    }

    { //  TX
        DMA_InitTypeDef dma;
        PC_UART_SEND_RCC_AxBxPeriphClockCmd(PC_UART_SEND_RCC_AxBxPeriph_DMAx, ENABLE);
        DMA_DeInit(PC_UART_SEND_DMAx_Streamx);
        while (DMA_GetCmdStatus(PC_UART_SEND_DMAx_Streamx) != DISABLE)
        {
        };
        dma.DMA_Channel = PC_UART_SEND_DMA_Channel_x;
        dma.DMA_PeripheralBaseAddr = (uint32_t) & (PC_UART_UARTx->DR);
        dma.DMA_Memory0BaseAddr = (uint32_t)SendToPC_Buff;
        dma.DMA_DIR = DMA_DIR_MemoryToPeripheral;
        dma.DMA_BufferSize = PC_SENDBUF_SIZE;
        dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
        dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        dma.DMA_Mode = DMA_Mode_Normal;
        dma.DMA_Priority = DMA_Priority_VeryHigh;
        dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
        dma.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
        dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(PC_UART_SEND_DMAx_Streamx, &dma);
        DMA_Cmd(PC_UART_SEND_DMAx_Streamx, DISABLE);
        DMA_ITConfig(PC_UART_SEND_DMAx_Streamx, DMA_IT_TC, ENABLE);
    }
}
// /**
//  * @brief 与PC通信的接收中断
//  * @param[in] void
//  */
// void PC_UART_RECV_DMAx_Streamx_IRQHandler(void)
// {
//     if (DMA_GetITStatus(PC_UART_RECV_DMAx_Streamx, PC_UART_RECV_DMA_IT_TCIFx))
//     {
//         PCReceive(PCbuffer);
//         DMA_ClearFlag(PC_UART_RECV_DMAx_Streamx, PC_UART_RECV_DMA_FLAG_TCIFx);
//         DMA_ClearITPendingBit(PC_UART_RECV_DMAx_Streamx, PC_UART_RECV_DMA_IT_TCIFx);
//     }
// }

/**
 * @brief 与PC通信的串口接收中断
 * @param[in] void
 */
void PC_UART_RECV_USARTx_IRQHandler(void)
{
    if (USART_GetITStatus(PC_UART_UARTx, USART_IT_IDLE) != RESET)
    {
        (void)PC_UART_UARTx->SR;
        (void)PC_UART_UARTx->DR;
        DMA_Cmd(PC_UART_RECV_DMAx_Streamx, DISABLE);
        DMA_ClearFlag(PC_UART_RECV_DMAx_Streamx, PC_UART_RECV_DMA_FLAG_TCIFx);
        DMA_ClearITPendingBit(PC_UART_RECV_DMAx_Streamx, PC_UART_RECV_DMA_IT_TCIFx);

        PCReceive(PCbuffer);

        DMA_SetCurrDataCounter(PC_UART_RECV_DMAx_Streamx, PC_RECVBUF_SIZE);
        DMA_Cmd(PC_UART_RECV_DMAx_Streamx, ENABLE);
    }
}

/**
 * @brief 与PC通信的发送中断
 * @param[in] void
 */
void PC_UART_SEND_DMAx_Streamx_IRQHandler(void)
{
    if (DMA_GetITStatus(PC_UART_SEND_DMAx_Streamx, PC_UART_SEND_DMA_IT_TCIFx))
    {
        DMA_ClearFlag(PC_UART_SEND_DMAx_Streamx, PC_UART_SEND_DMA_FLAG_TCIFx);
        DMA_ClearITPendingBit(PC_UART_SEND_DMAx_Streamx, PC_UART_SEND_DMA_IT_TCIFx);
        while (USART_GetFlagStatus(PC_UART_UARTx, USART_FLAG_TC) == RESET);
        USART_ClearFlag(PC_UART_UARTx, USART_FLAG_TC);
    }
}
