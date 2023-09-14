/**
 ******************************************************************************
 * File Name          : CAN.c
 * Description        : This file provides code for the configuration
 *                      of the CAN instances.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
CAN_TxHeaderTypeDef can_tx; //发送消息
CAN_FilterTypeDef can_filter;
/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  hcan.Instance = CAN;
  hcan.Init.Prescaler = 3;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_CAN_MspInit(CAN_HandleTypeDef *canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (canHandle->Instance == CAN)
  {
    /* USER CODE BEGIN CAN_MspInit 0 */

    /* USER CODE END CAN_MspInit 0 */
    /* CAN clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_CAN;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_CAN;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USER CODE BEGIN CAN_MspInit 1 */

    /* USER CODE END CAN_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef *canHandle)
{

  if (canHandle->Instance == CAN)
  {
    /* USER CODE BEGIN CAN_MspDeInit 0 */

    /* USER CODE END CAN_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_12);

    /* USER CODE BEGIN CAN_MspDeInit 1 */

    /* USER CODE END CAN_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void CAN_Init()
{
  CAN_Filter_Init();
  HAL_CAN_ConfigFilter(&hcan, &can_filter);
  CAN_Transmit_Init();
  while (HAL_CAN_Start(&hcan) != HAL_OK)
    ;
}
void CAN_Transmit_Init()
{

  can_tx.StdId = 0;          //标准标识符
  can_tx.ExtId = 0;          //扩展标识符(29位)
  can_tx.IDE = CAN_ID_STD;   //使用标准帧
  can_tx.RTR = CAN_RTR_DATA; //数据帧
  can_tx.DLC = 8;
  can_tx.TransmitGlobalTime = DISABLE;
}
void CAN_Filter_Init()
{
  can_filter.FilterIdHigh = 0;
  can_filter.FilterIdLow = 0;
  can_filter.FilterMaskIdHigh = 0;
  can_filter.FilterMaskIdLow = 0;
  can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  can_filter.FilterBank = 0;
  can_filter.FilterMode = CAN_FILTERMODE_IDLIST;
  can_filter.FilterScale = CAN_FILTERSCALE_16BIT;
  can_filter.FilterActivation = ENABLE;
  can_filter.SlaveStartFilterBank = 0;
}
/**********************************************************************************************************
*函 数 名:  CANSendMag
*功能说明:  通过can总线发送数据
*形    参:  data1 --- 发送数据前4位
      data2 --- 发送数据后4位
      id ------ can ID
*返 回 值:  1 --- 成功
      0 --- 失败
**********************************************************************************************************/
unsigned char CANSendMag(float data1, float data2, uint32_t id)
{
  uint8_t tx_data[8];
  uint32_t tx_mailbox = 0;
  can_tx.StdId = id;         //标准标识符
  can_tx.ExtId = id;         //扩展标识符(29位)
  can_tx.IDE = CAN_ID_STD;   //使用标准帧
  can_tx.RTR = CAN_RTR_DATA; //数据帧
  can_tx.DLC = 8;            //数据长度

  memcpy(tx_data, &data1, 4);
  memcpy(&tx_data[4], &data2, 4);
  if (HAL_CAN_AddTxMessage(&hcan, &can_tx, tx_data, &tx_mailbox) != HAL_OK)
    return 1; //发送
  return 0;
}
unsigned char CANSendMagRaw(short *data, uint32_t id)
{
  uint8_t tx_data[8];
  uint32_t tx_mailbox = CAN_TX_MAILBOX1;
  can_tx.StdId = id;         //标准标识符
  can_tx.ExtId = id;         //扩展标识符(29位)
  can_tx.IDE = CAN_ID_STD;   //使用标准帧
  can_tx.RTR = CAN_RTR_DATA; //数据帧
  can_tx.DLC = 8;            //数据长度

  //    memcpy(tx_data,&data1,4);
  //  	memcpy(&tx_data[4],&data2,4);
  memcpy(tx_data, &data[0], 2);
  memcpy(tx_data + 2, &data[1], 2);
  memcpy(tx_data + 4, &data[2], 2);
  if (HAL_CAN_AddTxMessage(&hcan, &can_tx, tx_data, &tx_mailbox) != HAL_OK)
    return 1; //发送
  return 0;
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
