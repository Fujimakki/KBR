/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */

#include "dma.h"
#include <string.h>

/* USER CODE END 0 */

/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2|LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART2 DMA Init */

  /* USART2_TX Init */
  LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_6, LL_DMA_CHANNEL_4);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_6, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_6, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_6, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_6, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_6, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_6, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_6, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_6);

  /* USART2 interrupt Init */
  NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART2_IRQn);

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.BaudRate = 2250000;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART2);
  LL_USART_Enable(USART2);
  /* USER CODE BEGIN USART2_Init 2 */

  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_6, LL_USART_DMA_GetRegAddr(USART2));
  //LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_6);
  LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_6);


  /* USER CODE END USART2_Init 2 */

}

/* USER CODE BEGIN 1 */

uint32_t CRC_calc(const uint32_t *const payload, uint16_t pldSize)
{
  LL_CRC_ResetCRCCalculationUnit(CRC);

  for (uint16_t i = 0; i < pldSize; i++) {
    uint32_t data = (payload[i]);
    LL_CRC_FeedData32(CRC, __RBIT(data));
  }

  uint32_t crc = ~(__RBIT(LL_CRC_ReadData32(CRC)));
  return crc;
}


void UART_send(uint8_t* const packet, uint16_t bytesCount)
{
  // uint32_t primask_bit = __get_PRIMASK(); // Remeber the state of primask
  // __disable_irq();  // Disable interrupts

  uint32_t crcValue = CRC_calc((uint32_t* const)(packet + 2), (bytesCount - 6) / sizeof(uint32_t));
  memcpy(packet + bytesCount - sizeof(crcValue), &crcValue, sizeof(crcValue));

#ifdef DBG
  extern uint32_t delta_time[];
  static uint8_t dt_counter = 0;
  delta_time[dt_counter % 4] = DWT->CYCCNT;
#endif // DBG

  while(LL_DMA_IsEnabledStream(DMA1, LL_DMA_STREAM_6)) {}

#ifdef DBG
  delta_time[dt_counter + 1 % 4] = DWT->CYCCNT - delta_time[dt_counter % 4];
  dt_counter++;
#endif // DBG

  LL_DMA_ClearFlag_TC6(DMA1);
  LL_DMA_ClearFlag_HT6(DMA1);
  LL_DMA_ClearFlag_TE6(DMA1);

  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_6, (uint32_t)packet);

  LL_USART_EnableDMAReq_TX(USART2);

  DMA_resetAllIT(DMA1);
  DMA_startStream(DMA1, LL_DMA_STREAM_6, bytesCount);

#ifdef DBG
  DWT->CYCCNT = delta_time[4] = 0;
#endif // DBG

  while(!LL_USART_IsActiveFlag_TC(USART2)) {}

#ifdef DBG
  delta_time[4] = DWT->CYCCNT;
#endif // DBG

  // if(!primask_bit)
  // {
  //   __enable_irq(); // Enable interrupts if they were enabled before the function
  // }
}

/* USER CODE END 1 */
