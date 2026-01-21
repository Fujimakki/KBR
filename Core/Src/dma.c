/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dma.c
  * @brief   This file provides code for the configuration
  *          of all the requested memory to memory DMA transfers.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "dma.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure DMA                                                              */
/*----------------------------------------------------------------------------*/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * Enable DMA controller clock
  */
void MX_DMA_Init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/* USER CODE BEGIN 2 */

void DMA_Stream_Stop(DMA_TypeDef* DMAx, uint32_t Stream)
{
  LL_DMA_DisableStream(DMAx, Stream);
  while(LL_DMA_IsEnabledStream(DMAx, Stream)) {}

  }

void DMA_Stream_Start(DMA_TypeDef* DMAx, uint32_t Stream, uint32_t BUF_SIZE)
{
  WRITE_REG(DMA1->HIFCR , 0xFFFFFFFFUL);
  LL_DMA_SetDataLength(DMAx, Stream, BUF_SIZE);

  LL_DMA_EnableStream(DMAx, Stream);
  while(!LL_DMA_IsEnabledStream(DMA1, LL_DMA_STREAM_5)) {}
}

/* USER CODE END 2 */

