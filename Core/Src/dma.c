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
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream6_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/* USER CODE BEGIN 2 */

void DMA_resetAllIT(DMA_TypeDef* DMAx)
{
  WRITE_REG(DMAx->LIFCR , 0xFFFFFFFFUL);
  WRITE_REG(DMAx->HIFCR , 0xFFFFFFFFUL);
}

void DMA_startStream(DMA_TypeDef* DMAx, uint32_t Stream, uint32_t BUF_SIZE)
{

  if(BUF_SIZE != 0)
  {
    LL_DMA_SetDataLength(DMAx, Stream, BUF_SIZE);
  }

  DMA_resetAllIT(DMAx);

  while(!LL_DMA_IsEnabledStream(DMAx, Stream))
  {
    LL_DMA_EnableStream(DMAx, Stream);
  }
}

void DMA_stopStream(DMA_TypeDef* DMAx, uint32_t Stream)
{
  while(LL_DMA_IsEnabledStream(DMAx, Stream))
  {
    LL_DMA_DisableStream(DMAx, Stream);
  }
}

/* USER CODE END 2 */

