/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

#include <stdbool.h>
#include <strings.h>


/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

enum PacketType
{

  // TODO Find another way to use the original enums

  AWS = 0x31, // New AVRG_WINDOW_SIZE value
  RAW = 0x51, // Raw data from ADC
  FFT = 0x52  // Calculated FFT magnitudes

};

/* USER CODE END Private defines */

void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */

void sendUart(uint32_t* buffer, uint16_t size, uint8_t type);
bool readUart(uint8_t *buffer, uint16_t size);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

