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

#define UART_PREHEADER 0xAA

#define UART_HEADER_AWS 0x31

#define UART_HEADER_RAW 0x51
#define UART_HEADER_FFT 0x52

#define UART_ADC_PAYLOAD_SIZE (FFT_SIZE << 1)
#define UART_FFT_PAYLOAD_SIZE FFT_SIZE

typedef struct __attribute__((packed)) // __attribute__((packed)) is used to remove the paddings
{
  uint8_t header[2];
  uint16_t payload[UART_ADC_PAYLOAD_SIZE];
  uint32_t crc;
}UART_ADC_TxPacket;

typedef struct __attribute__((packed))  // __attribute__((packed)) is used to remove the paddings
{
  uint8_t header[2];
  float32_t payload[FFT_SIZE];
  uint32_t crc;
} UART_FFT_TxPacket;

/* USER CODE END Private defines */

void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */

void UART_send(uint8_t* const packet, uint16_t bytesCount);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

