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

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

#include <stdbool.h>

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

#define UART_HEADER_BYTE_SIZE 2
#define UART_PREHEADER 0xAA
#define UART_HEADER_AWS 0x31
#define UART_HEADER_RAW 0x51
#define UART_HEADER_FFT 0x52

#define UART_ADC_PAYLOAD_U16_SIZE (FFT_SIZE << 1)
#define UART_ADC_PAYLOAD_BYTE_SIZE (UART_ADC_PAYLOAD_U16_SIZE << 1)

#define UART_FFT_PAYLOAD_F_SIZE ((UART_ADC_PAYLOAD_U16_SIZE >> 1) + 2)
#define UART_FFT_PAYLOAD_BYTE_SIZE (UART_FFT_PAYLOAD_F_SIZE << 2)

#define UART_CRC_BYTE_SIZE 4

#define UART_ADC_PACKET_BYTE_SIZE (UART_HEADER_BYTE_SIZE + UART_ADC_PAYLOAD_BYTE_SIZE + UART_CRC_BYTE_SIZE)
#define UART_FFT_PACKET_BYTE_SIZE (UART_HEADER_BYTE_SIZE + UART_FFT_PAYLOAD_BYTE_SIZE + UART_CRC_BYTE_SIZE)

/* USER CODE END Private defines */

void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */

void UART_send(uint8_t* const packet, uint16_t bytesCount);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

