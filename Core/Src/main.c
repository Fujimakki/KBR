/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "crc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "fft_mag.h"
#include "custom_flags.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
  RAW = 0x51,
  FFT = 0x52
} PacketType;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ADC_DMA_BUF_SIZE FFT_SIZE

#define QUANT_STEP (3.3f / 4096.0f)

#define UART_CRC_BYTES 4

#define UART_RAW_PAYLOAD_FLOATS FFT_SIZE
#define UART_RAW_PAYLOAD_BYTES (UART_RAW_PAYLOAD_FLOATS * sizeof(float32_t))
#define UART_RAW_PACKET_SIZE (2 + UART_RAW_PAYLOAD_BYTES + UART_CRC_BYTES)

#define UART_FFT_PAYLOAD_FLOATS (FFT_SIZE / 2)
#define UART_FFT_PAYLOAD_BYTES (UART_FFT_PAYLOAD_FLOATS * sizeof(float32_t))
#define UART_FFT_PACKET_SIZE (2 + UART_FFT_PAYLOAD_BYTES + UART_CRC_BYTES)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

uint32_t crcCalc(const uint32_t* const payload, uint16_t pldSize);
void buildPacket(uint32_t* const pldData, uint8_t* const packet, const PacketType type);

void readAdc(uint16_t* const buffer, uint16_t size);
void sendUart(uint8_t* buffer, uint16_t size);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  static uint16_t adcDmaBuf[ADC_DMA_BUF_SIZE];
  static float32_t rawData[ADC_DMA_BUF_SIZE];

  static float32_t magnitudes[UART_RAW_PAYLOAD_FLOATS];
  static uint8_t txPacket[UART_RAW_PACKET_SIZE];
  txPacket[0] = 0xAA;

  arm_rfft_fast_instance_f32 S;
  arm_rfft_fast_init_f32(&S, FFT_SIZE);


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_CRC_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  LL_TIM_EnableAllOutputs(TIM1);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
  LL_TIM_EnableCounter(TIM1);


  LL_ADC_Enable(ADC1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	readAdc(adcDmaBuf, ADC_DMA_BUF_SIZE);

	for(uint16_t i = 0; i < ADC_DMA_BUF_SIZE; i++)
	{
	  rawData[i] = adcDmaBuf[i] * QUANT_STEP;
	}

	buildPacket((uint32_t*)rawData, txPacket, RAW);
	sendUart(txPacket, UART_RAW_PACKET_SIZE);

	fftMagCalc(&S, rawData, magnitudes);

	buildPacket((uint32_t*)magnitudes, txPacket, FFT);
	sendUart(txPacket, UART_FFT_PACKET_SIZE);
  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_4)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_PWR_DisableOverDriveMode();
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_4, 144, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  while (LL_PWR_IsActiveFlag_VOS() == 0)
  {
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(144000000);
  LL_SetSystemCoreClock(144000000);
  LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);
}

/* USER CODE BEGIN 4 */

uint32_t crcCalc(const uint32_t* const payload, uint16_t pldSize)
{
	LL_CRC_ResetCRCCalculationUnit(CRC);

  for(uint16_t i = 0; i < pldSize; i++)
  {
	uint32_t data = payload[i];
	LL_CRC_FeedData32(CRC, __RBIT(data));
  }

  uint32_t crc = ~( __RBIT( LL_CRC_ReadData32(CRC) ) );
  return crc;
}

void buildPacket(uint32_t* const pldData, uint8_t* const packet, const PacketType type)
{
  packet[1] = type;

  uint16_t pldSizeBytes;
  uint16_t pldSizeFloats;
  switch(type)
  {
  case RAW:
	pldSizeBytes = UART_RAW_PAYLOAD_BYTES;
	pldSizeFloats = UART_RAW_PAYLOAD_FLOATS;
	break;

  case FFT:
	pldSizeBytes = UART_FFT_PAYLOAD_BYTES;
	pldSizeFloats = UART_FFT_PAYLOAD_FLOATS;
	break;

  default:
	pldSizeBytes = 0;
	pldSizeFloats = 0;
	break;
  }
  memcpy(&packet[2], pldData, pldSizeBytes);

  uint32_t crc = crcCalc(pldData, pldSizeFloats);

  memcpy(&packet[2 + pldSizeBytes], &crc, sizeof(uint32_t));
}

void readAdc(uint16_t* const buffer, uint16_t size)
{
  if(LL_ADC_IsEnabled(ADC1) == 0)
  {
    LL_ADC_Enable(ADC1);
  }

  for (int i = 0; i < size; i++)
  {
    LL_ADC_REG_StartConversionSWStart(ADC1);

    while( !LL_ADC_IsActiveFlag_EOCS(ADC1) );

    buffer[i] = LL_ADC_REG_ReadConversionData12(ADC1);
  }
}

void sendUart(uint8_t* buffer, uint16_t size)
{
  for (uint16_t i = 0; i < size; i++)
  {
    while (!LL_USART_IsActiveFlag_TXE(USART2));

    LL_USART_TransmitData8(USART2, buffer[i]);
  }

  while (!LL_USART_IsActiveFlag_TC(USART2));
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
