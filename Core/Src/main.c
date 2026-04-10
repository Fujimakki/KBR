/* USER CODE BEGIN Header */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "arm_math_types.h"
#include "crc.h"
#include "dma.h"
#include "stm32f4xx_ll_adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint32_t DMA_ADC_buffer[UART_ADC_PAYLOAD_U16_SIZE];
volatile uint32_t* DMA_ADC_bufferHalf;

static uint8_t ADC_txPacket[UART_ADC_PACKET_BYTE_SIZE] = { UART_PREHEADER, UART_HEADER_RAW };
static uint8_t FFT_txPacket[UART_FFT_PACKET_BYTE_SIZE] = { UART_PREHEADER, UART_HEADER_FFT };

static const float32_t QUANT_STEP = 3.3f / ((1 << 12) - 1);
static float32_t ADC_voltData[UART_ADC_PAYLOAD_U16_SIZE];

#ifdef DBG
uint32_t delta_time[5];
#endif // DBG

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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

#ifdef DBG
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif // DBG

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_CRC_Init();
  MX_TIM1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_0, (uint32_t)DMA_ADC_buffer);
  LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_0, UART_ADC_PAYLOAD_U16_SIZE);

  DMA_startStream(DMA2, LL_DMA_STREAM_0, 0);

  LL_ADC_Enable(ADC1);
  LL_ADC_Enable(ADC2);
  while(!LL_ADC_IsEnabled(ADC1) && !LL_ADC_IsEnabled(ADC2)) {}
  LL_mDelay(1);

  if(LL_ADC_REG_IsTriggerSourceSWStart(ADC1))
  {
    LL_ADC_REG_StartConversionSWStart(ADC1);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1) {
    static volatile bool readyFft = false;

    if(DMA_ADC_bufferHalf)
    {
      memcpy(ADC_txPacket + UART_HEADER_BYTE_SIZE, (const void *)DMA_ADC_bufferHalf, UART_ADC_PAYLOAD_BYTE_SIZE);
      UART_send(ADC_txPacket, UART_ADC_PACKET_BYTE_SIZE);

      if(!readyFft)
      {
        const size_t ADC_payloadHalfSize = UART_ADC_PAYLOAD_U16_SIZE >> 1;
        for(uint32_t i = 0; i < ADC_payloadHalfSize; i++)
        {
          ADC_voltData[i] = (*DMA_ADC_bufferHalf & 0xFFFF) * QUANT_STEP;
          ADC_voltData[ADC_payloadHalfSize + i] = (*DMA_ADC_bufferHalf >> 16) * QUANT_STEP;
          DMA_ADC_bufferHalf ++;
        }
        readyFft = true;
      }

      DMA_ADC_bufferHalf = NULL;
    }

    if(readyFft)
    {
      fftMagCalc(&S, ADC_voltData, (float32_t*)(FFT_txPacket + UART_HEADER_BYTE_SIZE));
      fftMagCalc(&S, ADC_voltData + FFT_SIZE, (float32_t*)(FFT_txPacket + UART_HEADER_BYTE_SIZE + (UART_FFT_PAYLOAD_BYTE_SIZE >> 1)));

      UART_send(FFT_txPacket, UART_FFT_PACKET_BYTE_SIZE);

      readyFft = false;
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  while (1) {
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
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
