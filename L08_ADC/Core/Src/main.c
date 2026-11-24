/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "eth.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
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
volatile uint32_t ADCValue3 = 0; // Variable to store the ADC Channel 3 value
volatile uint32_t ADCConversion3 = 0;  // Variable to store the ADC Channel 3 conversion result
volatile uint32_t ADCValue4 = 0; // Variable to store the ADC Channel 4 value
volatile uint32_t ADCConversion4 = 0;  // Variable to store the ADC Channel 4 conversion result
uint8_t msgPlot[39];
uint8_t msgSerial[182];
volatile uint8_t uartSendFlag = 0; // Flag for UART transmission
volatile uint8_t currentRank = 0; // Variable to keep track of the current ADC rank
volatile uint8_t adcNextStep = 0; // Flag to indicate the next ADC step
float R4 = 5100.0f;
float V_ntc = 0.0f;
float R_ntc = 0.0f;
float T_celsius = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
uint32_t ConvertADCToMillivolts(uint32_t adcValue) {
    return (adcValue * 3300) / 4095; // Convert ADC value to millivolts (assuming 12-bit ADC and 3.3V reference)
}

void HandleLEDStrip(uint32_t value) {
    if(value < 500) {
      HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);
    }
    else if(value < 1000) {
      HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);
    }
    else if(value < 1500) {
      HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);
    }
    else {
      HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        uartSendFlag = 1; // Set the flag to indicate it's time to send data over UART
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    if (currentRank == 0) {
        ADCValue3 = HAL_ADC_GetValue(hadc);
        ADCConversion3 = ConvertADCToMillivolts(ADCValue3);
        HandleLEDStrip(ADCConversion3);
        currentRank = 1; // Move to the next rank
    } else if (currentRank == 1) {
        ADCValue4 = HAL_ADC_GetValue(hadc);
        ADCConversion4 = ConvertADCToMillivolts(ADCValue4);
        currentRank = 0; // Reset to the first rank
    }
    adcNextStep = 1; // Indicate that the next ADC step can be taken
  }
}
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

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ETH_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim6); // Start the timer with interrupt
  HAL_ADC_Start_IT(&hadc1); // Start ADC in interrupt mode
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (adcNextStep) {
      adcNextStep = 0;
      if (currentRank == 1) {
        SET_BIT(hadc1.Instance->CR2, ADC_CR2_SWSTART); // Start Rank 2
      } else {
        HAL_ADC_Start_IT(&hadc1); // Start ADC conversion for Rank 1
      }
    }

    int len0 = sprintf((char*)msgPlot, ">napiecie_mV3:%lu\r\n>napiecie_mV4:%lu\r\n", ADCConversion3, ADCConversion4);
    HAL_UART_Transmit(&huart3, msgPlot, len0, 10);
    if (uartSendFlag) {
        uartSendFlag = 0; // Reset the flag
        
        V_ntc = (float)ADCConversion4;
        if (V_ntc >= 3299.0f) {
            V_ntc = 3299.0f; // Prevent division by zero or negative resistance
        }
        R_ntc = (R4 * V_ntc) / (3300.0f - V_ntc); // Calculate NTC resistance
        T_celsius = (1.0f / ((log(R_ntc / 10000.0f) / 3950.0f) + (1.0f / 298.15f))) - 273.15f; // Calculate temperature in Celsius

        int len2 = sprintf((char*)msgSerial, "{\"channel\":\"adc_in%d\", \"voltage\":%lu.%03lu}\r\n"
                                             "{\"channel\":\"adc_in%d\", \"voltage\":%.3f, \"RNTC\":%.3f, \"T\":%.3f}\r\n",
                                              3, ADCConversion3 / 1000, ADCConversion3 % 1000,
                                              10, V_ntc / 1000.0f, R_ntc, T_celsius);
        HAL_UART_Transmit_IT(&huart3, msgSerial, len2);
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
