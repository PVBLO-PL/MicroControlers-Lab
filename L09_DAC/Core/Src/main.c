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
#include "dac.h"
#include "dma.h"
#include "eth.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include "stdbool.h"
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
volatile uint32_t adcValue = 0;
volatile uint32_t adcVoltage = 0; // Variable to store the converted voltage in mV
volatile uint32_t time = 0;
volatile uint32_t amplitude = 1000; // Amplitude for the sine wave in mV
uint8_t plotMsg[32]; // Variable to store the DAC value
uint8_t rxByte; // Variable to store the received byte
uint8_t rxCommand[32]; // Buffer to store the received command
uint8_t responseMsg[64]; // Buffer to store messages for UART transmission
volatile uint8_t rxLength = 0; // Length of the received command
volatile bool commandReady = false; // Flag to indicate a complete command is ready
volatile bool rxDiscard = false; // Flag to indicate if the next bytes should be discarded
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
void DAC_SetVoltage(uint32_t voltage) {
  if (voltage > 3300) {
    voltage = 3300; // Limit to 3.3V
  }
  uint32_t dacValue = (voltage * 4095) / 3300; // Convert mV to DAC value
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dacValue);
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
}

void DAC_GenerateSine(uint32_t time, uint32_t Amplitude) {
  if (Amplitude > 1500) {
    Amplitude = 1500; // Limit amplitude to avoid exceeding DAC range
  }
  uint32_t voltage = (Amplitude * sinf(2 * M_PI * time / 1000)) + 1500; // Generate a sine wave voltage
  uint32_t dacValue = (voltage * 4095) / 3300; // Convert mV to DAC value
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dacValue);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM6) {
    // Timer 6 interrupt callback
    time += 5; // Increment time by 5 ms
    if (time >= 1000) {
      time = 0; // Reset time after 1 second
    }
    DAC_GenerateSine(time, amplitude); // Generate a sine wave with the specified amplitude
  }
}

void HandleCommand(uint8_t *command) {
  char *ptr = (char*)command;
  while (*ptr == ' ') {
    ptr++; // Pomiń ewentualne spacje na początku
  }

  if (strncmp(ptr, "set_amp", 7) == 0) {
    ptr += 7;
    while (*ptr == ' ' || *ptr == '=') {
      ptr++; // Pomiń spacje oraz znak '='
    }

    if (*ptr == '\0') {
      int len = snprintf((char*)responseMsg, sizeof(responseMsg), "Error: Missing amplitude value\r\n");
      HAL_UART_Transmit(&huart3, responseMsg, len, 100);
      return;
    }

    int newAmplitude = atoi(ptr);
    if (newAmplitude >= 0 && newAmplitude <= 1500) {
      amplitude = (uint32_t)newAmplitude;
      int len = snprintf((char*)responseMsg, sizeof(responseMsg), "Amplitude set to: %lu mV\r\n", amplitude);
      HAL_UART_Transmit(&huart3, responseMsg, len, 100);
    } else {
      int len = snprintf((char*)responseMsg, sizeof(responseMsg), "Error: Amplitude must be 0..1500 mV\r\n");
      HAL_UART_Transmit(&huart3, responseMsg, len, 100);
    }
  } else {
    int len = snprintf((char*)responseMsg, sizeof(responseMsg), "Error: Unknown command\r\n");
    HAL_UART_Transmit(&huart3, responseMsg, len, 100);
  }
}

void ReceiveByByte(UART_HandleTypeDef *huart) {
  char receivedChar = (char)rxByte; // Convert the received byte to a character
  if(receivedChar == ';') // Check for command termination
  {
    rxCommand[rxLength] = '\0'; // Add \0 so it will be compatible with fucntions that expect C string
    commandReady = true; // Set the flag to indicate a complete command is ready
    rxLength = 0; // Reset the length for the next command
    rxDiscard = false; // Reset discard flag
  }
  else if(receivedChar == '\r' || receivedChar == '\n') // Ignore carriage return and newline characters
  {

  }
  else if(rxLength < sizeof(rxCommand) - 1) // Check if there's space in the buffer
  {
    if(!rxDiscard)
    {
      rxCommand[rxLength] = receivedChar; // Store the character
      rxLength++; // Increment index of array iwth our command
    }
  }
  else
  {
    rxDiscard = true; // If buffer is full, discard next bytes
  }
  HAL_UART_Receive_IT(&huart3, &rxByte, 1); // Continue receiving the next byte
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if(huart->Instance == USART3)
  {
    ReceiveByByte(huart); // Call the function to handle the received byte
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
  MX_ETH_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  //DAC_SetVoltage(1650); // Set DAC output to 1.65V (midpoint of 0-3.3V range)
  HAL_TIM_Base_Start_IT(&htim6); // Start the timer for DAC triggering
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1); // Start the DAC
  HAL_UART_Receive_IT(&huart3, &rxByte, 1); // Start receiving the first byte via UART
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
      adcValue = HAL_ADC_GetValue(&hadc1);
      // Convert ADC value to voltage in mV
      adcVoltage = (adcValue * 3300) / 4095;
      // Here you can use the voltage value as needed
      int len = snprintf((char*)plotMsg, sizeof(plotMsg), ">ADC_Voltage:%lu mV\r\n", adcVoltage);
      HAL_UART_Transmit(&huart3, plotMsg, len, HAL_MAX_DELAY); //
    }
    if (commandReady) {
      commandReady = false; // Reset the flag
      HandleCommand(rxCommand); // Process the received command
    }
    HAL_Delay(5); // Delay for 5 ms before the next reading
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
