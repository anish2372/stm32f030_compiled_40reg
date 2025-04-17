/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "i2c_slave.h"
#include "string.h"
#include <stdbool.h>
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
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
//uint16_t Register[8];  // Array to store ADC values adc0, adc1, ..., adc7
uint32_t ADC_Reads[8];
extern uint32_t I2C_REGISTERS[41];
//uint32_t I2C_REGISTERS_2[41] = {0};
uint32_t lastButtonState[8] = {0};  // To keep track of the last state of buttons

uint32_t sample_sum[8] = {0}; // Array to hold sums for averaging if needed
uint16_t sample_count[8] = {0}; // Array to hold counts for averaging if needed
float average_value[8] = {0.0}; // Array to hold average values if needed
bool sampling[8] = {false}; // Array to track sampling state for each channel
uint32_t max_value[8] = {0}; // Array to store the maximum value for each channel
uint16_t zero_count[8] = {0};
uint32_t currentButtonState[8];
// Define an array of pins to map to the corresponding button/LED outputs
const uint16_t ledPins[8] = { GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_10, GPIO_PIN_11, GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void Read_ADC_Channels(void);  // Function to read ADC values
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void ADC_Select_Channel(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;

    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

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
  MX_ADC_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  if(HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK)		//L we'll call this inside the main function
    {						//where it can check any incoming requests by master
  	  Error_Handler();
    }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  while (1)
//  {
////	  Read_ADC_Channels();  // Read all ADC channels (PA0 to PA7)
//
//	  // ADC Read Code
//	  HAL_ADC_Start_DMA(&hadc, ADC_Reads, 8);
//	  for (int i = 0; i < 8; i++) {
//	      I2C_REGISTERS_2[i] = ADC_Reads[i];
//	  }
//	  memcpy(&I2C_REGISTERS[0], I2C_REGISTERS_2, 8 * sizeof(uint32_t));
//	  I2C_REGISTERS[16] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15);
//	  I2C_REGISTERS[17] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14);
//	  I2C_REGISTERS[18] = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_1);
//	  I2C_REGISTERS[19] = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0);
//	  I2C_REGISTERS[20] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
//	  I2C_REGISTERS[21] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
//	  I2C_REGISTERS[22] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
//	  I2C_REGISTERS[23] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);
//	  HAL_Delay(100);  // Add a delay of 1 second between ADC readings
//    /* USER CODE END WHILE */
//
//    /* USER CODE BEGIN 3 */
//  }

  while (1)
  {
	  HAL_ADC_Start_DMA(&hadc, ADC_Reads, 8);
	      HAL_Delay(1); // Short delay to allow for sampling

	      for (int i = 0; i < 8; i++) {
	          uint32_t current_sample = ADC_Reads[i];
	          // Update maximum value if the current sample is greater than the previous maximum
			  if (current_sample > max_value[i]) {
				  max_value[i] = current_sample;
			  }
			  // Store the maximum value in I2C_REGISTERS_2[i + 32]
//			  I2C_REGISTERS[i + 32] = max_value[i]*0.001345219;
			  I2C_REGISTERS[i + 32] = (uint32_t)(max_value[i] * 1748.879);
	          // If current sample is 0, and not currently sampling, reset the register to 0
	          if (current_sample == 0) {
	        	  zero_count[i]++;
	        	  if (zero_count[i] > 5) { // Adjust threshold as needed for continuous zero detection
						  max_value[i] = 0; // Reset max value to zero if zeros are continuous
						  I2C_REGISTERS[i + 32] = 0; // Also reset register to zero
					  }
	              I2C_REGISTERS[i] = 0; // Set register to 0
//	              I2C_REGISTERS[i + 32] = 0; // Set maximum value register to 0
	              sampling[i] = false; // Reset the sampling state
	              sample_sum[i] = 0; // Reset sum for averaging
	              sample_count[i] = 0; // Reset count for averaging
//	              max_value[i] = 0; // Reset max value for the next signal cycle
	          } else {
	        	  zero_count[i] = 0; // Reset zero count when a non-zero sample is detected
	              // Check for the start of sampling
	              if (!sampling[i]) {
	                  sampling[i] = true; // Start sampling
	                  sample_sum[i] = 0; // Reset sum for new samples
	                  sample_count[i] = 0; // Reset count for new samples
//	                  max_value[i] = current_sample; // Initialize max with the first sample
	              }

	              if (sampling[i]) {
	                  // Accumulate samples for averaging
	                  sample_sum[i] += current_sample;
	                  sample_count[i]++;
	                  I2C_REGISTERS[i] = current_sample;

	                  // Check for falling edge zero crossing to stop sampling
	                  if (current_sample == 0) {
	                      sampling[i] = false;

	                      // Calculate average value of the half-wave
	                      if (sample_count[i] > 0) {
	                          average_value[i] = (float)sample_sum[i] / sample_count[i];

	                          // Optionally, you could store the average value in a different register
	                          // For example, if you want to store it in index 8 for each channel:
//	                           I2C_REGISTERS[i] = (uint32_t)average_value[i];
	                      }

	                      // Reset for the next half-sine wave cycle
	                      sample_sum[i] = 0;
	                      sample_count[i] = 0;
	                  }
	              }
	          }
	      }
	      // Update digital input feedback values to I2C registers 16-23
	          I2C_REGISTERS[16] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15);
	          I2C_REGISTERS[17] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14);
	          I2C_REGISTERS[18] = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_1);
	          I2C_REGISTERS[19] = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0);
	          I2C_REGISTERS[20] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
	          I2C_REGISTERS[21] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
	          I2C_REGISTERS[22] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
	          I2C_REGISTERS[23] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);
//      // ADC Read Code
//      HAL_ADC_Start_DMA(&hadc, ADC_Reads, 8);
//      for (int i = 0; i < 8; i++) {
//          I2C_REGISTERS[i] = ADC_Reads[i]; // Store ADC values in registers 0-7
//      }

      // Read the button states (registers 24-31)


      currentButtonState[0] = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_7);   // PF7
      currentButtonState[1] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);  // PA15
      currentButtonState[2] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);   // PB3
      currentButtonState[3] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);   // PB4
      currentButtonState[4] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);   // PB5
      currentButtonState[5] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);   // PB8
      currentButtonState[7] =  HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);   //
      currentButtonState[6] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);  // PC13

      // Update I2C_REGISTERS 24-31 with the current button states
      memcpy(&I2C_REGISTERS[24], currentButtonState, 8 * sizeof(uint32_t));

      // Toggle LEDs based on button states
      for (int i = 0; i < 8; i++) {
          // Check if the button was previously HIGH and is now LOW (pressed)
          if (currentButtonState[i] == GPIO_PIN_RESET && lastButtonState[i] == GPIO_PIN_SET) {
              // Toggle the corresponding LED in registers 8-15
              I2C_REGISTERS[8 + i] ^= 1; // Toggle the LED state (1 for ON, 0 for OFF)
              // Update the physical state of the LEDs
//              HAL_GPIO_WritePin(GPIOB, (GPIO_PIN_0 << i), I2C_REGISTERS[8 + i] ? GPIO_PIN_SET : GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, ledPins[i], I2C_REGISTERS[8 + i] ? GPIO_PIN_SET : GPIO_PIN_RESET);
          }
          lastButtonState[i] = currentButtonState[i]; // Update last button state
      }

      HAL_Delay(100);  // Delay of 100ms between reads
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 36;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_ENABLE;
  hi2c1.Init.OwnAddress2 = 0x45<<1;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_ENABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
//  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
//  {
//    Error_Handler();
//  }

  /** Configure Digital filter
  */
//  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
//  {
//    Error_Handler();
//  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  // Enable the required GPIO Ports Clock
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // Configure PA0-PA7 as Analog Inputs
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                        GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL; // No pull-up or pull-down
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Configure PB0, PB1, PB2, PB10, PB11, PB12, PB13, PB14 as Output Pins
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_10 |
                        GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Configure PC15, PC14, PF1, PF0, PB15, PA8, PA9, PA10 as Digital Input Pins with Pull-Down
  GPIO_InitStruct.Pin = GPIO_PIN_15 | GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN; // Set pull-down for input pins
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_0;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_15;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/**
  * @brief Reads ADC values from channels 0 to 7 and stores them in the Register array.
  * @retval None
  */
//void Read_ADC_Channels(void)
//{
//    for (int i = 0; i < 8; i++)
//    {
//        // Select the corresponding ADC channel
//        ADC_Select_Channel(ADC_CHANNEL_0 + i);  // ADC_CHANNEL_0 corresponds to PA0, ADC_CHANNEL_1 to PA1, etc.
//
//        // Start the ADC and poll for conversion completion
//        HAL_ADC_Start(&hadc);
//        HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
//
//        // Store the ADC result in the respective register
//        uint32_t adc_val = HAL_ADC_GetValue(&hadc);
//        Register[i] = adc_val;
//
//        // Stop the ADC
//        HAL_ADC_Stop(&hadc);
//    }
//}

/* USER CODE END 0 */

/* Infinite loop */
/* USER CODE BEGIN WHILE */



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

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
