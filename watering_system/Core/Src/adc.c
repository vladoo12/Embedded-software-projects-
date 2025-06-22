/*
 * adc.c
 *
 *  Created on: May 27, 2025
 *      Author: vladimir
 */

#include"adc.h"
//private variables
uint16_t raw_values[2];
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

// Public variables
uint16_t lux = 0;
uint16_t moisture = 0;
uint8_t convCompleted = 0;


void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }


  // Channel 0 (LUX - PA0)
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;  // Increased from 15 cycles
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
      Error_Handler();
  }

  // Channel 1 (Moisture - PA1)
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;  // Explicitly set
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
      Error_Handler();
  }
  /*
   *  USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}



void My_ADC_Init(void)
{
    // Initialize ADC hardware
    MX_ADC1_Init();

    // Start first conversion
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)raw_values, 2);
}

void ADC_StartConversion(void)
{
    convCompleted = 0;
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*) raw_values, 2);
}


void ADC_GetValues(uint16_t* lux_val, uint16_t* moisture_val)
{
    while (!convCompleted); // Wait for conversion to complete

    *lux_val = raw_values[0];      // Channel 0 (PA0)
    *moisture_val = raw_values[1]; // Channel 1 (PA1)

    // Start next conversion
    ADC_StartConversion();
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    convCompleted = 1;
}





