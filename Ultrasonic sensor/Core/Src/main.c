
#include "main.h"
#include "stm32f4xx.h"

#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#define usTIM	TIM4
#define MIN_DISTANCE 10.0f  // Minimum distance in cm to trigger the alarm
#define ALARM_TIME_MS 5000

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart2;

volatile uint8_t alarmActive = 0;
volatile uint32_t lastAlarmTime = 0;
const float speedOfSound = 0.0343;
float distance = 0;
uint32_t numTicks = 0;

char uartBuf[100];

float currentDistance = 0;



void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void TIMER3_Setup(void);
void TIMER4_Setup(void);
void TIMER2_Setup(void);
void ControlAlarm(bool enable);
float MeasureDistance(void);
static void microDelay(uint32_t delay);
void TurnBuzzerOn(void);
void TurnBuzzerOff(void);
void TurnLedOn(void);
void TurnLedOff(void);

int main(void)
{
	  HAL_Init();
	  SystemClock_Config();
	  MX_GPIO_Init();
	  MX_USART2_UART_Init();

	  TIMER3_Setup();
	  TIMER4_Setup();

	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	  TurnBuzzerOff();
	  TurnLedOff();

	  while (1) {
		  currentDistance = MeasureDistance();
		  	//used for debbuging and to check if measurement is correct !
		    sprintf(uartBuf, "Distance (cm) = %.1f\r\n", currentDistance);
		    HAL_UART_Transmit(&huart2, (uint8_t *)uartBuf, strlen(uartBuf), 100);

	      // Check minimum distance
	      if(currentDistance < MIN_DISTANCE) {
	          alarmActive = true;
	          lastAlarmTime = HAL_GetTick();
	      }

	      // Control alarm
	      if (alarmActive) {
	          ControlAlarm(true);

	          if (HAL_GetTick() - lastAlarmTime >= ALARM_TIME_MS) {
	              alarmActive = false;
	              ControlAlarm(false);
	          }
	      }

	      HAL_Delay(100);
	  }
	}


float MeasureDistance(void) {
    uint32_t numTicks = 0;
    float distance = 0.0f;

    // 1. Initially reset the TRIG pin
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
    microDelay(3);  // Short delay for stabilization

    // 2. Generate a 10 µs TRIG pulse
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
    microDelay(10);
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

    // 3. Wait for the rising edge on the ECHO pin
    while (HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_RESET);

    // 4. Measure the duration of the ECHO pulse (in µs)
    while (HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_SET) {
        numTicks++;
        microDelay(1);  // Each tick ≈ 1 µs
    }

    // 5. Calculate the distance in cm
    distance = (float)numTicks * speedOfSound;

    return distance;
}
/**
  * @brief System Clock Configuration
  * @retval None
  */

void ControlAlarm(bool enable) {
    if (enable) {
    	TurnLedOn();
		TurnBuzzerOn();

    } else {
    	TurnLedOff();
        TurnBuzzerOff();

    }
}
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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



/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
void TIMER4_Setup(void)
{

	__HAL_RCC_TIM4_CLK_ENABLE();

	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 83;	//     84MHz/(83+1) = 1KHz
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 999;	// (999+1)/1000  = 1s

	if(HAL_TIM_Base_Init(&htim4) != HAL_OK) {
	     Error_Handler();
	 }

	TIM_OC_InitTypeDef	sConfigOC = {0};

	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
	sConfigOC.Pulse = 0;
	HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);

	//oc3 and oc4 set as pwm
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 499;		//half duty cycle
    HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3);
    sConfigOC.Pulse = 249;		//10% duty cycle
    HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4);
    HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2);

    HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

}
/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{


  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }


}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin|TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin TRIG_Pin */
  GPIO_InitStruct.Pin = LED_Pin|TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ECHO_Pin */
  GPIO_InitStruct.Pin = ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);



  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;//yellow, green, blue, purple
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;

  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void TIMER3_Setup(void)
{
	  __HAL_RCC_TIM3_CLK_ENABLE();
	  htim3.Instance = TIM3;
	  htim3.Init.Prescaler = 84 - 1;  // 1 MHz clock (1 µs/tick)
	  htim3.Init.Period = 0xFFFFFFFF;     // Max period
	  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	if(HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		    Error_Handler();
		}
	  HAL_TIM_Base_Start(&htim3);


}

void TurnLedOn(void)
{
    TIM4->CCR3 = 1000;
    TIM4->CCR4 = 1000;
}

void TurnLedOff(void)
{

      TIM4->CCR3 = 0;
      TIM4->CCR4 = 0;

}


void TurnBuzzerOn(void)
{
    TIM4->CCR2 = 1000;
}

void TurnBuzzerOff(void)
{
    TIM4->CCR2 = 0;
}



static void microDelay (uint32_t delay){

	  __HAL_TIM_SET_COUNTER(&htim3, 0);
	  while (__HAL_TIM_GET_COUNTER(&htim3) < delay);

}



/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  
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

}
#endif /* USE_FULL_ASSERT */
