
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_hal.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
osTimerId LedDimmerTimerHandle;
osStaticTimerDef_t LedDimmerTimerControlBlock;
osTimerId ValveSwitchDelayTimerHandle;
osStaticTimerDef_t ValveSwitchDelayTimerControlBlock;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static IndicatorstateTypeDef led_dimmer_sp[NB_LED];

uint8_t waitQueue[NB_SSR];
SsrstateTypeDef channelStatus[NB_SSR];
uint8_t currentlyPreHeating = EMPTY_SLOT;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void const * argument);
void LedDimmerTimerCallback(void const * argument);
void ValveSwitchDelayTimerCallback(void const * argument);                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void timeStepElapsed();
void stop(uint8_t channelNb);
void addToEnableRequests(uint8_t channelNb);
void InitWaitQueue();


/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  InitWaitQueue();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of LedDimmerTimer */
  osTimerStaticDef(LedDimmerTimer, LedDimmerTimerCallback, &LedDimmerTimerControlBlock);
  LedDimmerTimerHandle = osTimerCreate(osTimer(LedDimmerTimer), osTimerPeriodic, NULL);

  /* definition and creation of ValveSwitchDelayTimer */
  osTimerStaticDef(ValveSwitchDelayTimer, ValveSwitchDelayTimerCallback, &ValveSwitchDelayTimerControlBlock);
  ValveSwitchDelayTimerHandle = osTimerCreate(osTimer(ValveSwitchDelayTimer), osTimerOnce, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128, defaultTaskBuffer, &defaultTaskControlBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 3, 0);
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SSR0_Pin|SSR1_Pin|SSR2_Pin|SSR3_Pin 
                          |SSR4_Pin|SSR5_Pin|SSR6_Pin|SSR7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SSR8_GPIO_Port, SSR8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED0_Pin|LED4_Pin|LED5_Pin|LED6_Pin 
                          |LED7_Pin|LED8_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : BTN_Pin */
  GPIO_InitStruct.Pin = BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_STATUS_Pin */
  GPIO_InitStruct.Pin = LED_STATUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_STATUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SSR0_Pin SSR1_Pin SSR2_Pin SSR3_Pin 
                           SSR4_Pin SSR5_Pin SSR6_Pin SSR7_Pin 
                           LED1_Pin LED2_Pin LED3_Pin */
  GPIO_InitStruct.Pin = SSR0_Pin|SSR1_Pin|SSR2_Pin|SSR3_Pin 
                          |SSR4_Pin|SSR5_Pin|SSR6_Pin|SSR7_Pin 
                          |LED1_Pin|LED2_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SSR8_Pin LED0_Pin LED4_Pin LED5_Pin 
                           LED6_Pin LED7_Pin LED8_Pin */
  GPIO_InitStruct.Pin = SSR8_Pin|LED0_Pin|LED4_Pin|LED5_Pin 
                          |LED6_Pin|LED7_Pin|LED8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void SetSwitch(uint8_t switchNb, OnoffstateTypeDef state)
{
	GPIO_TypeDef* gpioPort;
	uint16_t gpioPin;
	switch(switchNb)
	{
	case 0:
		gpioPort = GPIOA;
		gpioPin = SSR0_Pin;
		break;
	case 1:
		gpioPort = GPIOA;
		gpioPin = SSR1_Pin;
		break;
	case 2:
		gpioPort = GPIOA;
		gpioPin = SSR2_Pin;
		break;
	case 3:
		gpioPort = GPIOA;
		gpioPin = SSR3_Pin;
		break;
	case 4:
		gpioPort = GPIOA;
		gpioPin = SSR4_Pin;
		break;
	case 5:
		gpioPort = GPIOA;
		gpioPin = SSR5_Pin;
		break;
	case 6:
		gpioPort = GPIOA;
		gpioPin = SSR6_Pin;
		break;
	case 7:
		gpioPort = GPIOA;
		gpioPin = SSR7_Pin;
		break;
	default:
		gpioPort = GPIOB;
		gpioPin = SSR8_Pin;
		break;
	}
	if (state == ON)
	{
		HAL_GPIO_WritePin(gpioPort, gpioPin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(gpioPort, gpioPin, GPIO_PIN_RESET);
	}
}

void SetLed(uint8_t ledNb, OnoffstateTypeDef state)
{
	GPIO_TypeDef* gpioPort;
	uint16_t gpioPin;
	switch(ledNb)
	{
	case 0:
		gpioPort = GPIOB;
		gpioPin = LED0_Pin;
		break;
	case 1:
		gpioPort = GPIOA;
		gpioPin = LED1_Pin;
		break;
	case 2:
		gpioPort = GPIOA;
		gpioPin = LED2_Pin;
		break;
	case 3:
		gpioPort = GPIOA;
		gpioPin = LED3_Pin;
		break;
	case 4:
		gpioPort = GPIOB;
		gpioPin = LED4_Pin;
		break;
	case 5:
		gpioPort = GPIOB;
		gpioPin = LED5_Pin;
		break;
	case 6:
		gpioPort = GPIOB;
		gpioPin = LED6_Pin;
		break;
	case 7:
		gpioPort = GPIOB;
		gpioPin = LED7_Pin;
		break;
	case 8:
		gpioPort = GPIOB;
		gpioPin = LED8_Pin;
		break;
	default:
		gpioPort = LED_STATUS_GPIO_Port;
		gpioPin = LED_STATUS_Pin;
		break;
	}
	if (state == ON)
	{
		HAL_GPIO_WritePin(gpioPort, gpioPin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(gpioPort, gpioPin, GPIO_PIN_SET);
	}
}


void InitWaitQueue()
{
	for (uint8_t i = 0; i<NB_SSR;i++)
	{
		waitQueue[i] = EMPTY_SLOT;
	}
	currentlyPreHeating = EMPTY_SLOT;
}

void addToEnableRequests(uint8_t channelNb)
{
	if (currentlyPreHeating == EMPTY_SLOT)
	{
		currentlyPreHeating = channelNb;
		channelStatus[channelNb] = SSR_ON;
	}
	else
	{
		for (uint8_t i = 0; i<NB_SSR;i++)
		{
			if (waitQueue[i] == channelNb)
				break;

			if (waitQueue[i] == EMPTY_SLOT)
			{
				waitQueue[i] = channelNb;
				channelStatus[channelNb] = SSR_PENDING_ON;
				if (i == 0)
				{
					//timer started. first element inserted
					osTimerStart(ValveSwitchDelayTimerHandle,OVERCONSUMPTION_MS);
				}
				break;
			}
		}
	}

}

void timeStepElapsed()
{
	currentlyPreHeating = waitQueue[0];
	channelStatus[currentlyPreHeating] = SSR_ON;

	for (uint8_t j = 0; j<NB_SSR-1;j++)
	{
		waitQueue[j] = waitQueue[j+1];
	}
	waitQueue[NB_SSR-1] = EMPTY_SLOT;

	if (waitQueue[0] != EMPTY_SLOT)
	{
		// timer started. elements still in the queue
		osTimerStart(ValveSwitchDelayTimerHandle,OVERCONSUMPTION_MS);
	}

}

void stop(uint8_t channelNb)
{
	for (uint8_t i = 0; i<NB_SSR;i++)
	{
		if (waitQueue[i] == channelNb)
			waitQueue[i] = EMPTY_SLOT;

		if ((waitQueue[i] == EMPTY_SLOT) && (i < (NB_SSR-1) ))
		{
			waitQueue[i] = waitQueue[i + 1];
			waitQueue[i + 1] = EMPTY_SLOT;
		}
	}
	channelStatus[channelNb] = SSR_OFF;
	if (currentlyPreHeating == channelNb)
	{
		timeStepElapsed();
	}
}



/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 5 */
  osTimerStart(LedDimmerTimerHandle,BLINK_MS);
  /* Infinite loop */

  for(;;)
  {
    osDelay(1);
    for (uint8_t i = 0; i<NB_SSR;i++)
    {
    	switch(channelStatus[i])
    	{
    	case SSR_OFF:
    		SetSwitch(i,OFF);
    		led_dimmer_sp[i] = IND_OFF;
    		break;
    	case SSR_ON:
    		SetSwitch(i,ON);
    		led_dimmer_sp[i] = IND_ON;
    		break;
    	case SSR_PENDING_ON:
    		SetSwitch(i,OFF);
    		led_dimmer_sp[i] = IND_BLINK;
    		break;
    	default:
    		break;
    	}
    }
  }
  /* USER CODE END 5 */ 
}

/* LedDimmerTimerCallback function */
void LedDimmerTimerCallback(void const * argument)
{
  /* USER CODE BEGIN LedDimmerTimerCallback */
	static OnoffstateTypeDef oldLedState[NB_LED];

	for (uint8_t i = 0; i<NB_LED; i++)
	{
		switch(led_dimmer_sp[i])
		{
		case IND_OFF:
			SetLed(i,OFF);
			oldLedState[i] = OFF;
			break;
		case IND_ON:
			SetLed(i,ON);
			oldLedState[i] = ON;
			break;
		case IND_BLINK:
			if (oldLedState[i] == ON)
			{
				SetLed(i,OFF);
				oldLedState[i] = OFF;
			}
			else
			{
				SetLed(i,ON);
				oldLedState[i] = ON;
			}
			break;
		}
	}
  
  /* USER CODE END LedDimmerTimerCallback */
}

/* ValveSwitchDelayTimerCallback function */
void ValveSwitchDelayTimerCallback(void const * argument)
{
  /* USER CODE BEGIN ValveSwitchDelayTimerCallback */
	timeStepElapsed();
  /* USER CODE END ValveSwitchDelayTimerCallback */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
