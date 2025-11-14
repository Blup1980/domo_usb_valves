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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "queue.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
typedef StaticTimer_t osStaticTimerDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 64 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for USBParserTask */
osThreadId_t USBParserTaskHandle;
uint32_t USBParserTaskBuffer[ 128 ];
osStaticThreadDef_t USBParserTaskControlBlock;
const osThreadAttr_t USBParserTask_attributes = {
  .name = "USBParserTask",
  .cb_mem = &USBParserTaskControlBlock,
  .cb_size = sizeof(USBParserTaskControlBlock),
  .stack_mem = &USBParserTaskBuffer[0],
  .stack_size = sizeof(USBParserTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for USBFromPCQueue */
osMessageQueueId_t USBFromPCQueueHandle;
uint8_t USBFromPCQueueBuffer[ 10 * sizeof( usbCommand_t ) ];
osStaticMessageQDef_t USBFromPCQueueControlBlock;
const osMessageQueueAttr_t USBFromPCQueue_attributes = {
  .name = "USBFromPCQueue",
  .cb_mem = &USBFromPCQueueControlBlock,
  .cb_size = sizeof(USBFromPCQueueControlBlock),
  .mq_mem = &USBFromPCQueueBuffer,
  .mq_size = sizeof(USBFromPCQueueBuffer)
};
/* Definitions for LedDimmerTimer */
osTimerId_t LedDimmerTimerHandle;
osStaticTimerDef_t LedDimmerTimerControlBlock;
const osTimerAttr_t LedDimmerTimer_attributes = {
  .name = "LedDimmerTimer",
  .cb_mem = &LedDimmerTimerControlBlock,
  .cb_size = sizeof(LedDimmerTimerControlBlock),
};
/* Definitions for ValveSwitchDelayTimer */
osTimerId_t ValveSwitchDelayTimerHandle;
osStaticTimerDef_t ValveSwitchDelayTimerControlBlock;
const osTimerAttr_t ValveSwitchDelayTimer_attributes = {
  .name = "ValveSwitchDelayTimer",
  .cb_mem = &ValveSwitchDelayTimerControlBlock,
  .cb_size = sizeof(ValveSwitchDelayTimerControlBlock),
};
/* USER CODE BEGIN PV */

static IndicatorstateTypeDef led_dimmer_sp[NB_LED];

uint8_t waitQueue[NB_SSR];
SsrstateTypeDef channelStatus[NB_SSR];
uint8_t currentlyPreHeating = EMPTY_SLOT;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void *argument);
void StartUSBParserTask(void *argument);
void LedDimmerTimerCallback(void *argument);
void ValveSwitchDelayTimerCallback(void *argument);

/* USER CODE BEGIN PFP */

void timeStepElapsed ();
void stop (uint8_t channelNb);
void addToEnableRequests (uint8_t channelNb);
void InitWaitQueue ();

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

  /* MCU Configuration--------------------------------------------------------*/

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

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of LedDimmerTimer */
  LedDimmerTimerHandle = osTimerNew(LedDimmerTimerCallback, osTimerPeriodic, NULL, &LedDimmerTimer_attributes);

  /* creation of ValveSwitchDelayTimer */
  ValveSwitchDelayTimerHandle = osTimerNew(ValveSwitchDelayTimerCallback, osTimerOnce, NULL, &ValveSwitchDelayTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of USBFromPCQueue */
  USBFromPCQueueHandle = osMessageQueueNew (10, sizeof(usbCommand_t), &USBFromPCQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of USBParserTask */
  USBParserTaskHandle = osThreadNew(StartUSBParserTask, NULL, &USBParserTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SSR0_Pin|SSR1_Pin|SSR2_Pin|SSR3_Pin
                          |SSR4_Pin|SSR5_Pin|SSR6_Pin|SSR7_Pin
                          |LED1_Pin|LED2_Pin|LED3_Pin|BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SSR8_Pin|LED0_Pin|LED4_Pin|LED5_Pin
                          |LED6_Pin|LED7_Pin|LED8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BTN_Pin */
  GPIO_InitStruct.Pin = BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_STATUS_Pin */
  GPIO_InitStruct.Pin = LED_STATUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_STATUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SSR0_Pin SSR1_Pin SSR2_Pin SSR3_Pin
                           SSR4_Pin SSR5_Pin SSR6_Pin SSR7_Pin
                           LED1_Pin LED2_Pin LED3_Pin BUZZER_Pin */
  GPIO_InitStruct.Pin = SSR0_Pin|SSR1_Pin|SSR2_Pin|SSR3_Pin
                          |SSR4_Pin|SSR5_Pin|SSR6_Pin|SSR7_Pin
                          |LED1_Pin|LED2_Pin|LED3_Pin|BUZZER_Pin;
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

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void SetSwitch(uint8_t switchNb, OnoffstateTypeDef state)
{
  GPIO_TypeDef *gpioPort;
  uint16_t gpioPin;
  switch (switchNb)
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
  GPIO_TypeDef *gpioPort;
  uint16_t gpioPin;
  switch (ledNb)
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
  for (uint8_t i = 0; i < NB_SSR; i++)
  {
    waitQueue[i] = EMPTY_SLOT;
  }
  currentlyPreHeating = EMPTY_SLOT;
}

void addToEnableRequests(uint8_t channelNb)
{
  if (channelStatus[channelNb] != SSR_OFF)
    return;
  if (currentlyPreHeating == EMPTY_SLOT)
  {
    currentlyPreHeating = channelNb;
    channelStatus[channelNb] = SSR_ON;
  }
  else
  {
    for (uint8_t i = 0; i < NB_SSR; i++)
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
          osTimerStart(ValveSwitchDelayTimerHandle,
          OVERCONSUMPTION_MS);
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

  for (uint8_t j = 0; j < NB_SSR - 1; j++)
  {
    waitQueue[j] = waitQueue[j + 1];
  }
  waitQueue[NB_SSR - 1] = EMPTY_SLOT;

  if (waitQueue[0] != EMPTY_SLOT)
  {
    // timer started. elements still in the queue
    osTimerStart(ValveSwitchDelayTimerHandle, OVERCONSUMPTION_MS);
  }

}

void stop(uint8_t channelNb)
{
  for (uint8_t i = 0; i < NB_SSR; i++)
  {
    if (waitQueue[i] == channelNb)
      waitQueue[i] = EMPTY_SLOT;

    if ((waitQueue[i] == EMPTY_SLOT) && (i < (NB_SSR - 1)))
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

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  osTimerStart(LedDimmerTimerHandle, BLINK_MS);
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
    for (uint8_t i = 0; i < NB_SSR; i++)
    {
      switch (channelStatus[i])
      {
        case SSR_OFF:
          SetSwitch(i, OFF);
          led_dimmer_sp[i] = IND_OFF;
          break;
        case SSR_ON:
          SetSwitch(i, ON);
          led_dimmer_sp[i] = IND_ON;
          break;
        case SSR_PENDING_ON:
          SetSwitch(i, OFF);
          led_dimmer_sp[i] = IND_BLINK;
          break;
        default:
          break;
      }
    }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartUSBParserTask */
/**
* @brief Function implementing the USBParserTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUSBParserTask */
void StartUSBParserTask(void *argument)
{
  /* USER CODE BEGIN StartUSBParserTask */
  OnoffstateTypeDef blinkState = OFF;
  usbCommand_t newCmd;
  uint8_t channelNb;
  uint8_t outBuffer[2];
  outBuffer[1] = STR_CR;
  /* Infinite loop */
  for (;;)
  {
    if (xQueueReceive(USBFromPCQueueHandle, &newCmd, portMAX_DELAY))
    {
      if (newCmd.byte[0] < '0' || newCmd.byte[0] > '8')
        continue;

      if (blinkState == ON){
        blinkState = OFF;
      } else {
        blinkState = ON;
      }
      SetLed(9, blinkState);
      channelNb = newCmd.byte[0] - 0x30;
      switch (newCmd.byte[1])
      {
        case '+':
          addToEnableRequests(channelNb);
          break;
        case '-':
          stop(channelNb);
          break;
        case '?':
          switch (channelStatus[channelNb])
          {
            case SSR_OFF:
              outBuffer[0] = '0';
              break;
            case SSR_ON:
              outBuffer[0] = '1';
              break;
            case SSR_PENDING_ON:
              outBuffer[0] = '1';
              break;
            default:
              outBuffer[0] = 'X';
              break;
          }
          CDC_Transmit_FS(outBuffer, 2);
          break;
        default:
          break;
      }
    }
    osDelay(1);
  }
  /* USER CODE END StartUSBParserTask */
}

/* LedDimmerTimerCallback function */
void LedDimmerTimerCallback(void *argument)
{
  /* USER CODE BEGIN LedDimmerTimerCallback */
  static OnoffstateTypeDef oldLedState[NB_LED];

  for (uint8_t i = 0; i < NB_LED; i++)
  {
    switch (led_dimmer_sp[i])
    {
      case IND_OFF:
        SetLed(i, OFF);
        oldLedState[i] = OFF;
        break;
      case IND_ON:
        SetLed(i, ON);
        oldLedState[i] = ON;
        break;
      case IND_BLINK:
        if (oldLedState[i] == ON)
        {
          SetLed(i, OFF);
          oldLedState[i] = OFF;
        }
        else
        {
          SetLed(i, ON);
          oldLedState[i] = ON;
        }
        break;
    }
  }

  /* USER CODE END LedDimmerTimerCallback */
}

/* ValveSwitchDelayTimerCallback function */
void ValveSwitchDelayTimerCallback(void *argument)
{
  /* USER CODE BEGIN ValveSwitchDelayTimerCallback */
  timeStepElapsed();
  /* USER CODE END ValveSwitchDelayTimerCallback */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
