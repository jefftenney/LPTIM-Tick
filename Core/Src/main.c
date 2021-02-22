/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "testTickTiming.h"
#include "ulp.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NOTIFICATION_FLAG_B1_PIN      (1UL << 0)
#define NOTIFICATION_FLAG_LED_BLIP    (1UL << 1)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart2;

osThreadId mainTaskHandle;
osTimerId ledTimerHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
void mainOsTask(void const * argument);
void ledTimerCallback(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define MAX_DEMO_STATE 1
static void vSetDemoState( int state )
{
   uint32_t ledIntervalMs = 0;
   if (state == 0)
   {
      //      Demonstrate minimum energy consumption.  With configUSE_TICKLESS_IDLE == 2 (lptimTick.c), we
      // draw only 2uA, with RTC, and with FreeRTOS timers/timeouts/delays active.
      //
      ledIntervalMs = 5000UL;
      vTttSetEvalInterval( (TickType_t)configTICK_RATE_HZ );
   }
   else if (state == 1)
   {
      //      Demonstrate energy consumption waking every 10ms, and test the tick timing.
      //
      ledIntervalMs = 2000UL;
      vTttSetEvalInterval( pdMS_TO_TICKS(10) );
   }
#if 0
   else if (state == 2)
   {
      // In state 2 we can add an actor to stress the tick timing
   }
#endif
   else
   {
      configASSERT(0);
   }

   if (ledIntervalMs != 0)
   {
      osTimerStart(ledTimerHandle, ledIntervalMs);
      xTaskNotify(mainTaskHandle, NOTIFICATION_FLAG_LED_BLIP, eSetBits);
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

  // Integrate lptimTick.c -- Start of Block
  //
  //      Code in lptimTick.c expects LSE or LSI to be configured and started by the application code.  This
  // RCC code starts the LSE, but it is commented out because this demo application uses the RTC, so CubeMX
  // already generates the code we need to start LSE.  See SystemClock_Config().
  //
  //  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  //  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE;
  //  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  //  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  //  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  //  {
  //    Error_Handler();
  //  }
  //
  // Integrate lptimTick.c -- End of Block

  //      Initialize Ultra-Low Power support (ULP).
  //
  vUlpInit();

  #if (configUSE_TICKLESS_IDLE != 2)
  {
    //      The tick-timing tests make more accurate evaluations of the tick timing when FreeRTOS has the best
    // possible value in configCPU_CLOCK_HZ.  Since CubeMX defines that symbol as SystemCoreClock, we update
    // SystemCoreClock here.  This code helped us "prove" that the MSI PLL behaves like a real PLL (albeit a
    // very jittery one).  There's "no" tick drift as measured by the RTC when tickless idle is disabled and
    // the MSI is in PLL mode driving the core clock.
    //
    //      We don't care if the HAL reverts this change at some point later during the application execution.
    // We only want this updated value to endure long enough for FreeRTOS startup code to use it to calculate
    // the tick timing.
    //
    if (RCC->CR & RCC_CR_MSIPLLEN)  // App Specific.  Assumes MSIPLLEN being set means MSI is the core clock.
    {
      int pllModeMultiplier = ( SystemCoreClock + (LSE_VALUE/2) ) / LSE_VALUE;
      SystemCoreClock = LSE_VALUE * pllModeMultiplier;
    }
  }
  #endif

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  //      Instead of notifying the ULP API that USART2 is now active, just disable USART2.  The application
  // knows when USART2 is supposed to be active.
  //
  // vUlpOnPeripheralsActive(ulpPERIPHERAL_USART2);
  //
  HAL_UART_DeInit(&huart2);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of ledTimer */
  osTimerDef(ledTimer, ledTimerCallback);
  ledTimerHandle = osTimerCreate(osTimer(ledTimer), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of mainTask */
  osThreadDef(mainTask, mainOsTask, osPriorityNormal, 0, 128);
  mainTaskHandle = osThreadCreate(osThread(mainTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */

  osThreadDef(tickTest, vTttOsTask, osPriorityAboveNormal, 0, configMINIMAL_STACK_SIZE);
  osThreadCreate(osThread(tickTest), &hrtc);

  /* USER CODE END RTOS_THREADS */

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_HSI;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 3;
  hrtc.Init.SynchPrediv = 8191;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
   if (GPIO_Pin == B1_Pin)
   {
      //      Don't attempt to notify a task that isn't yet created.  Drop the button press instead.
      //
      if ( mainTaskHandle != NULL )
      {
         BaseType_t xWasHigherPriorityTaskWoken = pdFALSE;
         xTaskNotifyFromISR( mainTaskHandle, NOTIFICATION_FLAG_B1_PIN, eSetBits, &xWasHigherPriorityTaskWoken);
         portYIELD_FROM_ISR(xWasHigherPriorityTaskWoken);
      }
   }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_mainOsTask */
/**
  * @brief  Function implementing the mainTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_mainOsTask */
void mainOsTask(void const * argument)
{
  /* USER CODE BEGIN 5 */

   int xDemoState = 0;
   vSetDemoState(xDemoState);

  /* Infinite loop */
   uint32_t notificationFlags;
   for(;;)
   {
      //      Wait forever for any notification.  In the future we'll have different test modes, driven by
      // the user button B1.  One will allow the user to measure our ultra-low power consumption, which is
      // only 2uA, with RTC, on a Nucleo L476 in STOP 2.  Another will stress and validate tick timing.
      //
      xTaskNotifyWait(0, UINT32_MAX, &notificationFlags, portMAX_DELAY);

      if (notificationFlags & NOTIFICATION_FLAG_B1_PIN)
      {
         if (++xDemoState > MAX_DEMO_STATE )
         {
            xDemoState = 0;
         }

         vSetDemoState(xDemoState);
      }

      if (notificationFlags & NOTIFICATION_FLAG_LED_BLIP)
      {
         //      Blip the LED for 100ms.
         //
         HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
         osDelay(100);
         HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
      }
  }
  /* USER CODE END 5 */
}

/* ledTimerCallback function */
void ledTimerCallback(void const * argument)
{
  /* USER CODE BEGIN ledTimerCallback */
   UNUSED(argument);

   if ( mainTaskHandle != NULL )
   {
      BaseType_t xWasHigherPriorityTaskWoken = pdFALSE;
      xTaskNotifyFromISR( mainTaskHandle, NOTIFICATION_FLAG_LED_BLIP, eSetBits, &xWasHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xWasHigherPriorityTaskWoken);
   }

  /* USER CODE END ledTimerCallback */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM17) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
