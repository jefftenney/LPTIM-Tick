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

#include <stdio.h>
#include <string.h>
#include "testTickTiming.h"
#include "ulp.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NOTIFICATION_FLAG_B1_PIN    (1UL << 0)
#define NOTIFICATION_FLAG_LED_BLIP  (1UL << 1)
#define NOTIFICATION_FLAG_RESULTS   (1UL << 2)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
LPTIM_HandleTypeDef hlptim2;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart2;

osThreadId mainTaskHandle;
osTimerId ledTimerHandle;
osTimerId resultsTimerHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_LPTIM2_Init(void);
void mainOsTask(void const * argument);
void ledTimerCallback(void const * argument);
void resultsTimerCallback(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define DEMO_STATE_TEST_1  0
#define DEMO_STATE_TEST_2  1
#define DEMO_STATE_TEST_3  2
#define MAX_DEMO_STATE     DEMO_STATE_TEST_3
static void vSetDemoState( int state )
{
   //      Assume for now that we're activating a demo state that doesn't need LPTIM2, our stress-test actor.
   //
   if (LPTIM2->CR & LPTIM_CR_ENABLE)
   {
      HAL_LPTIM_Counter_Stop_IT(&hlptim2);
      vUlpOnPeripheralsInactive(ulpPERIPHERAL_LPTIM2);
   }

   uint32_t ledIntervalMs = 0;
   if (state == DEMO_STATE_TEST_1)
   {
      //      Demonstrate minimum energy consumption.  With configUSE_TICKLESS_IDLE == 2 (lptimTick.c), we
      // draw only 2uA, with RTC, and with FreeRTOS timers/timeouts/delays active.
      //
      ledIntervalMs = 5000UL;
      vTttSetEvalInterval( portMAX_DELAY );
      osTimerStop(resultsTimerHandle);
   }
   else if (state == DEMO_STATE_TEST_2)
   {
      //      Instruct the tick-timing test to sample the tick timing every 10 milliseconds.  Sampling faster
      // would require that we open the pass/fail criteria to accommodate a couple hundred microseconds of
      // jitter or tick "jump" since that criteria is expressed as a percentage of the sampling interval.
      // Sampling much slower might allow large jitter or "jump" that should be caught as an error, again
      // because jitter is measured as a percentage of the sampling interval.
      //
      #define TICK_TEST_SAMPLING_INTERVAL_MS 10

      //      Demonstrate energy consumption waking every 10ms, and test the tick timing.
      //
      ledIntervalMs = 2000UL;
      vTttSetEvalInterval( pdMS_TO_TICKS(TICK_TEST_SAMPLING_INTERVAL_MS) );
      osTimerStart(resultsTimerHandle, 1000UL);
   }
   else if (state == DEMO_STATE_TEST_3)
   {
      //      In state 2, add an actor to stress the tick timing.  Use LPTIM2 interrupts since that timer
      // keeps operating in STOP 1 mode, which allows us to keep demonstrating low-power operation.
      //
      //      Carefully select the interval of the nuisance interrupts to be slightly longer than the tick-
      // test sampling interval.  The sampling interval drives the "expected idle time" in the tickless logic,
      // so a nuisance interval slightly longer ensures lots of different interrupt timing, including some
      // tickless periods lasting the full expected idle time.  For example, with a 10ms sampling interval,
      // there are 327.68 LSE cycles between samplings, on average.  The code below would set up the nuisance
      // interrupt to be every 328 LSE cycles for that case.  (The value passed to the HAL is 327, but the
      // period ends up being 328 due to LPTIM reload behavior.)
      //
      //      Based on the discussion above, a good long soak test is strongly recommended here.
      //
      vUlpOnPeripheralsActive(ulpPERIPHERAL_LPTIM2);
      HAL_LPTIM_Counter_Start_IT(&hlptim2, (TICK_TEST_SAMPLING_INTERVAL_MS * LSE_VALUE) / 1000U);

      ledIntervalMs = 1000UL;
      vTttSetEvalInterval( pdMS_TO_TICKS(TICK_TEST_SAMPLING_INTERVAL_MS) );
      osTimerStart(resultsTimerHandle, 1000UL);
   }
   else
   {
      configASSERT(0);
   }

   if (ledIntervalMs != 0)
   {
      osTimerStart(ledTimerHandle, ledIntervalMs);
      xTaskNotify(mainTaskHandle, NOTIFICATION_FLAG_LED_BLIP, eSetBits);
   }

   char banner[100];
   int len = sprintf(banner, "\r\n\r\nRunning Test %d.", state + 1);
   if (state != DEMO_STATE_TEST_1)
   {
      len += sprintf(&banner[len], "  Jumps are shown as %% of %d ms.\r\n",
                     TICK_TEST_SAMPLING_INTERVAL_MS);
   }
   HAL_UART_Transmit(&huart2, (uint8_t*)banner, len, HAL_MAX_DELAY);
}

static int xDescribeTickTestResults(TttResults_t* results, int periodNumber, char* dest)
{
   int durationSeconds = (results->durationSs + results->subsecondsPerSecond/2) / results->subsecondsPerSecond;
   return (sprintf(dest, "Period %d: %d s, drift: %d/%d s, jump: %+d%% (min), %+d%% (max)",
                   periodNumber,
                   durationSeconds,
                   (int)results->driftSs, (int) results->subsecondsPerSecond,
                   results->minDriftRatePct,
                   results->maxDriftRatePct));
}

static int xUpdateResults( int xDemoState )
{
   static int resultsCount = 0;

   TttResults_t complete, inProgress;
   vTttGetResults(&complete, &inProgress);

   //      Build the results text here.  Use a "large" buffer because the results might require two lines of
   // terminal output.  The buffer is static so its contents remain valid even after we return from this
   // function.  In turn this allows us to return to our caller without waiting for the UART to finish.
   //
   static char textResults[200];
   static const char* const eraseLine = "\r\x1B[K";
   int resultsLen = sprintf(textResults, "%s", eraseLine);

   if (resultsCount != complete.resultsCounter)
   {
      resultsCount = complete.resultsCounter;
      if (resultsCount != 0)
      {
         resultsLen += xDescribeTickTestResults(&complete, resultsCount, &textResults[resultsLen]);
         resultsLen += sprintf(&textResults[resultsLen], "\r\n");
      }
   }

   resultsLen += xDescribeTickTestResults(&inProgress, resultsCount + 1, &textResults[resultsLen]);

   //      Send the results to the terminal.  In test 3, use interrupt-driven I/O with the terminal as
   // a way to enhance the stress test.  The interrupts induce a rapid sequence of early wake-ups from
   // tickless idle (if enabled).  This barrage of early wake-ups is a great stress test for the tickless
   // logic.  In the other demo states, use busy-wait I/O to avoid adding a test actor when we don't want one.
   //
   if (xDemoState == DEMO_STATE_TEST_3)
   {
      vUlpOnPeripheralsActive(ulpPERIPHERAL_USART2);
      HAL_UART_Transmit_IT(&huart2, (uint8_t*)textResults, resultsLen);
   }
   else
   {
      //      Because busy-wait I/O keeps this task "ready", it also prevents tickless idle, so we don't need
      // to bother notifying the ULP driver that the UART peripheral is active.
      //
      // vUlpOnPeripheralsActive(ulpPERIPHERAL_USART2);
      HAL_UART_Transmit(&huart2, (uint8_t*)textResults, resultsLen, HAL_MAX_DELAY);
      // vUlpOnPeripheralsInactive(ulpPERIPHERAL_USART2);
   }

   //      Apply some pass/fail criteria and report any failures.
   //
   int isFailure = pdFALSE;
   if (inProgress.minDriftRatePct < -2 || inProgress.maxDriftRatePct > 2)
   {
      isFailure = pdTRUE;
   }

   return (isFailure);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
   vUlpOnPeripheralsInactiveFromISR(ulpPERIPHERAL_USART2);
}

void vBlipLed( uint32_t ms )
{
   HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
   osDelay(ms);
   HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
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

  //      Compensate for a CubeMX omission in HAL_InitTick() in stm32l4xx_hal_timebase_tim.c.
  //
  uwTickPrio = TICK_INT_PRIORITY;

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
  MX_LPTIM2_Init();
  /* USER CODE BEGIN 2 */

  //      If the user is currently holding the blue button down, clear DBGMCU->CR.  This step prevents the
  // debugger from keeping clocks and regulators running when we use a low-power mode.  This in turn allows
  // the user to observe the "true" impact of low-power modes on current consumption.  Note that the debugger
  // will lose its connection as soon as we use a low-power mode, but we don't have any choice if we want to
  // see "true" current consumption.
  //
  if ( HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET )
  {
     DBGMCU->CR = 0; // POR and BOR also clear this register, but not the reset pin, and not other resets.
  }

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

  /* definition and creation of resultsTimer */
  osTimerDef(resultsTimer, resultsTimerCallback);
  resultsTimerHandle = osTimerCreate(osTimer(resultsTimer), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of mainTask */
  osThreadDef(mainTask, mainOsTask, osPriorityNormal, 0, 256);
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
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
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief LPTIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPTIM2_Init(void)
{

  /* USER CODE BEGIN LPTIM2_Init 0 */

  /* USER CODE END LPTIM2_Init 0 */

  /* USER CODE BEGIN LPTIM2_Init 1 */

  /* USER CODE END LPTIM2_Init 1 */
  hlptim2.Instance = LPTIM2;
  hlptim2.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  hlptim2.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV1;
  hlptim2.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
  hlptim2.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
  hlptim2.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  hlptim2.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
  hlptim2.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
  hlptim2.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;
  if (HAL_LPTIM_Init(&hlptim2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPTIM2_Init 2 */

  /* USER CODE END LPTIM2_Init 2 */

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

void HAL_LPTIM_AutoReloadMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
   //      The HAL calls this function when LPTIM2 has an ARR match event.  We use LPTIM2 as a stress-test
   // actor during test state 2.  This function has nothing to do with lptimTick.c or LPTIM1, but it does
   // help us *test* the tick timing provided by that code and that timer.

   //      Wake the main task, but for no reason.  We're just trying to stress test the tick timing.
   //
   BaseType_t xWasHigherPriorityTaskWoken = pdFALSE;
   xTaskNotifyFromISR( mainTaskHandle, 0, eNoAction, &xWasHigherPriorityTaskWoken);
   portYIELD_FROM_ISR(xWasHigherPriorityTaskWoken);
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

   //      Stop the timer that provides the HAL tick.  Now it won't needlessly interrupt tickless idle periods
   // that use sleep mode.  (The HAL tick already can't interrupt tickless idle periods that use stop mode,
   // because the HAL timer doesn't operate in stop mode.)  In a real application, the HAL tick might be
   // required by the HAL even after FreeRTOS has control.  It's still best to stop the timer here, and then
   // define HAL_GetTick() and HAL_Delay() to use the FreeRTOS tick (and delay) once available.
   //
   TIM17->CR1 &= ~TIM_CR1_CEN;  // wish CubeMX would generate a symbol for the HAL tick timer

   //      Be sure LPTIM2 ignores its input clock when the debugger stops program execution.
   //
   taskDISABLE_INTERRUPTS();
   DBGMCU->APB1FZR2 |= DBGMCU_APB1FZR2_DBG_LPTIM2_STOP;
   taskENABLE_INTERRUPTS();

   //      Start the demo in state 0.
   //
   int xDemoState = DEMO_STATE_TEST_1;
   vSetDemoState(xDemoState);

   int isFailureDetected = pdFALSE;
   uint32_t notificationFlags;
   for(;;)
   {
      //      Wait forever for any notification.
      //
      xTaskNotifyWait(0, UINT32_MAX, &notificationFlags, portMAX_DELAY);

      if (notificationFlags & NOTIFICATION_FLAG_B1_PIN)
      {
         //      Advance the demo state in response to the button press.  Clear any previous test failures.
         //
         if (++xDemoState > MAX_DEMO_STATE )
         {
            xDemoState = DEMO_STATE_TEST_1;
         }

         vSetDemoState(xDemoState);

         isFailureDetected = pdFALSE;
      }

      if (notificationFlags & NOTIFICATION_FLAG_RESULTS)
      {
         //      Make failure detections "sticky" so an observer can rely on the LED even for past failures.
         // We clear past failures when we advance the demo state above (for a button press).
         //
         if (xUpdateResults( xDemoState ))
         {
            isFailureDetected = pdTRUE;
         }
      }

      if (notificationFlags & NOTIFICATION_FLAG_LED_BLIP)
      {
         //      Blip the LED for 100ms.  Blip twice if the test has failed.
         //
         vBlipLed(100UL);
         if (isFailureDetected)
         {
            osDelay(100UL);
            vBlipLed(100UL);
         }
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
      xTaskNotify( mainTaskHandle, NOTIFICATION_FLAG_LED_BLIP, eSetBits );
   }

  /* USER CODE END ledTimerCallback */
}

/* resultsTimerCallback function */
void resultsTimerCallback(void const * argument)
{
  /* USER CODE BEGIN resultsTimerCallback */
   UNUSED(argument);

   if ( mainTaskHandle != NULL )
   {
      xTaskNotify( mainTaskHandle, NOTIFICATION_FLAG_RESULTS, eSetBits );
   }

  /* USER CODE END resultsTimerCallback */
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
