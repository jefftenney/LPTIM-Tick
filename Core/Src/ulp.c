// Ultra Low Power API implementation (ulp.c)

#include "stm32l4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ulp.h"

// Use of MSI as the clock source for the PLL
//
//      An ultra-low-power application that uses MSI as the clock source for the PLL must ensure that the MSI
// frequency is compatible with the flash wait-state and core-voltage regulator configurations as if MSI were
// providing the core clock directly.  In these applications, MSI does provide the core clock directly for a
// short time after waking from stop mode.  This is typically not an issue because the PLL is typically set
// for a frequency faster than its incoming MSI frequency.

// Warning about HSE and Stop modes
//
//      An ultra-low-power application that uses HSE should use it only when needed, and should avoid the use
// of stop modes while HSE is in use.  This advice applies even if HSE is used in bypass mode.  An easy way to
// avoid stop modes while HSE is in use is to register it as a peripheral not safe for stop modes.  See
// ulpPERIPHERAL_HSE in ulp.h for example.
//
//      This implementation of ULP assumes the application follows the advice above.  The clock configuration
// that doesn't use HSE is considered the ULP-friendly clock configuration.

void vUlpInit()
{
   #ifdef configMIN_RUN_BETWEEN_DEEP_SLEEPS    // Errata workaround
   {
      //      Set the SysTick "load" register (or reload-value register) to support the errata workaround.
      //
      //      The SysTick timer uses a 24-bit counting register.  The longest minimum-run time is 15us
      // according to the errata sheet.  A 24-bit value (up to 16.7M) is more than sufficient for 15us,
      // no matter the core clock rate.
      //
      configASSERT( configMIN_RUN_BETWEEN_DEEP_SLEEPS <= 0x00FFFFFFU &&
                    configMIN_RUN_BETWEEN_DEEP_SLEEPS != 0 );
      SysTick->LOAD = configMIN_RUN_BETWEEN_DEEP_SLEEPS;

      //      Be sure SysTick uses the lowest interrupt priority.
      //
      NVIC_SetPriority(SysTick_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY);
   }
   #endif

   //      Be sure the MCU wakes up from stop mode on the same clock we normally use as the core clock, if
   // possible.  Might as well give the MCU a head start getting the clock going while waking from STOP.
   //
   if ( (RCC->CFGR & RCC_CFGR_SWS_Msk) == RCC_CFGR_SWS_HSI )
   {
      SET_BIT(RCC->CFGR, RCC_CFGR_STOPWUCK);
   }
   else if ( (RCC->CFGR & RCC_CFGR_SWS_Msk) == RCC_CFGR_SWS_MSI )
   {
      CLEAR_BIT(RCC->CFGR, RCC_CFGR_STOPWUCK);
   }
   else
   {
      //     An assertion failure here means the application has called vUlpInit() while HSE is providing the
      // core clock.  The application must call vUlpInit() while the ULP-friendly clock configuration is in
      // place.  See "Warning about HSE and Stop modes" above for more information.
      //
      configASSERT( (RCC->CFGR & RCC_CFGR_SWS_Msk) == RCC_CFGR_SWS_PLL );

      //      The application has selected the PLL as the system clock.  In this case, we optimize the
      // wake-up timing by configuring the MCU to wake from the source clock for the PLL (MSI or HSI).
      //
      if ( (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC_Msk) == RCC_PLLCFGR_PLLSRC_HSI )
      {
         SET_BIT(RCC->CFGR, RCC_CFGR_STOPWUCK);
      }
      else
      {
         CLEAR_BIT(RCC->CFGR, RCC_CFGR_STOPWUCK);

         //      An assertion failure here means the application has called vUlpInit() while HSE is providing
         // the input clock to the PLL.  The application must call vUlpInit() while the ULP-friendly clock
         // configuration is in place.  See "Warning about HSE and Stop modes" above for more information.
         //
         configASSERT( (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC_Msk) == RCC_PLLCFGR_PLLSRC_MSI );
      }
   }
}

static volatile int xDeepSleepForbiddenFlags = 0;

void vUlpOnPeripheralsActive( int xPeripherals )
{
   taskENTER_CRITICAL();
   xDeepSleepForbiddenFlags |= xPeripherals;
   taskEXIT_CRITICAL();
}

void vUlpOnPeripheralsActiveFromISR( int xPeripherals )
{
   UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
   xDeepSleepForbiddenFlags |= xPeripherals;
   taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

void vUlpOnPeripheralsInactive( int xPeripherals )
{
   taskENTER_CRITICAL();
   xDeepSleepForbiddenFlags &= ~xPeripherals;
   taskEXIT_CRITICAL();
}

void vUlpOnPeripheralsInactiveFromISR( int xPeripherals )
{
   UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
   xDeepSleepForbiddenFlags &= ~xPeripherals;
   taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}


//      Functions vUlpPreSleepProcessing() and vUlpPostSleepProcessing() are called from a critical section,
// so we happily take a few shortcuts made safe by that usage model.
//
static uint32_t rccCfgrSave;
static uint32_t rccCrSave;

void vUlpPreSleepProcessing()
{
   int useDeepSleep = pdFALSE;
   if (xDeepSleepForbiddenFlags == 0)
   {
      useDeepSleep = pdTRUE;
      MODIFY_REG(PWR->CR1, PWR_CR1_LPMS_Msk, PWR_CR1_LPMS_STOP2);
   }
   else if ((xDeepSleepForbiddenFlags & ~ulpPERIPHERALS_OK_IN_STOP1) == 0)
   {
      useDeepSleep = pdTRUE;
      MODIFY_REG(PWR->CR1, PWR_CR1_LPMS_Msk, PWR_CR1_LPMS_STOP1);
   }

   if (useDeepSleep)
   {
      rccCrSave = RCC->CR;
      rccCfgrSave = RCC->CFGR;

      SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
   }
}

void vUlpPostSleepProcessing()
{
   if (SCB->SCR & SCB_SCR_SLEEPDEEP_Msk)
   {
      #ifdef configMIN_RUN_BETWEEN_DEEP_SLEEPS    // Errata workaround
      {
         //     Start a new SysTick timer period.  We won't attempt to enter STOP mode until the timer period
         // ends.  Note that we do start a new period here unnecessarily if the CPU didn't actually enter stop
         // mode (due to a pending interrupt).  That's OK.
         //
         SysTick->VAL = 0;
         SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk;

         //      Mark the "min-run" peripheral as being now in use so that we won't attempt STOP mode until
         // it's no longer in use.  See SysTick_Handler() below.
         //
         xDeepSleepForbiddenFlags |= ulpPERIPHERAL_MIN_RUN;
      }
      #endif

      //      We may have been in deep sleep.  If we were, the hardware cleared several enable bits in the CR,
      // and it changed the selected system clock in CFGR.  Restore them now.  If we're restarting the PLL as
      // the CPU clock here, the CPU will not wait for it.  Instead, the CPU continues executing from the
      // wake-up clock (MSI or HSI) until the PLL is stable and then the CPU starts using the PLL.
      //
      RCC->CR = rccCrSave;
      RCC->CFGR = rccCfgrSave;

      SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

      //      This application bypasses the RTC shadow registers, so we don't need to clear the sync flag for
      // those registers.  They are always out of sync when coming out of deep sleep.
      //
      // RTC->ISR &= ~RTC_ISR_RSF;
   }
}

#ifdef configMIN_RUN_BETWEEN_DEEP_SLEEPS    // Errata workaround
   //      When the SysTick timer expires, we allow STOP mode again.
   //
   void SysTick_Handler()
   {
      //      Stop the SysTick timer.  We use it in "one-shot" mode to know when it's safe to use STOP mode
      // again.  Then mark our "min-run" peripheral as no longer in use.
      //
      SysTick->CTRL = 0;
      vUlpOnPeripheralsInactiveFromISR( ulpPERIPHERAL_MIN_RUN );
   }
#endif
