// Ultra Low Power API implementation (ulp.c)

#include "stm32l4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ulp.h"

void vUlpInit()
{
   //      Select STOP 2 as the stop mode we use whenever we set the SLEEPDEEP bit.  It's the deepest sleep
   // that doesn't end in reset.
   //
   MODIFY_REG(PWR->CR1, PWR_CR1_LPMS_Msk, PWR_CR1_LPMS_STOP2);

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


static uint32_t rccCfgrSave;
static uint32_t rccCrSave;

void vUlpPreSleepProcessing()
{
   if (xDeepSleepForbiddenFlags == 0)
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
      //      We may have been in deep sleep.  If we were, the hardware cleared several enable bits in the CR,
      // and it changed the selected system clock in CFGR.  Restore them now.  If we're restarting the PLL as
      // the CPU clock here, the CPU will not wait for it.  Instead, the CPU continues executing from the
      // wake-up clock (MSI in our case) until the PLL is stable and then the CPU starts using the PLL.
      //
      RCC->CR = rccCrSave;
      RCC->CFGR = rccCfgrSave;

      SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
   }
}
