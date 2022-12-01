// lptimTick.c -- Jeff Tenney
//
// STM32 No-Drift FreeRTOS Tick/Tickless via LPTIM
//
// Revision: 2021.11.23
// Tabs: None
// Columns: 110
// Compiler: gcc (GNU) / armcc (Arm-Keil) / iccarm (IAR)
// SPDX-License-Identifier: MIT


// Copyright 2021 Jeff Tenney <jeff.tenney@gmail.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
// associated documentation files (the "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the
// following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial
// portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
// LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO
// EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
// IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
// USE OR OTHER DEALINGS IN THE SOFTWARE.


#include "FreeRTOS.h"
#include "task.h"
#include "stm32l4xx.h"

//      This FreeRTOS port "extension" for STM32 uses LPTIM to generate the OS tick instead of the systick
// timer.  The benefit of the LPTIM is that it continues running in "stop" mode as long as its clock source
// does.  A typical clock source for the LPTIM is LSE (or LSI), which does keep running in stop mode.
//
//      The resulting FreeRTOS port:
//
//   o Allows use of low-power stop modes during tickless idle, while still keeping kernel time.
//   o Eliminates kernel-time drift associated with tickless idle in official FreeRTOS port
//   o Eliminates kernel-time drift caused by rounding the OS tick to a whole number of timer counts.
//   o Avoids drift and errors found in other LPTIM implementations available from ST or the public domain.
//   o Detects/reports ticks dropped due to the application masking interrupts (the tick ISR) for too long.
//
//      This software is currently adapted for STM32L4(+) but is easily adaptable to (or already compatible
// with) any STM32 that provides an LPTIM peripheral, such as STM32L, STM32F, STM32G, STM32H, STM32W, and the
// new STM32U.


// Terminology
//
//      "Count" - What a timer does in its "count" register (CNT).
//
//      "Tick" - The OS tick, made up of some number of timer counts.


// Perfect Tick Frequency
//
//      This software optionally varies the number of timer counts per OS tick to achieve the target OS tick
// frequency.  The OS tick generally occurs no more than half a timer count early or half a timer count late
// compared to the ideal tick time.  This is especially useful with a 32768Hz reference on LSE and a desired
// 1000Hz system tick.  In that case, this software uses 32-count and 33-count tick durations as needed to
// stay on the 1000Hz tick schedule.  No matter how you set configLPTIM_REF_CLOCK_HZ and configTICK_RATE_HZ,
// this software stays precisely on schedule.
//
//      You can disable this feature and instead use a simple, constant number of timer counts per OS tick by
// defining configLPTIM_ENABLE_PRECISION to 0.  Your effective tick rate will be as close as possible to
// configTICK_RATE_HZ while using a constant number of counts per tick.  For example, with a 32768 Hz clock
// and an ideal tick frequency of 1000 Hz, the actual tick frequency is ~993 Hz.


// Silicon Bug
//
//      The LPTIM has a silicon bug that can cause the CPU to be temporarily stuck in the LPTIM ISR with no
// way for the CPU to clear the IRQ.  The silicon bug is not documented by ST (yet), but the "stuck" condition
// appears to last one full count of the LPTIM.  It seems to occur only when the application is using the MCU
// "stop" power modes.
//
//      Any attempt to use stop mode in the window between a "match" and the CMPM one count later causes the
// match interrupt to be asserted (early), and it cannot be cleared until the CMPM flag is set.  Additionally,
// any attempt to use stop mode during the timer count after a CMPM event that we tried to suppress too late
// also results in the entire count duration stuck in the ISR.  By "suppress too late", we mean write a new
// value to CMP just before a CMPM event.
//
//      As a workaround, the application should be sure that configTICK_INTERRUPT_PRIORITY is a lower priority
// than application interrupts.  We considered a workaround in this file but realized that our only viable
// recourse is to avoid stop mode at strategic times, but those time windows last several timer counts.  Using
// sleep mode instead of stop mode for several counts is more costly than the silicon bug itself, which uses
// run mode instead of stop mode for one timer count (stuck in interrupt handlers).


// Quirks of LPTIM
//
//      The following "quirks" indicate that LPTIM is designed for PWM output control and not for generating
// timed interrupts.  This software overcomes all of them.
//
//   o Writes to CMP are delayed by a sync mechanism inside the timer.  The sync takes ~3 timer counts.
//   o During the synchronization process, additional writes to CMP are prohibited.
//   o CMPOK (sync completion) comes *after* the CMP value is asynchronously available for match events.
//   o The match condition is not simply "CNT == CMP" but is actually "CNT >= CMP && CNT != ARR".
//   o The timer sets CMPM (and optional IRQ) one timer count *after* the match condition is reached.
//   o With a new CMP value, the timer must first be in a "no-match" condition to generate a match event.
//   o Setting CMP == ARR is prohibited.  See below.
//   o Changing IER (interrupt-enable register) is prohibited while LPTIM is enabled.
//   o The CPU can get stuck temporarily in the LPTIM ISR when using stop mode.  See "Silicon Bug" above.
//
//      This software sets ARR to 0xFFFF permanently and modifies CMP to arrange each next tick interrupt.
// Due to the rule against setting CMP == ARR, we never set CMP to 0xFFFF.  If the ideal CMP value for a tick
// interrupt is 0xFFFF, we use 0 instead.


// Side Effects
//
//   o Tick Overhead.  OS ticks generated by this software have more overhead than ticks generated by the
//     official port code.  In most applications, the overhead doesn't make any real difference.  Our tick ISR
//     is longer by a handful of CPU instructions, and we execute a 2nd, very short ISR in between ticks.
//
//   o Tick IRQ Priority.  The tick IRQ priority must be high enough that no combination of ISRs can block its
//     execution for longer than 1 tick.  See configTICK_INTERRUPT_PRIORITY (below) for more information.
//     However, the application may safely mask interrupts for longer than 1 tick, rare as that need may be.
//     (One common example is for "fast-programming" flash memory on STM32.)  In that case, this software even
//     reports dropped ticks afterward via traceTICKS_DROPPED().
//
//   o Tick Jitter.  When precision is enabled (configLPTIM_ENABLE_PRECISION), ticks generated by this
//     software have jitter, as described above in "Perfect Tick Frequency".  In most applications, jitter in
//     the tick periods is not a concern.


#if ( !defined(configUSE_TICKLESS_IDLE) || configUSE_TICKLESS_IDLE != 2 )
#warning Please edit FreeRTOSConfig.h to define configUSE_TICKLESS_IDLE as 2 *or* exclude this file.
#else

#ifdef xPortSysTickHandler
#warning Please edit FreeRTOSConfig.h to eliminate the preprocessor definition for xPortSysTickHandler.
#endif


//      Symbol configTICK_INTERRUPT_PRIORITY, optionally defined in FreeRTOSConfig.h, controls the tick
// interrupt priority.  Most applications should define configTICK_INTERRUPT_PRIORITY to be
// configLIBRARY_LOWEST_INTERRUPT_PRIORITY because the tick doesn't need a high priority.  But if your
// application has long ISRs, you may need to increase the priority of the tick.  The tick priority must be
// high enough that no combination of ISRs at or above its priority level can block the tick ISR for longer
// than 1 tick.  But the priority must not be higher than configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
// (meaning that the value of configTICK_INTERRUPT_PRIORITY must not be numerically lower).
//
//      Be sure to consider the information in "Silicon Bug" above before increasing the interrupt priority of
// the system tick.
//
#ifndef configTICK_INTERRUPT_PRIORITY
#define configTICK_INTERRUPT_PRIORITY configLIBRARY_LOWEST_INTERRUPT_PRIORITY // default only; see above
#endif

//      Symbol configTICK_USES_LSI, optionally defined in FreeRTOSConfig.h, is defined only when LPTIM should
// use LSI as the clock instead of LSE.  By default, however, this software configures LPTIM to use LSE
// because a key feature of this software is timing accuracy -- no drift in tickless idle.
//
#ifdef configTICK_USES_LSI
   #define LPTIMSEL_Val 1 // LSI
   #define IS_REF_CLOCK_READY() (RCC->CSR & RCC_CSR_LSIRDY)
#else
   #define LPTIMSEL_Val 3 // LSE
   #define IS_REF_CLOCK_READY() (RCC->BDCR & RCC_BDCR_LSERDY)
#endif

//      Symbol configLPTIM_REF_CLOCK_HZ, optionally defined in FreeRTOSConfig.h, is the frequency of the
// selected reference clock or source clock for LPTIM.  If configTICK_USES_LSI is defined, then
// configLPTIM_REF_CLOCK_HZ equals the frequency of LSI (typically 32000 Hz or 37000 Hz depending on the MCU).
// Otherwise, configLPTIM_REF_CLOCK_HZ equals the frequency of LSE (usually 32768 Hz).
//
#ifndef configLPTIM_REF_CLOCK_HZ
#define configLPTIM_REF_CLOCK_HZ 32768UL
#endif

//      Symbol configLPTIM_ENABLE_PRECISION, optionally defined in FreeRTOSConfig.h, allows a configuration to
// eliminate the arithmetic in this software that maintains configTICK_RATE_HZ with perfect precision, as
// described above in "Perfect Tick Frequency".  The arithmetic corrects for any error in rounding the desired
// tick duration to a whole number of timer counts.  No matter how you set the precision option, this software
// eliminates the drift normally associated with tickless idle.  The precision option is enabled by default
// because it has a very small footprint by all measures (flash, RAM, execution time).  However, because the
// precision feature requires additional division operations, Cortex M0 users may consider disabling it.  CM0
// does not have a native divide instruction, so division operations are a little slow on that platform.
//
#ifndef configLPTIM_ENABLE_PRECISION
#define configLPTIM_ENABLE_PRECISION 1
#endif

//      If the application masks interrupts (specifically the tick interrupt) long enough to drop a tick, then
// the tick ISR calls this trace macro to report the condition along with the number of ticks dropped.  You
// can define this macro in FreeRTOSConfig.h.  Remember that the macro executes from within the tick ISR.
// Also be aware that the ISR resets the phase of the ticks after calling this macro.
//
//      If ticks are dropped and you did *not* mask the tick interrupt long enough to drop a tick, you
// probably have ISRs blocking the tick ISR for too long.  This is a serious issue that requires changes to
// your design.  Please see configTICK_INTERRUPT_PRIORITY above.
//
#ifndef traceTICKS_DROPPED
#define traceTICKS_DROPPED(x)
#endif

#define LPTIM_CLOCK_HZ ( configLPTIM_REF_CLOCK_HZ )

static TickType_t xMaximumSuppressedTicks;      //   We won't try to sleep longer than this many ticks during
                                                // tickless idle because any longer might confuse the logic in
                                                // our implementation.

static uint32_t ulTimerCountsForOneTick;        //   A "baseline" tick has this many timer counts.  The
                                                // baseline tick is as close as possible to the ideal duration
                                                // but is a whole number of timer counts.

#if ( configLPTIM_ENABLE_PRECISION != 0 )

   static int lSubcountErrorPerTick;            //   A "baseline" tick has this much error, measured in timer
                                                // subcounts.  There are configTICK_RATE_HZ subcounts per
                                                // count.  When this field is negative, the baseline tick is a
                                                // little too long because we rounded "up" to the nearest
                                                // whole number of counts per tick.  When this field is
                                                // positive, the baseline tick is a little too short because
                                                // we rounded "down" to the nearest whole number of counts per
                                                // tick.

   static volatile int lRunningSubcountError;   //   This error accumulator never exceeds half a count, or
                                                // configTICK_RATE_HZ/2.  When this field is negative, the
                                                // next tick is slightly late; when this field is positive,
                                                // the next tick is slightly early.  This field allows us to
                                                // schedule each tick on the timer count closest to the ideal
                                                // tick time.
#endif // configLPTIM_ENABLE_PRECISION

static volatile uint16_t usIdealCmp;            //   This field doubles as a write cache for LPTIM->CMP and a
                                                // way to remember that we set CMP to 0 because 0xFFFF isn't
                                                // allowed (hardware limitation).

static volatile uint8_t isCmpWriteInProgress;   //   This field helps us remember when we're waiting for the
                                                // CMP write to finish.  We must not write to CMP while a
                                                // previous write is still in progress.

static volatile uint8_t isTickNowSuppressed;    //   This field helps the tick ISR determine whether
                                                // usIdealCmp is in the past or the future.


// LPTIM Instance Selection
//
//      If your MCU has multiple LPTIM instances, you must (1) select the instance you want this software to
// use for the OS tick by updating the the following three #defines, and (2) change the first five statements
// of vPortSetupTimerInterrupt() to match your selection.  The default configuration is for LPTIM1 because it
// operates even in the lowest-power STOP level.
//
//      If your MCU has only one LPTIM instance, you may or may not need to update these three #defines.  But
// you must change the first five statements of vPortSetupTimerInterrupt() to match your STM32.
//
#ifndef LPTIM
#define LPTIM              LPTIM1
#define LPTIM_IRQn         LPTIM1_IRQn
#define LPTIM_IRQHandler   LPTIM1_IRQHandler
#endif

//============================================================================================================
// vPortSetupTimerInterrupt()
//
//      This function overrides the "standard" port function, decorated with __attribute__((weak)), in port.c.
// Call with interrupts masked.
//
void vPortSetupTimerInterrupt( void )
{
   //      Enable the APB clock to the LPTIM.  Then select either LSE or LSI as the kernel clock for the
   // LPTIM.  Then be sure the LPTIM "freezes" when the debugger stops program execution.  Then reset the
   // LPTIM just in case it was already in use prior to this function.
   //
   //      Modify these statements as needed for your STM32.  See LPTIM Instance Selection (above) for
   // additional information.
   //
   RCC->APB1ENR1 |= RCC_APB1ENR1_LPTIM1EN;
   MODIFY_REG(RCC->CCIPR, RCC_CCIPR_LPTIM1SEL, LPTIMSEL_Val << RCC_CCIPR_LPTIM1SEL_Pos);
   DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_LPTIM1_STOP;
   RCC->APB1RSTR1 |= RCC_APB1RSTR1_LPTIM1RST;   // Reset the LPTIM module per erratum 2.14.1.
   RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_LPTIM1RST;
   #ifdef STM32WL   // <-- "Family" symbol is defined in the ST device header file, e.g., "stm32wlxx.h".
   {
      #define EXTI_IMR1_LPTIM1   (1UL << 29)
      #define EXTI_IMR1_LPTIM2   (1UL << 30)
      #define EXTI_IMR1_LPTIM3   (1UL << 31)

      //      Users of STM32WL must also change this next statement to match their LPTIM instance selection.
      // By default these MCU's disable wake-up from deep sleep via LPTIM.  An oversight by ST?
      //
      EXTI->IMR1 |= EXTI_IMR1_LPTIM1;
   }
   #endif

   //      Be sure the reference clock is ready.  If this assertion fails, be sure your application code
   // starts the reference clock (LSE or LSI) prior to starting FreeRTOS.
   //
   configASSERT(IS_REF_CLOCK_READY());

   //      Calculate the constants required to configure the tick interrupt.
   //
   ulTimerCountsForOneTick = ( LPTIM_CLOCK_HZ + ( configTICK_RATE_HZ / 2 ) ) / configTICK_RATE_HZ;
   configASSERT( ulTimerCountsForOneTick >= 4UL );  // CLOCK frequency must be at least 3.5x TICK frequency

   //      Calculate the maximum number of ticks we can suppress.  Give 1 OS tick of margin between clearly
   // future match events and clearly past match events.  Anything within the previous one tick is clearly
   // past, within one tick before that is in the margin between, which we call the past for convenience.
   // Everything else is in the future when isTickNowSuppressed is true and in the past otherwise.  And
   // set up a couple of other things for the precision feature, if enabled.

   #if ( configLPTIM_ENABLE_PRECISION == 0 )
   {
      xMaximumSuppressedTicks = 65536UL / ulTimerCountsForOneTick - 1 - 1;
   }
   #else
   {
      xMaximumSuppressedTicks = 65536UL * configTICK_RATE_HZ / LPTIM_CLOCK_HZ - 1 - 1;

      //      For convenience, the code above rounded *up* if the ideal number of counts per tick is exactly
      // X.5.  So we might calculate lSubcountErrorPerTick to be -(configTICK_RATE_HZ/2) but never
      // +(configTICK_RATE_HZ/2).  If you get a build error on this line, be sure configTICK_RATE_HZ is a
      // simple numeric literal (eg, 1000UL) with no C typecasting (eg, (TickType_t) 1000).
      //
      #if ( LPTIM_CLOCK_HZ % configTICK_RATE_HZ < configTICK_RATE_HZ/2 )
         #define IS_SUBCOUNT_EPT_POSITIVE 1
      #else
         #define IS_SUBCOUNT_EPT_POSITIVE 0
      #endif

      lSubcountErrorPerTick = LPTIM_CLOCK_HZ - ( ulTimerCountsForOneTick * configTICK_RATE_HZ );
      configASSERT( lSubcountErrorPerTick != configTICK_RATE_HZ / 2 );
   }
   #endif // configLPTIM_ENABLE_PRECISION


   //      Configure and start LPTIM.
   //
   LPTIM->IER = LPTIM_IER_CMPMIE | LPTIM_IER_CMPOKIE;   // Modify this register only when LPTIM is disabled.
   LPTIM->CFGR = (0 << LPTIM_CFGR_PRESC_Pos);           // Modify this register only when LPTIM is disabled.
   LPTIM->CR = LPTIM_CR_ENABLE;
   LPTIM->ARR = 0xFFFF;        // timer period = ARR + 1.  Modify this register only when LPTIM is enabled.
   LPTIM->CMP = ulTimerCountsForOneTick;                // Modify this register only when LPTIM is enabled.
   isCmpWriteInProgress = pdTRUE;
   usIdealCmp = ulTimerCountsForOneTick;
   #if ( configLPTIM_ENABLE_PRECISION != 0 )
   {
      lRunningSubcountError = lSubcountErrorPerTick;
   }
   #endif // configLPTIM_ENABLE_PRECISION
   LPTIM->CR |= LPTIM_CR_CNTSTRT;

   //      Enable the timer interrupt at the configured priority.  See configTICK_INTERRUPT_PRIORITY for
   // important details.
   //
   NVIC_SetPriority( LPTIM_IRQn, configTICK_INTERRUPT_PRIORITY );
   NVIC_EnableIRQ( LPTIM_IRQn );
}


//============================================================================================================
// vPortSuppressTicksAndSleep()
//
//      This function overrides the "official" port function, decorated with __attribute__((weak)), in port.c.
// The idle task calls this function with the scheduler suspended, and only when xExpectedIdleTime is >= 2.
//
//      FreeRTOS version 10.4.0 or newer is recommended to ensure this function doesn't potentially return one
// OS tick *after* the intended time.
//
void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime )
{
   //      Limit the time we plan to spend in tickless idle.  LPTIM has only so much range.
   //
   if (xExpectedIdleTime > xMaximumSuppressedTicks)
   {
      xExpectedIdleTime = xMaximumSuppressedTicks;
   }

   //      Determine the number of "extra" timer counts to add to the compare register, which is currently set
   // for the next tick.  Because the next tick is less than one tick away, we know we won't set the compare
   // register more than xMaximumSuppressedTicks (in timer counts) from the *current* CNT value.
   //
   #if ( configLPTIM_ENABLE_PRECISION != 0 )
      uint32_t ulExtraCounts = (xExpectedIdleTime - 1UL) * LPTIM_CLOCK_HZ / configTICK_RATE_HZ;
      int32_t lExtraError = (xExpectedIdleTime - 1UL) * LPTIM_CLOCK_HZ % configTICK_RATE_HZ;
   #else
      uint32_t ulExtraCounts = (xExpectedIdleTime - 1UL) * ulTimerCountsForOneTick;
   #endif // configLPTIM_ENABLE_PRECISION

   //      Enter a critical section so we can safely check the sleep-mode status.  But don't use
   // taskENTER_CRITICAL() because on many platforms that function masks interrupts that we need to exit sleep
   // mode.  We must stay in the critical section until we go to sleep so that any interrupt starting now
   // wakes us up from sleep.
   //
   __disable_irq();
   // __ISB() is not needed here.  The CPSID instruction used by  __disable_irq() is self synchronizing.

   //      If a context switch is pending or a task is waiting for the scheduler to be unsuspended, then
   // abandon the low power entry and the critical section.  This status cannot change while interrupts are
   // masked.
   //
   if (eTaskConfirmSleepModeStatus() == eAbortSleep)
   {
      __enable_irq();
   }
   else
   {
      #if ( configLPTIM_ENABLE_PRECISION != 0 )
      {
         //      Adjust ulExtraCounts if needed to maintain proper alignment.  We left lExtraError positive
         // above instead of minimizing its absolute value, so we don't need to check the final value of
         // lRunningSubcountError for being too negative.
         //
         lRunningSubcountError += lExtraError;
         if (lRunningSubcountError > (int)(configTICK_RATE_HZ/2))
         {
            ulExtraCounts++;
            lRunningSubcountError -= configTICK_RATE_HZ;
         }
      }
      #endif // configLPTIM_ENABLE_PRECISION

      //      Before we suppress the tick by modifying usIdealCmp (and eventually CMP), make a note that the
      // tick is now suppressed.  The order isn't actually important because we're in a critical section.  The
      // tick ISR uses this field to help it determine whether usIdealCmp is in the past or in the future.
      //
      //     Our design relies on the tick interrupt having high enough priority that other ISRs can't delay
      // the tick ISR too much while isTickNowSuppressed is true.  Too much delay would cause the ISR to
      // reject the tick at the end of the delay because the tick would appear to be still in the future, but
      // we need that tick, either to get us out of the loop below or to help us decide if we reached the tick
      // after the loop.
      //
      isTickNowSuppressed = pdTRUE;

      //      Add the extra counts to the upcoming timer interrupt.  If we can't write to the CMP register
      // right now, the ISR for CMPOK will do it for us.  If the timer happens to match on the old value
      // before the new value takes effect (any time after we mask interrupts above), the tick ISR rejects it
      // as a tick when we unmask interrupts below.
      //
      usIdealCmp += ulExtraCounts;  // (usIdealCmp is a uint16_t)
      if (!isCmpWriteInProgress)
      {
         isCmpWriteInProgress = pdTRUE;
         LPTIM->CMP = usIdealCmp == 0xFFFF ? 0 : usIdealCmp;  // never write 0xFFFF to CMP (HW rule)
      }
      uint32_t ulExpectedEndCmp = usIdealCmp;

      //      Because our implementation uses an interrupt handler to process a successful write to CMP, we
      // use a loop here so we won't return to our caller merely for that interrupt.  Nor will we return for a
      // tick ISR that rejects the tick as described above, nor for any other ISR that doesn't make a task
      // ready to execute.  And don't worry about ISRs changing how long the OS expects to be idle; FreeRTOS
      // doesn't let ISRs do that -- not even the xTimerXyzFromISR() functions.  Those functions merely queue
      // jobs for the timer task, which from our perspective is a task now ready to execute.
      //
      //      Stay in the loop until an ISR makes a task ready to execute or until the timer reaches the end
      // of the sleep period.  We identify the end of the sleep period by recognizing that the tick ISR has
      // modified usIdealCmp for the next tick after the sleep period ends.
      //
      do
      {
         //      Give the application a chance to arrange for the deepest sleep it can tolerate right now.
         // Also give it an opportunity to provide its own WFI or WFE instruction.  (In that case, it sets
         // xModifiableIdleTime = 0.)  After the sleep, give the application a chance to restore clocks or
         // take other recovery measures related to deep sleep.
         //
         //      Note that we don't recalculate xModifiableIdleTime in the case of multiple passes through
         // this loop, even though the amount of expected idle time does shrink with each pass.  We don't want
         // to burden this loop with those calculations because the typical ISR requests a context switch,
         // inducing an exit from this loop, not another pass.  The CMPOK interrupt is the exception, and is
         // the primary reason for this loop, but it comes within one tick anyway, so there is no need to
         // recalculate xModifiableIdleTime for that case.
         //
         TickType_t xModifiableIdleTime = xExpectedIdleTime;
         configPRE_SLEEP_PROCESSING( xModifiableIdleTime );
         if (xModifiableIdleTime > 0)
         {
            //      Wait for an interrupt.
            //
            __DSB();
            __WFI();
            __ISB(); // required when debugging in low-power modes (ie, DBGMCU->CR != 0) on some MCUs.
         }
         configPOST_SLEEP_PROCESSING( (const TickType_t)xExpectedIdleTime );

         //      Re-enable interrupts, and then execute the ISR tied to the interrupt that brought the MCU out
         // of sleep mode.
         //
         __enable_irq();
         __ISB();  // ISB is recommended by ARM; not strictly needed in Cortex-M when __disable_irq() is next.

         //      Disable interrupts for our call to eTaskConfirmSleepModeStatus() and in case we iterate again
         // in the loop.
         //
         __disable_irq();
         // __ISB() is not needed here.  The CPSID instruction used by  __disable_irq() is self synchronizing.

      } while (usIdealCmp == ulExpectedEndCmp && eTaskConfirmSleepModeStatus() != eAbortSleep);

      //      Re-enable interrupts.  We try our best to support short ISR latency, especially for interrupt
      // priorities higher than configMAX_SYSCALL_INTERRUPT_PRIORITY.
      //
      __enable_irq();

      //      Determine how many tick periods elapsed during our sleep.  And if something other than the tick
      // timer woke us up, reschedule the tick that would normally come after the ones we've just skipped (if
      // any).
      //
      //      Begin by assuming we managed to stay asleep the entire time.  In that case, the tick ISR already
      // added one tick (well, actually the ISR "pended" the increment because the scheduler is currently
      // suspended, but it's all the same to us), so we use "- 1" here.
      //
      TickType_t xCompleteTickPeriods = xExpectedIdleTime - (TickType_t) 1;

      //      We identify that we reached the end of the expected idle time by noting that the tick ISR has
      // modified usIdealCmp.  So if it hasn't, then we probably have to reschedule the next tick as described
      // above.  We temporarily mask the tick interrupt while we make the assessment and manipulate usIdealCmp
      // (and CMP) if necessary.  We also mask any interrupts at or below its interrupt priority since those
      // interrupts are allowed to use consecutive execution time enough to cause us to miss ticks.
      //
      portDISABLE_INTERRUPTS();
      if (usIdealCmp == ulExpectedEndCmp)
      {
         //      Something else woke us up.  See how many timer counts we still had left, and then use that
         // number to determine how many OS ticks actually elapsed.  Then reschedule the next tick exactly
         // where it would have been.

         //      Get a coherent copy of the current count value in the timer.  The CNT register is clocked
         // asynchronously, so we keep reading it until we get the same value during a verification read.
         //
         uint32_t ulCurrCount;
         do ulCurrCount = LPTIM->CNT; while (ulCurrCount != LPTIM->CNT);

         //      See how many timer counts we still had left, but don't include the timer count currently
         // underway.  If a tick happens to align with the beginning of the timer count currently underway, we
         // consider it already suppressed.  Or, in the case we didn't suppress any ticks, if the timer count
         // currently underway is the first one of a tick period, then that tick must have happened before the
         // idle task disabled the scheduler and called this function.  We don't want to reschedule *that*
         // interrupt.
         //
         //      Several conditions can cause ulFullCountsLeft to be "negative" here, meaning that we actually
         // have reached the end of the expected idle time and now merely need to allow the ISR to execute.
         // First, CNT may have incremented after we masked interrupts but before we captured ulCurrCount.
         // Second, LPTIM generates the CMPM IRQ one count *after* the match event.  Third, when usIdealCmp is
         // 0xFFFF, we set CMP to 0x0000, which when combined with the first two cases, could easily leave
         // countsLeft set to -3 (65533).  And finally, interrupts with priority above
         // configMAX_SYSCALL_INTERRUPT_PRIORITY can delay our capture of ulCurrCount.  That delay is limited
         // to less than one tick duration by stated requirement.  No need to do anything if ulFullCountsLeft
         // is "negative".  In fact there's no need to do anything if ulFullCountsLeft is less than a whole
         // tick, but xFullTicksLeft (below) determines that.
         //
         uint32_t ulFullCountsLeft = (uint16_t)(usIdealCmp - ulCurrCount - 1UL);
         #if ( configLPTIM_ENABLE_PRECISION == 0 )
         if (ulFullCountsLeft < xMaximumSuppressedTicks * ulTimerCountsForOneTick)
         #else
         if (ulFullCountsLeft <= xMaximumSuppressedTicks * LPTIM_CLOCK_HZ / configTICK_RATE_HZ) // See below
         #endif // configLPTIM_ENABLE_PRECISION
         {
            //      Now calculate how many "full" or whole ticks we had left in our expected idle time.  If
            // there are zero full ticks left, then the next scheduled tick interrupt is the one we want
            // anyway.  But if the time left amounts to at least one full tick, then reschedule the first tick
            // interrupt that we haven't yet skipped.  And update xCompleteTickPeriods not to count the ones
            // we haven't skipped.
            //
            TickType_t xFullTicksLeft;
            #if ( configLPTIM_ENABLE_PRECISION != 0 )
            {
               xFullTicksLeft = ulFullCountsLeft * configTICK_RATE_HZ / LPTIM_CLOCK_HZ;
               if (xFullTicksLeft == xExpectedIdleTime)
               {
                  //      For efficiency, the rescheduling code below might count a tick that occurs at the
                  // very next timer count, even before that count occurs, instead scheduling the subsequent
                  // tick.  With a slow timer, the timer might *still* not have made that count as we arrive
                  // here again.  Correct xFullTicksLeft for that case here.  This same case can cause
                  // ulFullCountsLeft to be equal to the max above (and not necessarily less) -- see above.
                  //
                  xFullTicksLeft = xExpectedIdleTime - (TickType_t)1;
               }
            }
            #else
            {
               xFullTicksLeft = ulFullCountsLeft / ulTimerCountsForOneTick;
            }
            #endif // configLPTIM_ENABLE_PRECISION

            //      Verify that we calculated a sensible number of full ticks remaining in the expected idle
            // time.  If this assertion fails, then the tick ISR was delayed too long by other ISR(s).  You
            // must either reduce the execution times of your ISRs, decrease their priorities, or increase the
            // priority of the tick ISR.  See the description of configTICK_INTERRUPT_PRIORITY for details.
            //
            configASSERT( xFullTicksLeft < xExpectedIdleTime );

            if (xFullTicksLeft != 0)
            {
               xCompleteTickPeriods -= xFullTicksLeft;

               //      Reschedule the next timer interrupt; it's one we had expected to suppress.  Also,
               // correct lRunningSubcountError because it still reflects the error we'll have at the end of
               // the expected idle time.  Modify it to reflect the error at the time of the tick interrupt
               // we're rescheduling now.
               //
               uint32_t ulFullTicksLeftAsCounts;
               #if ( configLPTIM_ENABLE_PRECISION != 0 )
               {
                  ulFullTicksLeftAsCounts = xFullTicksLeft * LPTIM_CLOCK_HZ / configTICK_RATE_HZ;
                  lExtraError = xFullTicksLeft * LPTIM_CLOCK_HZ % configTICK_RATE_HZ;
                  lRunningSubcountError -= lExtraError;
                  if (lRunningSubcountError < -(int)(configTICK_RATE_HZ/2))
                  {
                     ulFullTicksLeftAsCounts++;
                     lRunningSubcountError += configTICK_RATE_HZ;
                  }
               }
               #else
               {
                  ulFullTicksLeftAsCounts = xFullTicksLeft * ulTimerCountsForOneTick;
               }
               #endif // configLPTIM_ENABLE_PRECISION

               usIdealCmp -= ulFullTicksLeftAsCounts; // usIdealCmp is a uint16_t
               if (!isCmpWriteInProgress)
               {
                  isCmpWriteInProgress = pdTRUE;
                  LPTIM->CMP = usIdealCmp == 0xFFFF ? 0 : usIdealCmp;  // never write 0xFFFF to CMP (HW rule)
               }
            }
         }
      }

      //      Set isTickNowSuppressed back to false before we unmask the tick interrupt so the ISR has a
      // chance to identify any ticks missed while we had the tick interrupt masked.  Any missed ticks here
      // indicate a configuration error.  No combination of ISRs above the tick priority may execute for
      // longer than a tick.  See configTICK_INTERRUPT_PRIORITY.
      //
      isTickNowSuppressed = pdFALSE;
      portENABLE_INTERRUPTS();

      //      Increment the tick count by the number of (full) "extra" ticks we waited.  Function
      // vTaskStepTick() asserts that this function didn't oversleep.  Note that we don't have to worry about
      // modifying xTickCount count here while the tick count ISR is enabled because the scheduler is
      // currently suspended.  That causes the tick ISR to accumulate ticks into a pended-ticks field.
      //
      vTaskStepTick( xCompleteTickPeriods );
   }
}


//============================================================================================================
// LPTIM_IRQHandler()
//
void LPTIM_IRQHandler( void )
{
   //      Whether we are running this ISR for a CMPOK interrupt or a CMPM interrupt (or both), we need to see
   // if it's time for a tick.  Think of it as interrupt-induced polling.  The synchronization mechanism that
   // controls writes to CMP can cause us to miss CMPM events or to delay them while we wait to write the new
   // CMP value while a previous write is still ongoing.
   //
   //      There are two ways we can lose a CMPM interrupt.  First, with very high CMP values (near 0xFFFF),
   // if the counter reaches 0xFFFF before the sync mechanism finishes, the timer won't identify the match.
   // And second, with *any* CMP value that becomes available to the timer a little too late, the timer won't
   // identify a *new* match if the previous CMP value already matched and was lower than the new CMP value.
   // Either one of these conditions might occur in a single execution of vPortSuppressTicksAndSleep().  When
   // that function stops a sleep operation early due to an application interrupt, it tries to revert to
   // wherever the next scheduled tick should be.  In so doing, it may write a CMP value that is imminent.
   // That CMP value may be "very high", or it may be numerically larger than the previous compare value, as
   // it would be for a reversion that "unwraps" from a CMP value in the next counter epoch back through
   // 0xFFFF to a CMP value in the current epoch.
   //
   //      So we proceed now to identify a tick, whether we have a CMPM interrupt or not.

   //      Acknowledge and clear the CMPM event.  Based on the errata, we dare not clear this flag unless it
   // is already set.  Call it an over-abundance of caution.
   //
   if (LPTIM->ISR & LPTIM_ISR_CMPM)
   {
      LPTIM->ICR = LPTIM_ICR_CMPMCF;
   }

   //      Get a coherent copy of the current count value in the timer.  The CNT register is clocked
   // asynchronously, so we keep reading it until we get the same value during a verification read.  Then
   // determine how "late" we are in responding to the timer interrupt request.  Normally, we're one count
   // late because LPTIM raises the CMPM interrupt one count *after* the match event.  Use usIdealCmp in this
   // determination because it reliably reflects the match time we want right now, regardless of the sync
   // mechanism for CMP.
   //
   uint32_t ulCountValue;
   do ulCountValue = LPTIM->CNT; while (ulCountValue != LPTIM->CNT);
   uint32_t ulCountsLate = (uint16_t)(ulCountValue - usIdealCmp);

   //      If we're more than one full tick late, then the application masked interrupts for too long.  That
   // condition is typically an indication of a design flaw, so we don't take heroic measures here to avoid
   // dropping the tick(s) we missed or to keep upcoming ticks in proper phase.  Instead, we just resume the
   // tick starting right away.  Without this correction, a "missed" tick could drop all ticks for a long time
   // -- until the timer came back around to the CMP value again.
   //
   //      Be sure not to misinterpret other conditions though.  When the tick is suppressed and scheduled for
   // more than one tick from now, it looks like we're late.  And when the tick is not suppressed, and we're
   // still waiting for an upcoming tick, it looks like we're very, very late.  In those cases, we're not
   // actually late, and there is no tick right now.
   //
   if (ulCountsLate >= ulTimerCountsForOneTick &&
       ulCountsLate < 65536UL - 1 - ulTimerCountsForOneTick &&
       !isTickNowSuppressed)
   {
      //      Optionally report the number of ticks dropped.  (No need for precision here.)  Then arrange to
      // count the tick (below) and to schedule the next tick based on the current timer value.
      //
      traceTICKS_DROPPED( ulCountsLate / ulTimerCountsForOneTick );
      usIdealCmp = ulCountValue;
      ulCountsLate = 0;
   }

   //      If the ideal CMP value is in the recent past -- within one OS tick time -- then count the tick.
   //
   //      The condition of this one "if" statement handles several important cases.  First, it temporarily
   // ignores the unwanted match condition that occurs when vPortSuppressTicksAndSleep() wraps CMP into the
   // next timer epoch.  Second, it helps us ignore an imminent tick that vPortSuppressTicksAndSleep() is
   // trying to suppress but may occur anyway due to the CMP sync mechanism.  Third, it helps us honor an
   // interrupt that vPortSuppressTicksAndSleep() has restored even if it happens a little bit late due to the
   // sync mechanism, and even if that interrupt occurs before its corresponding CMPOK event occurs (happens
   // occasionally).  And finally, it helps us honor a tick that vPortSuppressTicksAndSleep() has restored,
   // but when the timer appears to have missed the match completely as explained above.
   //
   if (ulCountsLate < ulTimerCountsForOneTick)
   {
      //      We officially have an OS tick.  Count it, and set up the next one.

      uint32_t ulNumCounts = ulTimerCountsForOneTick;
      #if ( configLPTIM_ENABLE_PRECISION != 0 )
      {
         lRunningSubcountError += lSubcountErrorPerTick;
         #if ( IS_SUBCOUNT_EPT_POSITIVE )
         {
            if (lRunningSubcountError >= (int)(configTICK_RATE_HZ/2))
            {
               ulNumCounts++;
               lRunningSubcountError -= configTICK_RATE_HZ;
            }
         }
         #else
         {
            if (lRunningSubcountError <= -(int)(configTICK_RATE_HZ/2))
            {
               ulNumCounts--;
               lRunningSubcountError += configTICK_RATE_HZ;
            }
         }
         #endif // IS_SUBCOUNT_EPT_POSITIVE
      }
      #endif // configLPTIM_ENABLE_PRECISION

      //      Set up the next tick interrupt.  We must check isCmpWriteInProgress here as usual, in case CMPM
      // can come before CMPOK.
      //
      usIdealCmp += ulNumCounts;  // usIdealCmp is a uint16_t
      if (!isCmpWriteInProgress)
      {
         LPTIM->CMP = usIdealCmp == 0xFFFF ? 0 : usIdealCmp;  // never write 0xFFFF to CMP (HW rule)
         isCmpWriteInProgress = pdTRUE;
      }

      //      Tell the OS about the tick.  We don't need to bother saving and restoring the current BASEPRI
      // setting because (1) we cannot possibly already be in a critical section and (2) the NVIC won't take
      // interrupts of lower priority than LPTIM even when we set BASEPRI to zero until our ISR ends.
      //
      portDISABLE_INTERRUPTS();
      BaseType_t xWasHigherPriorityTaskWoken = xTaskIncrementTick();
      portENABLE_INTERRUPTS();

      portYIELD_FROM_ISR(xWasHigherPriorityTaskWoken);
   }


   //      Now that we've given as much time as possible for any CMP write to be finished, see if it has
   // finished.  We may have a new value to write after handling CMPM above.  Handling CMPOK last in this ISR
   // isn't very important, but it is a slight optimization over other ordering.
   //
   if (LPTIM->ISR & LPTIM_ISR_CMPOK)
   {
      //      Acknowledge and clear the CMPOK event.
      //
      LPTIM->ICR = LPTIM_ICR_CMPOKCF;

      //      If there is a "pending" write operation to CMP, do it now.  Otherwise, make note that the write
      // is now complete.  Remember to watch for CMP set to 0 when usIdealCmp is 0xFFFF.  There's no pending
      // write in that case.
      //
      if ((uint16_t)(LPTIM->CMP - usIdealCmp) > 1UL)
      {
         LPTIM->CMP = usIdealCmp == 0xFFFF ? 0 : usIdealCmp; // never write 0xFFFF to CMP (HW rule)
         // isCmpWriteInProgress = pdTRUE;  // already true here in the handler for write completed
      }
      else
      {
         isCmpWriteInProgress = pdFALSE;
      }
   }
}

#endif  // configUSE_TICKLESS_IDLE == 2
