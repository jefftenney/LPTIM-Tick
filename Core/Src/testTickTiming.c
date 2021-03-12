// Test FreeRTOS Tick Timing on STM32L4 (testTickTiming.c)
//
// o Generates a new TttResults_t every tttTEST_DURATION_SECONDS unless disabled.
// o Disable with vTttSetEvalInterval( portMAX_DELAY );
// o Task starts in the disable state.
// o Enable with vTttSetEvalInterval( xIntervalTicks ); where 2 <= xIntervalTicks <= configTICK_RATE_HZ

#include "testTickTiming.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32l4xx_hal.h"

TttResults_t prevResults;

typedef struct
{
   uint32_t rtcTotalSeconds;
   uint32_t rtcSubseconds;
   TickType_t tickCount;

} timeStampT;

static struct
{
   timeStampT syncTime;

   uint32_t totalTestDuration;
   int32_t  totalDrift;
   int16_t  minDriftRatePct;
   int16_t  maxDriftRatePct;

} testState;

static struct
{
   RTC_HandleTypeDef* hrtc;
   TickType_t evalInterval;
   uint32_t subsecondsPerSecond;

} config;

typedef struct
{
   uint32_t TR;
   uint32_t SSR;

} rtcSnapshotT;

static TaskHandle_t xTttOsTaskHandle;
static volatile TickType_t xRequestedEvalInterval = portMAX_DELAY;
static volatile rtcSnapshotT tickHookSnapshot;

static void analyzeTickTiming( const timeStampT* ts, const timeStampT* refTs );
static void ingestTimePair( const rtcSnapshotT* rtcTime, TickType_t tickCount );
static void updateResults( TttResults_t* results );
static void syncTo( const timeStampT* ts );

#if ( configUSE_TICK_HOOK != 1 )
#error Symbol configUSE_TICK_HOOK must be 1 to use the tick-timing test.
#endif

void vApplicationTickHook( void )
{
   //      FreeRTOS calls vApplicationTickHook() from the tick interrupt handler, so this a great time to
   // capture the current RTC time including subseconds.  Code in the tick-timing task analyzes these captures
   // occasionally to evaluate tick-timing accuracy.
   //
   //      Be careful to capture a coherent time from the RTC.  The RTC provides shadow registers to alleviate
   // the need for this kind of careful coding, but we don't use the shadow registers.  They are out of sync
   // after STOP mode, and we don't want to wait around for them to sync up.
   //
   do
   {
      tickHookSnapshot.SSR = RTC->SSR;
      tickHookSnapshot.TR = RTC->TR;
   } while (tickHookSnapshot.SSR != RTC->SSR);
}

void vTttOsTask( void const * argument )
{
   //      Save a copy of our task handle so our API functions can use task notifications easily.
   //
   xTttOsTaskHandle = xTaskGetCurrentTaskHandle();

   //      Save the RTC handle and relevant RTC configuration info to our configuration structure.
   //
   config.hrtc = (RTC_HandleTypeDef*) argument;
   config.subsecondsPerSecond = ( RTC->PRER & RTC_PRER_PREDIV_S ) + 1;

   //      Verify bare minimum configuration of the RTC to provide timing resolution at least 4x the tick
   // frequency.  This software would work correctly with less resolution, but the testing wouldn't be as
   // useful.  If this assertion fails, decrease PREDIV_A and increase PREDIV_S in your RTC configuration.
   //
   configASSERT(config.subsecondsPerSecond / configTICK_RATE_HZ >= 4UL)

   //      Be sure the RTC ignores its input clock when the debugger stops program execution.
   //
   taskDISABLE_INTERRUPTS();
   DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_RTC_STOP;
   taskENABLE_INTERRUPTS();

   //      Coming out of stop mode, the RTC shadow registers aren't up-to-date.  So when we read the RTC,
   // bypass the shadow registers.  We manually ensure coherency between the various RTC fields.
   //
   HAL_RTCEx_EnableBypassShadow(config.hrtc);

   //      Initialize the notification state for our task to notified so we'll set config.evalInterval.
   //
   xTaskNotifyGive(xTttOsTaskHandle);

   while (1)
   {
      if (ulTaskNotifyTake(pdTRUE, config.evalInterval) == pdFAIL)
      {
         //      Capture an RTC snapshot and its matching tick count.  Remember that tickHookSnapshot changes
         // asynchronously, in the tick ISR.
         //
         TickType_t xTickCount;
         rtcSnapshotT rtcTimeAtTick;
         do
         {
            xTickCount = xTaskGetTickCount();
            rtcTimeAtTick = tickHookSnapshot; // structure copy

         } while (xTickCount != xTaskGetTickCount());

         //      Process the snapshot and tick count.
         //
         ingestTimePair( &rtcTimeAtTick, xTickCount );
      }
      else
      {
         //      Set the new evaluation interval, and arrange for the next evaluation to be the start of a
         // new test run (tttTEST_DURATION_SECONDS).
         //
         config.evalInterval = xRequestedEvalInterval;
         testState.syncTime.tickCount = 0;
      }
   }
}

void vTttGetResults( TttResults_t* lastCompletedRun, TttResults_t* runNowUnderway )
{
   //      The "results" for the run now underway are incomplete, so set the results counter to zero for the
   // caller's benefit.
   //
   runNowUnderway->resultsCounter = 0;

   //      Use a critical section to retrieve test results.  This function executes in a task context other
   // than the tick-timing task context, and we don't want the results to change in the middle of our
   // retrieving them.
   //
   taskENTER_CRITICAL();
   *lastCompletedRun = prevResults; // structure copy
   updateResults( runNowUnderway );
   taskEXIT_CRITICAL();
}


// 2 <= interval <= configTICK_RATE_HZ or portMAX_DELAY
void vTttSetEvalInterval( TickType_t interval )
{
   if (interval > (TickType_t) 1)
   {
      xRequestedEvalInterval = interval;

      if (xTttOsTaskHandle != NULL)
      {
         xTaskNotifyGive(xTttOsTaskHandle);
      }
   }
}

#define SECONDS_PER_MINUTE ( 60UL )
#define SECONDS_PER_HOUR   ( SECONDS_PER_MINUTE * 60UL )
#define SECONDS_PER_DAY    ( SECONDS_PER_HOUR * 24UL )

static void ingestTimePair( const rtcSnapshotT* rtcTime, TickType_t tickCount )
{
   uint32_t hours, minutes, seconds;
   hours   = RTC_Bcd2ToByte( (rtcTime->TR & (RTC_TR_HT  | RTC_TR_HU )) >> RTC_TR_HU_Pos );
   minutes = RTC_Bcd2ToByte( (rtcTime->TR & (RTC_TR_MNT | RTC_TR_MNU)) >> RTC_TR_MNU_Pos );
   seconds = RTC_Bcd2ToByte( (rtcTime->TR & (RTC_TR_ST  | RTC_TR_SU )) >> RTC_TR_SU_Pos );

   timeStampT currentTs;
   currentTs.rtcTotalSeconds = (hours * SECONDS_PER_HOUR) + (minutes * SECONDS_PER_MINUTE) + seconds;
   currentTs.rtcSubseconds = config.subsecondsPerSecond - rtcTime->SSR - 1;
   currentTs.tickCount = tickCount;

   if (testState.syncTime.tickCount == 0)
   {
      syncTo(&currentTs);
   }
   else
   {
      analyzeTickTiming(&currentTs, &testState.syncTime);

      if (testState.totalTestDuration >= tttTEST_DURATION_SECONDS * config.subsecondsPerSecond)
      {
         //      Save the "final" results.  Use a critical section to prevent another task from reading the
         // test results while we're in the middle of updating them.  See vTttGetResults().
         //
         taskENTER_CRITICAL();
         updateResults( &prevResults );
         prevResults.resultsCounter += 1;
         taskEXIT_CRITICAL();

         //      Execute the callback function (if any).
         //
         #if configUSE_TICK_TEST_COMPLETE_HOOK != 0
         {
            vApplicationTickTestComplete();
         }
         #endif

         // Start the next test run coherently with the end of the previous run.
         //
         syncTo(&currentTs);
      }
   }
}

static void syncTo( const timeStampT* ts )
{
   //      Set a new synchronization time, and reset the test state to the beginning of a new test run.
   //
   testState.syncTime = *ts; // structure copy
   testState.maxDriftRatePct = -100;
   testState.minDriftRatePct = +100;
   testState.totalDrift = 0;
   testState.totalTestDuration = 0;
}

static void analyzeTickTiming( const timeStampT* ts, const timeStampT* refTs )
{
   uint32_t elapsedTicks = ts->tickCount - refTs->tickCount;
   uint32_t elapsedTicksAsSubseconds = ( (uint64_t)elapsedTicks * config.subsecondsPerSecond ) / configTICK_RATE_HZ;

   int32_t elapsedRtcSeconds = ( ts->rtcTotalSeconds - refTs->rtcTotalSeconds );
   if (elapsedRtcSeconds < 0)
   {
      elapsedRtcSeconds += SECONDS_PER_DAY;
   }

   int32_t elapsedRtcSubseconds = ( ts->rtcSubseconds - refTs->rtcSubseconds );
   if (elapsedRtcSubseconds < 0)
   {
      elapsedRtcSubseconds += config.subsecondsPerSecond;
      elapsedRtcSeconds -= 1;
   }

   uint32_t elapsedRtcTimeAsSubseconds = ( elapsedRtcSeconds * config.subsecondsPerSecond ) + elapsedRtcSubseconds;
   int32_t totalDrift = elapsedTicksAsSubseconds - elapsedRtcTimeAsSubseconds;

   int32_t driftDelta  = totalDrift - testState.totalDrift;
   int32_t driftPeriod = elapsedRtcTimeAsSubseconds - testState.totalTestDuration;

   int driftRatePct;
   if (driftDelta < 0 && -driftDelta > driftPeriod)
   {
      driftRatePct = -100;
   }
   else if (driftDelta > 0 && driftDelta > driftPeriod)
   {
      driftRatePct = 100;
   }
   else
   {
      driftRatePct = ( driftDelta * 100 ) / driftPeriod;
   }

   if (driftRatePct < testState.minDriftRatePct)
   {
      testState.minDriftRatePct = driftRatePct;
   }
   if (driftRatePct > testState.maxDriftRatePct)
   {
      testState.maxDriftRatePct = driftRatePct;
   }
   testState.totalDrift = totalDrift;
   testState.totalTestDuration = elapsedRtcTimeAsSubseconds;
}

static void updateResults( TttResults_t* results )
{
   results->driftSs = testState.totalDrift;
   results->durationSs = testState.totalTestDuration;
   results->maxDriftRatePct = testState.maxDriftRatePct;
   results->minDriftRatePct = testState.minDriftRatePct;
   results->subsecondsPerSecond = config.subsecondsPerSecond;
}
