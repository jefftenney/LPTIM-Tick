// Test FreeRTOS Tick Timing on STM32L4 (testTickTiming.c)

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

   int evalPeriodsMissed;
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

volatile rtcSnapshotT tickHookSnapshot;

static void analyzeTickTiming( const timeStampT* ts, const timeStampT* refTs );
static void ingestTimePair( const rtcSnapshotT* rtcTime, TickType_t tickCount );
static void updateResults( TttResults_t* results );
static void syncTo( const timeStampT* ts );

#if ( configUSE_TICK_HOOK != 1 )
#error Symbol configUSE_TICK_HOOK must be 1 to use the tick-timing test.
#endif

void vApplicationTickHook( void )
{
   do
   {
      tickHookSnapshot.SSR = RTC->SSR;
      tickHookSnapshot.TR = RTC->TR;
   } while (tickHookSnapshot.SSR != RTC->SSR);
}

void vTttOsTask( void const * argument )
{
   config.hrtc = (RTC_HandleTypeDef*) argument;
   config.subsecondsPerSecond = ( RTC->PRER & RTC_PRER_PREDIV_S ) + 1;

   taskDISABLE_INTERRUPTS();
   DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_RTC_STOP;
   taskENABLE_INTERRUPTS();

   //      Coming out of stop mode, the RTC shadow registers won't be up-to-date.  So when we read the RTC,
   // bypass the shadow registers.  We manually ensure coherency between the various RTC fields.
   //
   HAL_RTCEx_EnableBypassShadow(config.hrtc);

   if (config.evalInterval == 0)
   {
      config.evalInterval = pdMS_TO_TICKS(10);
   }

   rtcSnapshotT rtcTimeAtTick;
   TickType_t prevWakeTime = xTaskGetTickCount();

   while (1)
   {
      vTaskDelayUntil(&prevWakeTime, config.evalInterval);

      //      Capture the most-recent RTC snapshot before another tick comes along and overwrites it.
      //
      rtcTimeAtTick = tickHookSnapshot; // structure copy

      //      If we captured the snapshot before we lost it, process it.  Otherwise, count the missed evaluation.
      //
      if (xTaskGetTickCount() == prevWakeTime)
      {
         ingestTimePair( &rtcTimeAtTick, prevWakeTime );
      }
      else
      {
         testState.evalPeriodsMissed += 1;
      }
   }
}

void vTttGetResults( TttResults_t* lastCompletedCycle, TttResults_t* cycleNowUnderway )
{
   cycleNowUnderway->resultsCounter = 0;

   taskENTER_CRITICAL();
   *lastCompletedCycle = prevResults; // structure copy
   updateResults( cycleNowUnderway );
   taskEXIT_CRITICAL();
}

void vTttResync()
{
   testState.syncTime.tickCount = 0;
}

void vTttSetEvalInterval( TickType_t interval )
{
   if (interval > (TickType_t) 1)
   {
      config.evalInterval = interval;
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

   timeStampT refTs = testState.syncTime; // structure copy
   if (refTs.tickCount == 0)
   {
      syncTo(&currentTs);
   }
   else
   {
      analyzeTickTiming(&currentTs, &refTs);

      if (testState.totalTestDuration >= tttCYCLE_DURATION_SECONDS * config.subsecondsPerSecond)
      {
         // Save "final" results someplace, and execute the callback function (if any).
         //
         updateResults( &prevResults );
         prevResults.resultsCounter += 1;

         // Start the next test cycle coherently with the end of the previous cycle.
         //
         syncTo(&currentTs);
      }
   }
}

static void syncTo( const timeStampT* ts )
{
   testState.syncTime = *ts; // structure copy
   testState.maxDriftRatePct = -100;
   testState.minDriftRatePct = +100;
   testState.evalPeriodsMissed = 0;
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
   results->drift = testState.totalDrift;
   results->duration = testState.totalTestDuration;
   results->maxDriftRatePct = testState.maxDriftRatePct;
   results->minDriftRatePct = testState.minDriftRatePct;
   results->subsecondsPerSecond = config.subsecondsPerSecond;
}
