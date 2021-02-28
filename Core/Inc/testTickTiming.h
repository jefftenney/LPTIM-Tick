// Test FreeRTOS Tick Timing on STM32L4 (testTickTiming.h)
//
// o Generates a new TttResults_t every tttTEST_DURATION_SECONDS unless disabled.
// o Disable with vTttSetEvalInterval( portMAX_DELAY );
// o Task starts in the disable state.
// o Enable with vTttSetEvalInterval( xIntervalTicks ); where 2 <= xIntervalTicks <= configTICK_RATE_HZ

#ifndef INC_TESTTICKTIMING_H_
#define INC_TESTTICKTIMING_H_

#include "FreeRTOS.h"

//      The testing occurs in back-to-back runs with each run lasting approximately this long.  Maximum
// setting is 1 hour.
//
#define tttTEST_DURATION_SECONDS 60 // MAX 3600

void vTttOsTask( void const * argument );

void vApplicationTickHook( void );

#if configUSE_TICK_TEST_COMPLETE_HOOK != 0
extern void vApplicationTickTestComplete();
#endif

typedef struct
{
   uint32_t subsecondsPerSecond;

   int32_t  drift;
   uint32_t duration;

   int16_t  minDriftRatePct;
   int16_t  maxDriftRatePct;

   int resultsCounter;

} TttResults_t;

void vTttGetResults( TttResults_t* lastCompletedRun, TttResults_t* runNowUnderway );

// To enable tick-timing evaluation, set interval: 2 <= interval <= configTICK_RATE_HZ
// To disable tick-timing evaluation, set interval = portMAX_DELAY.
//
void vTttSetEvalInterval( TickType_t interval );

#endif /* INC_TESTTICKTIMING_H_ */
