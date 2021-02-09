// Test FreeRTOS Tick Timing on STM32L4 (testTickTiming.h)

#ifndef INC_TESTTICKTIMING_H_
#define INC_TESTTICKTIMING_H_

#include "FreeRTOS.h"

//      The testing occurs in back-to-back cycles with each cycle lasting approximately this long.  Maximum
// setting is 1 hour.
//
#define tttCYCLE_DURATION_SECONDS 60 // MAX 3600

void vTttOsTask( void const * argument );

void vApplicationTickHook( void );

typedef struct
{
   uint32_t subsecondsPerSecond;

   int32_t  drift;
   uint32_t duration;

   int16_t  minDriftRatePct;
   int16_t  maxDriftRatePct;

   int resultsCounter;

} TttResults_t;

void vTttGetResults( TttResults_t* lastCompletedCycle, TttResults_t* cycleNowUnderway );

void vTttResync();

void vTttSetEvalInterval( TickType_t interval );

#endif /* INC_TESTTICKTIMING_H_ */
