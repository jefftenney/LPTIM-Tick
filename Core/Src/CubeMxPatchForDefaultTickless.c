// CubeMxPatchForDefaultTickless.c

// When we configure the project for built-in tickless functionality (configUSE_TICKLESS_IDLE set to 1),
// CubeMX 6.2.0-RC3 generates configPRE_SLEEP_PROCESSING() which sets ulExpectedIdleTime to zero and then
// calls this function.  That behavior is wrong.  It tells FreeRTOS that we are providing the WFI (or WFE)
// instruction (which we shouldn't have to do), and it hides from this function the amount of time we
// expect to stay in tickless idle.  And of course CubeMX generates these definitions at the end of
// FreeRTOSConfig.h, where there are no user-code sections left for us to undo the damage.
//
// So this function fixes the problem.  Ideally CubeMX will fix the issue and we can delete this file.

#include "FreeRTOS.h"
#include "task.h" // for configASSERT(), which uses taskDISABLE_INTERRUPTS()
#include "stm32l4xx.h"

// Bug fix only.  Applies only to configUSE_TICKLESS_IDLE == 1.  See above.
void PreSleepProcessing(uint32_t ulExpectedIdleTime)
{
   //      When this assertion fails, ST has probably fixed CubeMX, and we can probably remove this file from
   // the project.
   //
   configASSERT(ulExpectedIdleTime == 0);

   __DSB();
   __WFI();
   __ISB();
}
