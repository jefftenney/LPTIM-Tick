// Ultra Low Power API (ulp.h)
//
// Automatically use STOP modes in FreeRTOS whenever application conditions permit.

void vUlpInit();

//      Define an application-specific set of peripherals of interest to the ULP support code.  The values
// are used as flags in the ULP implementation.  The peripherals of interest are those used by the application
// that cannot operate in STOP 2 mode.
//
#define ulpPERIPHERAL_USART2   (1UL << 0)
#define ulpPERIPHERAL_LPTIM2   (1UL << 1)

//      Identify the subset of the peripherals listed above that can operate in STOP 1 mode.
//
#define ulpPERIPHERALS_OK_IN_STOP1 (ulpPERIPHERAL_LPTIM2)

void vUlpOnPeripheralsActive( int xPeripherals );
void vUlpOnPeripheralsActiveFromISR( int xPeripherals );

void vUlpOnPeripheralsInactive( int xPeripherals );
void vUlpOnPeripheralsInactiveFromISR( int xPeripherals );

void vUlpPreSleepProcessing();

void vUlpPostSleepProcessing();
