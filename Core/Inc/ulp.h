// Ultra Low Power API (ulp.h)
//
// Automatically use STOP modes in FreeRTOS whenever application conditions permit.

//      Symbol configMIN_RUN_BETWEEN_DEEP_SLEEPS, optionally defined in FreeRTOSConfig.h, enables a workaround
// for erratum 2.3.21, which affects some STM32 devices (like the 'L476).  If the STM32 used in this project
// is not affected by this erratum, leave symbol configMIN_RUN_BETWEEN_DEEP_SLEEPS undefined.
//
//      To enable this workaround, define configMIN_RUN_BETWEEN_DEEP_SLEEPS in FreeRTOSConfig.h.  The value of
// this symbol is a number of core clock cycles, and it depends on Vdd and on the core-clock speed.  See the
// errata sheet for details.  If this value changes at run time, call vUlpInit() after each change.
//
// *** Note that this workaround uses the SysTick timer. ***
//
// Example (in FreeRTOSConfig.h):
//
// #define configMIN_RUN_BETWEEN_DEEP_SLEEPS  ( (13U * configCPU_CLOCK_HZ) / 1000000 )  // 13us for 1.8V VDD

void vUlpInit();

//      Define an application-specific set of peripherals of interest to the ULP support code.  The values
// are used as flags in the ULP implementation.  The peripherals of interest are those used by the application
// that cannot operate in STOP 2 mode.
//
#define ulpPERIPHERAL_USART2     (1UL << 0)
#define ulpPERIPHERAL_LPTIM2     (1UL << 1)
#define ulpPERIPHERAL_MIN_RUN    (1UL << 2)

//      Identify the subset of the peripherals listed above that can operate in STOP 1 mode.
//
#define ulpPERIPHERALS_OK_IN_STOP1 (ulpPERIPHERAL_LPTIM2)

void vUlpOnPeripheralsActive( int xPeripherals );
void vUlpOnPeripheralsActiveFromISR( int xPeripherals );

void vUlpOnPeripheralsInactive( int xPeripherals );
void vUlpOnPeripheralsInactiveFromISR( int xPeripherals );

void vUlpPreSleepProcessing();

void vUlpPostSleepProcessing();
