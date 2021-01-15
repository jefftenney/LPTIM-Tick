# LPTIM-Tick
*FreeRTOS Tick/Tickless via LPTIM*

Use LPTIM for the FreeRTOS tick instead of the SysTick Timer for ultra-low-power applications.

- No drift or slippage in kernel time
- Use STOP modes even while FreeRTOS timers are running or delays are underway
- For any STM32 with LPTIM (most STM32L, some STM32F, STM32G, STM32H, STM32W)

This repository is intended to demonstrate integration and testing of the [lptimTick.c gist](https://gist.github.com/jefftenney/02b313fe649a14b4c75237f925872d72).  The project is made for STM32CubeIDE.
