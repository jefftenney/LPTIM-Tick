#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

#include "stm32l4xx.h"
#include "stm32l4xx_hal.h"

// Jeff:
#define dbg_util_event_log(x) ((void)0)

/* Frequency of clock feeding LPTIM1
 * This cannot be asserted statically since it depend on RCC configuration */
#define LPTIM1_CLK_FRQ 32768

/* This is the maximum value that the LPTIM1 CNT register can hold. */
#define LPTIM1_CNT_MAXVAL UINT16_MAX

/* How much we're dividing the input clock before feeding it to LPTIM1 */
#define LPTIM1_PRESCALER_DIV LPTIM_PRESCALER_DIV2
#define LPTIM1_PRESCALER (1 << (LPTIM1_PRESCALER_DIV >> LPTIM_CFGR_PRESC_Pos))

/* Frequency at which LPTIM1's CNT register is incremented by 1 */
#define LPTIM1_INC_FREQ (LPTIM1_CLK_FRQ / LPTIM1_PRESCALER)
static_assert(LPTIM1_INC_FREQ == 16384, "");

/* How many LPTIM1 CNT register to use as 1 tick */
#define LPTIM1_CNTS_PER_TICK (LPTIM1_INC_FREQ / configTICK_RATE_HZ)
static_assert(LPTIM1_CNTS_PER_TICK == 16, "");

/* Longest LPTIM1 delay before overflow */
#define LPTIM1_MAX_DELAY_SECONDS ((UINT16_MAX + 1) / LPTIM1_INC_FREQ)
static_assert(LPTIM1_MAX_DELAY_SECONDS == 4, "");

/* How many ticks the system can sleep before CNT wraps around to 0 */
#define LPTIM1_MAX_SUPPRESSED_TICKS ((UINT16_MAX + 1) / LPTIM1_CNTS_PER_TICK)

/* How many CNTs it takes to load a new value into LPTIMx's register
 * See page 14 of AN4865 for more detail
 * CAUTION: This value was derived empirically and by increasing it by one
 * until the tick frequency was correct in low power run mode, then 1 was
 * added for safety */
#define LPTIM_REG_LOAD_DLY 3

#define MIN(A,B) (A < B ? A : B)
#define MAX(A,B) (A > B ? A : B)


static LPTIM_HandleTypeDef lptim_handle;

static inline uint32_t lptim_read_cnt(LPTIM_HandleTypeDef * t);

static inline bool lptim_should_load_arr(uint32_t cnt_current,
                                         uint32_t arr_current,
                                         uint32_t arr_new);

static inline uint32_t lptim_calculate_arr_for_next_n_ticks(uint32_t cnt,
                                                            uint32_t n_ticks);

static inline uint32_t lptim_calculate_tick_from_cnt(uint32_t cnt);

static inline void lptim_load_arr_from_init(LPTIM_HandleTypeDef * t,
                                            uint32_t              arr);
static inline void lptim_load_arr(LPTIM_HandleTypeDef * t,
                                  uint32_t              arr);


/* Uncomment to enable debug hooks */
//#define USE_DEBUG_HOOKS
#ifdef USE_DEBUG_HOOKS

#include "sensor_int.h"
#define INIT_DEBUG_PIN(PIN) sensor_int_configure_debug_out((enum sensor_int_pin)PIN, false)
#define DEBUG_PIN(PIN, STATE) sensor_int_set_debug_out((enum sensor_int_pin)PIN, STATE)
#define CLEAR_DEBUG()                                   \
    sensor_int_set_debug_out(SENSOR_INT_0, false);      \
    sensor_int_set_debug_out(SENSOR_INT_1, false)

#define hook_TIMER_SETUP()                      \
    do                                          \
    {                                           \
    INIT_DEBUG_PIN(SENSOR_INT_0);               \
    INIT_DEBUG_PIN(SENSOR_INT_1);               \
    CLEAR_DEBUG();                              \
} while (0)

#define hook_IRQ_START()                        \
      do                                        \
      {                                         \
          DEBUG_PIN(SENSOR_INT_0, true);        \
      } while (0)

#define hook_IRQ_FINISH()                       \
      do                                        \
      {                                         \
          DEBUG_PIN(SENSOR_INT_0, false);       \
      } while (0)

#define hook_PRE_SLEEP()                        \
    do                                          \
    {                                           \
    } while (0)

#define hook_POST_SLEEP()                       \
    do                                          \
    {                                           \
    } while (0)

#define hook_PRE_ARR()                          \
    do                                          \
    {                                           \
        DEBUG_PIN(SENSOR_INT_1, true);          \
    } while (0)

#define hook_POST_ARR()                         \
    do                                          \
    {                                           \
        DEBUG_PIN(SENSOR_INT_1, false);         \
    } while (0)


#else

#define hook_TIMER_SETUP() do {} while (0)
#define hook_IRQ_START()   do {} while (0)
#define hook_IRQ_FINISH()  do {} while (0)
#define hook_PRE_SLEEP()   do {} while (0)
#define hook_POST_SLEEP()  do {} while (0)
#define hook_PRE_ARR()     do {} while (0)
#define hook_POST_ARR()    do {} while (0)

#endif

/** LPTIM1 Init
 *
 * Initializes LPTIM1 to act as a 1 ms(ish) tick timer */
void system_config_lptim1_init(void)
{
    // Jeff - added this block:
    RCC->APB1ENR1 |= RCC_APB1ENR1_LPTIM1EN;
    MODIFY_REG(RCC->CCIPR, RCC_CCIPR_LPTIM1SEL, 3 << RCC_CCIPR_LPTIM1SEL_Pos);
    DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_LPTIM1_STOP;


    lptim_handle = (LPTIM_HandleTypeDef){
        .Instance = LPTIM1,
        .Init =
            {
                .Clock =
                    {
                        /* Clock LPTIM1 from whatever RCC is feeding it */
                        .Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC,
                        /* Divide input clock (32768 Hz oscillator) by N for an
                         * increment rate of 32768/N Hz */
                        .Prescaler = LPTIM1_PRESCALER_DIV,
                    },
                .Trigger =
                    {
                        /* Control (start) the timer trigger from software */
                        .Source = LPTIM_TRIGSOURCE_SOFTWARE,
                    },
                /* Immediately use new autoreload value when it's changed */
                .UpdateMode = LPTIM_UPDATE_IMMEDIATE,
                /* Increment timer with every clock tick */
                .CounterSource = LPTIM_COUNTERSOURCE_INTERNAL,
            },
    };

    /* Initialize timer */
    HAL_StatusTypeDef const stat = HAL_LPTIM_Init(&lptim_handle);
    assert(stat == HAL_OK);

    /* Enable global LPTIM1 interrupt */
    NVIC_SetPriority(LPTIM1_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY); // Jeff - was SYSTEM_CONFIG_IRQ_PRIORITY_LPTIM1
    NVIC_EnableIRQ(LPTIM1_IRQn);

    assert(0 == lptim_read_cnt(&lptim_handle));

    /* Enable LPTIM1 compare match interrupt
     *
     * CAUTION: Can only be set before LPTIMx's ENABLE bit is set to 1 */
    __HAL_LPTIM_ENABLE_IT(&lptim_handle, LPTIM_IT_ARRM);

    /* Enable LPTIM1 */
    __HAL_LPTIM_ENABLE(&lptim_handle);

    /* Set autoreload value to trigger an interrupt at the next whole tick
     * boundary
     *
     * CAUTION: Can only be set after LPTIMx's ENABLE bit is set to 1 */
    lptim_load_arr_from_init(&lptim_handle, LPTIM1_CNTS_PER_TICK - 1);

    /* Start LPTIM1
     *
     * CAUTION: Can only be set after LPTIMx's ENABLE bit is set to 1 */
    __HAL_LPTIM_START_CONTINUOUS(&lptim_handle);
}


void LPTIM1_IRQHandler(void)
{
    dbg_util_event_log(dbg_lptim1_isr_start);

    /* Make sure that LPTIM's interrupts are configured correctly */
    assert(__HAL_LPTIM_GET_FLAG(&lptim_handle, LPTIM_FLAG_ARRM));
    assert(__HAL_LPTIM_GET_IT_SOURCE(&lptim_handle, LPTIM_FLAG_ARRM));

    /* Clear the interrupt */
    __HAL_LPTIM_CLEAR_FLAG(&lptim_handle, LPTIM_FLAG_ARRM);

    /* The SysTick runs at the lowest interrupt priority, so when this interrupt
     * executes all interrupts must be unmasked. There is therefore no need to
     * save and then restore the interrupt mask value as its value is already
     * known. */
    (void)portSET_INTERRUPT_MASK_FROM_ISR();
    {
        hook_IRQ_START();

        /* Increment the RTOS tick. */
        if (xTaskIncrementTick() != pdFALSE)
        {
            /* A context switch is required.  Context switching is performed in
            the PendSV interrupt.  Pend the PendSV interrupt. */
            portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;
        }

        hook_IRQ_FINISH();
    }
    portCLEAR_INTERRUPT_MASK_FROM_ISR(0);

    dbg_util_event_log(dbg_lptim1_isr_end);
}

/** Returns true only when there are enough CNTs left before the next tick based
 * on current ARR and the next desired tick based on the new ARR
 */
static inline bool lptim_should_load_arr(uint32_t cnt_current,
                                         uint32_t arr_current,
                                         uint32_t arr_new)
{
    assert(((arr_current + 1) % LPTIM1_CNTS_PER_TICK) == 0);
    assert(((arr_new + 1) % LPTIM1_CNTS_PER_TICK) == 0);

    /* Should not load ARR if the current and new ARRs match */

    /* Check that are at least LPTIM_REG_LOAD_DLY clock-cycles (CNTs) until the
     * current ARR will trigger an ARRM interrupt */
    if (arr_current - cnt_current <= LPTIM_REG_LOAD_DLY)
    {
        return false;
    }

    /* Check that are at least LPTIM_REG_LOAD_DLY clock-cycles (CNTs) until the
     * new ARR will trigger an ARRM interrupt */
    if (arr_new - cnt_current <= LPTIM_REG_LOAD_DLY)
    {
        return false;
    }

    return true;
}


/** Given an LPTIM count, returns the  autoreload (ARR) required to fire an
 * autoreload match (ARRM) interrupt `n_ticks` from now */
static inline uint32_t lptim_calculate_arr_for_next_n_ticks(uint32_t cnt,
                                                            uint32_t n_ticks)
{
    /* Must be at least 1 tick */
    assert(n_ticks > 0);

    /* If we try to delay more than this, we will get a bogus high value in arr
     * (cnt + rem_cnts + ...) >= 2^33 that could underflow (when == 2^33) when
     * we -= 1 at the end. So instead we return the max value that LPTIM1 can
     * count to, because that is always going to be sooner than this very long
     * delay. */
    const uint32_t tick_limit = UINT16_MAX / LPTIM1_CNTS_PER_TICK;
    if (n_ticks > tick_limit) {
        return LPTIM1_CNT_MAXVAL;
    }

    /* Calculate how counts are left until LPTIM1->CNT is a multiple of
     * LPTIM1_CNTS_PER_TICK */
    const uint32_t rem_cnts =
        LPTIM1_CNTS_PER_TICK - (cnt % LPTIM1_CNTS_PER_TICK);

    /* Calculate ARR
     * Adding rem_cnts to cnt takes us to the next tick boundry, so we need to
     * subtract one tick from n_ticks  */
    uint32_t arr = cnt + rem_cnts + ((n_ticks - 1) * LPTIM1_CNTS_PER_TICK);

    /* Clamp arr to the maximum possible ARR */
    if (arr > LPTIM1_CNT_MAXVAL + 1)
    {
        arr = LPTIM1_CNT_MAXVAL + 1;
    }
    assert(arr % LPTIM1_CNTS_PER_TICK == 0);

    /* The ARRM interrupt fires 1 LPTIM clock-cycle after LPTIMx->ARRM ==
     * LPTIMx->CNT. We must subtract one from our calculated ARR to avoid tick
     * drift */
    arr -= 1;

    return arr;
}

/** Returns which tick boundry a timer count belongs to
 *
 * The returned tick boundry is not absolute since LPTIM's CNT wraps back around
 * to 0 on every autoreload match.
 */
static inline uint32_t lptim_calculate_tick_from_cnt(uint32_t cnt)
{
    return cnt / LPTIM1_CNTS_PER_TICK;
}

static inline bool lptim_arr_is_stable(LPTIM_HandleTypeDef * t)
{
    return __HAL_LPTIM_GET_FLAG(t, LPTIM_FLAG_ARROK);
}

/** Load a new autoreload value into LPTIMx->ARR
 *
 *  The ARROK flag must be true.
 */
static inline void lptim_load_arr(LPTIM_HandleTypeDef * t,
                                  uint32_t              arr)
{
    /* ARR only uses 16 bits out of its 32 bit register */
    assert(arr <= UINT16_MAX);

    /* The ARROK flag must be false before setting a new value.
     * Otherwise, it's undefined behavior. */
    assert(lptim_arr_is_stable(t));

    /* Only load the ARR if the requested ARR is different. */
    if (t->Instance->ARR != arr)
    {
        hook_PRE_ARR();

        /* Clear the flag */
        __HAL_LPTIM_CLEAR_FLAG(t, LPTIM_FLAG_ARROK);

        /* Load the new autoreload value */
        __HAL_LPTIM_AUTORELOAD_SET(t, arr);

        hook_POST_ARR();
    }
}

static inline void lptim_load_arr_from_init(LPTIM_HandleTypeDef * t,
                                            uint32_t              arr)
{
    /* ARR only uses 16 bits out of its 32 bit register */
    assert(arr <= UINT16_MAX);

    /* There's one case where we expect the ARROK flag to be false
     * when setting it. That's from the initialization routine. Here
     * we'll assert that it's not OK (which indicates it's never been
     * set.) */
    assert(!lptim_arr_is_stable(t));

    /* Only load the ARR if the requested ARR is different. */
    if (t->Instance->ARR != arr)
    {
        /* Clear the flag */
        __HAL_LPTIM_CLEAR_FLAG(t, LPTIM_FLAG_ARROK);

        /* Load the new autoreload value */
        __HAL_LPTIM_AUTORELOAD_SET(t, arr);
    }
}

/** Read the CNT value until it's stable.
 *
 * CNT must return the same result twice in a row before we can assume
 * the value is stable.
 */
static inline uint32_t lptim_read_cnt(LPTIM_HandleTypeDef * t)
{
    uint32_t cnt0;
    uint32_t cnt1;

    do
    {
        cnt0 = t->Instance->CNT;
        cnt1 = t->Instance->CNT;
    } while (cnt1 != cnt0);

    return cnt1;
}

/** Read the ARR value.
 *
 * Convenience function to access the ARR value of a LPTIM.
 */
static inline uint32_t lptim_read_arr(LPTIM_HandleTypeDef * t)
{
    return t->Instance->ARR;
}

/*
 * FreeRTOS specific code
 */

/* Use custom tick source and tickless idle when true */
#if configUSE_TICKLESS_IDLE == 2

/** Start LPTIM1
 *
 * Called from FreeRTOS so it doesn't start receiving ticks before it's
 * ready.
 */
void vPortSetupTimerInterrupt(void)
{
    hook_TIMER_SETUP();
    system_config_lptim1_init();
}

extern bool system_config_power_sleep_permitted(void);

void vPortSuppressTicksAndSleep(TickType_t expected_idle_ticks)
{
    /* If the ARR isn't yet stable, return immediately. We'll spend
     * more time in the Idle task until the ARR is ready. */
    if (!lptim_arr_is_stable(&lptim_handle))
    {
        return;
    }

    /* Read the current ARR out of LPTIM  */
    const uint32_t entry_arr = lptim_read_arr(&lptim_handle);

    /* Read the current count value out of LPTIM and calculate tick boundary */
    const uint32_t pre_sleep_cnt = lptim_read_cnt(&lptim_handle);
    const uint32_t pre_sleep_tick =
        lptim_calculate_tick_from_cnt(pre_sleep_cnt);

    /* Calculate the ARR required to wait expected_idle_ticks tick periods */

    /* Enter a critical section but don't use the taskENTER_CRITICAL() method as
     * that will mask interrupts that should exit sleep mode. */
    __asm volatile("cpsid i");

    /* NOTE: system_config_power_sleep_permitted must be called from
     * inside a critical section. */
#if 0 // Jeff - exclude block
    if (!system_config_power_sleep_permitted()) {
        /* If we're not going to be going to sleep, we need to make
         * sure that the LPTIM ISR fires at least as often as we need
         * the watchdog pet. However, due to "difficulties" of the hardware,
         * we can sometimes get woken up from a sleep by an ISR and not be
         * able to reset the LPTIM ISR to go off at a regular period - see the
         * `dbg_lptim_complete_unstable_arr` log message below. By making sure
         * we only idle for 1 tick, we're making sure that the complete with
         * unstable arr case is impossible.
         * We only reduce sleeps to 1 tick when a power consumer is active
         * because we assume that is the only time we will wake due to
         * interrupts while ARR is still unstable. However, in deep sleep, it
         * is still possible for a sensor int wake to cause this bug. We don't
         * have a workaround for that for now.
         */
        expected_idle_ticks = MIN(1, expected_idle_ticks);
    }
#endif

    const uint32_t arr_for_sleep = lptim_calculate_arr_for_next_n_ticks(
        pre_sleep_cnt, expected_idle_ticks);

    /* If a context switch is pending or a task is waiting for the scheduler to
     * be unsuspended then abandon the low power entry. */
    if (eTaskConfirmSleepModeStatus() == eAbortSleep)
    {
        /* Re-enable interrupts - see comments above the cpsid instruction */
        __asm volatile("cpsie i");
    }
    else
    {
        /* Set new LPTIM1 autoreload value */
        if (lptim_should_load_arr(pre_sleep_cnt, entry_arr, arr_for_sleep))
        {
            lptim_load_arr(&lptim_handle, arr_for_sleep);
        }
        else
        {
            /* Exit early if there is not enough time to load `arr_for_sleep`
             * before the next tick will fire.
             *
             * If we do not exit early the following scenario can happen:
             * 1. Something other than LPTIM wakes the CPU
             * 2. The tick fires and LPTIM1 wraps around _after_ checking what
             *    woke us up
             * 3. `assert(post_sleep_cnt >= pre_sleep_cnt)` in the 'woke by
             *    other than LPTIM' branch fails
             */

            /* Re-enable interrupts and return without sleeping */
            __asm volatile("cpsie i");
            return;
        }

        /* Sleep until something happens.  configPRE_SLEEP_PROCESSING() can set
         * its parameter to 0 to indicate that its implementation contains its
         * own wait for interrupt or wait for event instruction, and so wfi
         * should not be executed again.  However, the original expected idle
         * time variable must remain unmodified, so a copy is taken. */
        uint32_t modifiable_idle_ticks = expected_idle_ticks;
        hook_PRE_SLEEP();
        configPRE_SLEEP_PROCESSING(&modifiable_idle_ticks);
        if (modifiable_idle_ticks > 0)
        {
            __asm volatile("dsb");
            __asm volatile("wfi");
            __asm volatile("isb");
        }
        configPOST_SLEEP_PROCESSING(&modifiable_idle_ticks);
        hook_POST_SLEEP();

        /* Immediately save LPTIM's count value for future calculation */
        uint32_t const post_sleep_cnt = lptim_read_cnt(&lptim_handle);
        uint32_t const post_sleep_arr = lptim_read_arr(&lptim_handle);

        /* Read LPTIM1 autoreload match flag before enabling interrupts */
        bool woke_by_lptim_arrm =
            __HAL_LPTIM_GET_FLAG(&lptim_handle, LPTIM_FLAG_ARRM);

        #if 0
        /* THIS DOES NOT HOLD WHILE DEBUGGING. IT CAN BE USED TO TEST
         * OTHER ASSUMPTIONS, BUT HALTING THE PROCESSOR WILL MAKE THIS
         * WEIRD. */
        assert(woke_by_lptim_arrm ? post_sleep_cnt == 0 : true);
        #endif

        /* If `post_sleep_cnt == 0`, we must have been woken up by
         * LPTIM ARRM or by an extremely well timed interrupt. Either
         * way, treat it as if we've been woken by the lptim_arrm.
         */
        if (post_sleep_cnt == 0) {
            woke_by_lptim_arrm = true;
        }

        /* Re-enable interrupts - see comments above the cpsid instruction */
        __asm volatile("cpsie i");

        /**************************************************************
         * LPTIM1_IRQHandler is called between these two statements if
         * the ARR matched/we got a normal wakeup.
         **************************************************************/

        /* Will be set to the number of ticks that were completed
         * between the time we went to sleep and woke up again. */
        uint32_t completed_ticks = 0;

        if (!lptim_arr_is_stable(&lptim_handle))
        {
            /* We went to sleep, but something woke us up before the
             * ARR had settled completely. We can't do anything about
             * this right now so bail out to the idle task. It will
             * attempt again shortly. */
            dbg_util_event_log(dbg_lptim_complete_unstable_arr);
            return;
        } else if (woke_by_lptim_arrm)
        {
            /* We're awake because of LPTIM compare match match interrupt, which
             * means we completed the tickless period. */

            /* Determine which tick of the N ticks we can hold in the
             * LPTIM1 CNT register. */
            uint32_t const post_sleep_tick =
                lptim_calculate_tick_from_cnt(post_sleep_arr);

            /* We will use this value to update FreeRTOS's tick count  */
            completed_ticks = post_sleep_tick - pre_sleep_tick;

            /* Configure timer to fire the next interrupt at a normal rate */
            uint32_t const next_arr =
                lptim_calculate_arr_for_next_n_ticks(0, 1);
            if (lptim_should_load_arr(post_sleep_cnt, post_sleep_arr, next_arr))
            {
                lptim_load_arr(&lptim_handle, next_arr);
            }
        }
        else
        {
            /* Something other than LPTIM1 woke us up, figure out how long we
             * actually slept */
            assert(post_sleep_cnt >= pre_sleep_cnt);
            uint32_t const post_sleep_tick =
                lptim_calculate_tick_from_cnt(post_sleep_cnt);
            completed_ticks = post_sleep_tick - pre_sleep_tick;
            assert(completed_ticks <= post_sleep_tick);

            /* Calculate ARR for the next tick boundary */
            uint32_t const next_arr_0 =
                lptim_calculate_arr_for_next_n_ticks(post_sleep_cnt, 1);
            /* In case we're too close to the next tick boundary, calculate ARR
             * for two tick boundaries from now */
            uint32_t const next_arr_1 =
                lptim_calculate_arr_for_next_n_ticks(post_sleep_cnt, 2);

            if (lptim_should_load_arr(
                    post_sleep_cnt, arr_for_sleep, next_arr_0))
            {
                /* We can load ARR for the next tick boundary */
                assert(next_arr_0 <= arr_for_sleep);
                lptim_load_arr(&lptim_handle, next_arr_0);
            }
            else if (lptim_should_load_arr(
                         post_sleep_cnt, arr_for_sleep, next_arr_1))
            {
                /* We can load ARR for two tick boundaries from now */
                assert(next_arr_1 <= arr_for_sleep);
                lptim_load_arr(&lptim_handle, next_arr_1);
            }
            else if ((post_sleep_tick + 1) == LPTIM1_MAX_SUPPRESSED_TICKS)
            {
                /* ARR is already set to its maximum value. All we can do set it
                 * for first tick boundary after wrap-around */
                lptim_load_arr(
                    &lptim_handle, lptim_calculate_arr_for_next_n_ticks(0, 1));
            }
            else
            {
                /* We're too close to the next time the tick would
                 * have expired. This happens when both the previous
                 * checks for lptim_should_load_arr fail. Since we
                 * can't reset the ARRM and since it's going to expire
                 * very soon any way, we're going to just update the
                 * completed ticks from this interrupt and wait for
                 * the next ARRM interrupt. */
            }
        }

        portENTER_CRITICAL();
        {
            vTaskStepTick(completed_ticks);
        }
        portEXIT_CRITICAL();
    }
}

#endif /* configUSE_TICKLESS_IDLE == 2 */
