/**
 * -------------------------------------------------------------------------------------------------
 * @author  Ryan Sullivan (ryansullivan@googlemail.com)
 *
 * @file    system.c
 * @brief   Module for system configuration
 *
 * @date    2021-04-03
 * -------------------------------------------------------------------------------------------------
 */

#include "types.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "stm32f3xx_ll_system.h"
#include "stm32f3xx_ll_bus.h"
#include "stm32f3xx_ll_rcc.h"
#include "stm32f3xx_ll_wwdg.h"
#pragma GCC diagnostic pop

#include "clocks.hpp"

#include "debug.hpp"

using namespace clocks;
/*------------------------------------------------------------------------------------------------*/
/*-constant-definitions---------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/*
 * Clock configuration                                          -> HCLK[48MHz]
 *                                                              |
 * HSI[8MHz] -> [/2][4MHz] -> [x12]PLL[48MHz] -> [x1]AHB[48MHz] -> [x1]APB1[48MHz] -> PCLK1[48MHz]
 *                                            |                 |
 *                                            V                 V
 *                                        SYSCLK[48MHz]    [x1]APB2[48MHz] -> PCLK2[48MHz]
 */

#define SYSCLK            48000000
#define HCLK_HZ           (SYSCLK)                         // Core clock
#define PCLK1_HZ          (HCLK_HZ)                        // Peripheral bus 1 clock
#define SYS_TIMER_1MS     (uint32_t)(1 * (HCLK_HZ / 1000)) // 1.000ms
#define WATCHDOG_TIMER_HZ (uint32_t)(PCLK1_HZ / 4096)      // Window watchdog clock

constexpr uint32_t sysclk_target_hz = 48'000'000;
constexpr uint32_t hclk_target_hz   = 48'000'000;
constexpr uint32_t pclk1_target_hz  = 48'000'000;
constexpr uint32_t pclk2_target_hz  = 48'000'000;

// Varify validity of clock targets.
static_assert(sysclk_target_hz >= 8'000'000, "SYSCLK must be >= 8MHZ");
static_assert(sysclk_target_hz <= 64'000'000, "SYSCLK must be <= 64MHZ");
static_assert(!(sysclk_target_hz % (HSI_VALUE / 2)), "SYSCLK must be a multiple of HSI/2");

static_assert(hclk_target_hz <= sysclk_target_hz, "HCLK must be <= SYSCLK");
static_assert(pclk1_target_hz <= hclk_target_hz, "PCLK1 must be <= HCLK");
static_assert(pclk2_target_hz <= hclk_target_hz, "PCLK2 must be <= HCLK");

#define WWDG_RESET_MAX     64
#define WWDG_PRESCALER_MAX 8
#define WWDG_RESET_TIME_MAX_MS \
    (uint32_t)((WWDG_RESET_MAX * WWDG_PRESCALER_MAX * 1000) / WATCHDOG_TIMER_HZ)

/*------------------------------------------------------------------------------------------------*/
/*-exported-variables-----------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------------------*/
/*-static-variables-------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

static uint32_t wwdg_reset_value = 1;

/*------------------------------------------------------------------------------------------------*/
/*-forward-declarations---------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------------------*/
/*-exported-functions-----------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/**
 * @brief Clear any flags set during the last reset.
 */
void clocks::clear_reset_flags(void)
{
    LL_RCC_ClearResetFlags();
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief               Setup the window watchdog.
 * @param reset_time_ms time period after which a system reset is triggered. Maximum of 43ms.
 */
void clocks::initialise_wwdg(uint32_t reset_time_ms)
{
    /*
     * Calculate the prescaler an reset value.
     * If the reset time is above the maximum, use the time possible.
     * Find the prescaler using the maximum possible reset value
     * Find the reset value using the found prescaler value
     */
    uint32_t wwdg_prescaler = 1;
    if(reset_time_ms < WWDG_RESET_TIME_MAX_MS)
    {
        while(((reset_time_ms * WATCHDOG_TIMER_HZ) / (WWDG_RESET_MAX * 1000) > wwdg_prescaler))
        {
            wwdg_prescaler = wwdg_prescaler * 2;
        }

        while(((reset_time_ms * WATCHDOG_TIMER_HZ) / (wwdg_prescaler * 1000) > wwdg_reset_value))
        {
            wwdg_reset_value++;
        }
    }
    else
    {
        wwdg_prescaler   = 3;
        wwdg_reset_value = 64;
    }

    switch(wwdg_prescaler)
    {
        case 1:
        {
            LL_WWDG_SetPrescaler(WWDG, LL_WWDG_PRESCALER_1);
            break;
        }
        case 2:
        {
            LL_WWDG_SetPrescaler(WWDG, LL_WWDG_PRESCALER_2);
            break;
        }
        case 4:
        {
            LL_WWDG_SetPrescaler(WWDG, LL_WWDG_PRESCALER_4);
            break;
        }
        case 8:
        {
            LL_WWDG_SetPrescaler(WWDG, LL_WWDG_PRESCALER_8);
            break;
        }
        default:
        {
            break;
        }
    }

    LL_WWDG_SetWindow(WWDG, 127);
    LL_WWDG_SetCounter(WWDG, (63 + wwdg_reset_value));

    LL_WWDG_EnableIT_EWKUP(WWDG);
    LL_WWDG_ClearFlag_EWKUP(WWDG);
    NVIC_SetPriority(WWDG_IRQn, 1);
    NVIC_EnableIRQ(WWDG_IRQn);

    LL_WWDG_Enable(WWDG);
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief Reset the watchdog timer.
 */
void clocks::reset_wwdg(void)
{
    LL_WWDG_SetCounter(WWDG, (63 + wwdg_reset_value));
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief           Get the number of microseconds elapsed since the last system timer interupt.
 * @return uint32_t Microseconds since last system timer interupt.
 */
uint32_t clocks::get_system_timer_us(void)
{
    uint32_t systick_us = 0;
    systick_us          = (SysTick->LOAD - SysTick->VAL);
    systick_us          = (systick_us / (HCLK_HZ / 1000000));

    return (systick_us);
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief Setup system clocks
 */
void clocks::initialise(void)
{
    /*
     * Setup clock sources (HSI running at 8MHz, LSI at 40KHz).
     */
    LL_RCC_DeInit();
    LL_RCC_HSI_Enable();
    LL_RCC_HSE_Disable();
    LL_RCC_LSI_Enable();
    LL_RCC_LSE_Disable();
    while(!LL_RCC_HSI_IsReady()) {}

    /*
     * Setup the PLL to multiply HSI by 6
     */
    LL_RCC_PLL_SetMainSource(LL_RCC_PLLSOURCE_HSI_DIV_2);
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_12);
    LL_RCC_PLL_Enable();
    while(!LL_RCC_PLL_IsReady()) {}

    /*
     * Setup the bus prescalers
     */
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

    /*
     * Set the flash wait cycles to 1
     */
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
    LL_FLASH_DisablePrefetch();

    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {}

    /*
     * Enable peripheral clocks
     */
    LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK2);
    while(LL_RCC_GetUSARTClockSource(LL_RCC_USART1_CLKSOURCE) != LL_RCC_USART1_CLKSOURCE_PCLK2) {}

    LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_SYSCLK);
    while(LL_RCC_GetI2CClockSource(LL_RCC_I2C1_CLKSOURCE) != LL_RCC_I2C1_CLKSOURCE_SYSCLK) {}

    /*
     * AHB clocks
     */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA | LL_AHB1_GRP1_PERIPH_GPIOB |
                             LL_AHB1_GRP1_PERIPH_GPIOC | LL_AHB1_GRP1_PERIPH_GPIOD |
                             LL_AHB1_GRP1_PERIPH_GPIOE | LL_AHB1_GRP1_PERIPH_GPIOF |
                             LL_AHB1_GRP1_PERIPH_DMA1);

    /*
     * APB1 clocks
     */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1 | LL_APB1_GRP1_PERIPH_WWDG);

    /*
     * APB2 clocks
     */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG | LL_APB2_GRP1_PERIPH_USART1 |
                             LL_APB2_GRP1_PERIPH_SPI1);

    SysTick_Config(SYS_TIMER_1MS);
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief Interrupt triggered by the window watchdog.
 */
void clocks::wwdg_isr(void)
{
    LL_WWDG_ClearFlag_EWKUP(WWDG);
}

/*------------------------------------------------------------------------------------------------*/
/*-static-functions-------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/**
 * @brief                 Determine whether the PLL is needed to reach the target
 *                        sysclk frequency.
 * @return constexpr bool True if the PLL is required. False otherwise.
 */
constexpr bool pll_required(void)
{
    if(sysclk_target_hz == HSI_VALUE)
    {
        return (true);
    }
    else
    {
        return (false);
    }
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief   Calculate the PLL multiplier required to match
 * @return constexpr uint32_t
 */
constexpr uint32_t calculate_pll_factor(void)
{
    // sysclk = (hsi / 2) * PLL_mul
    // (sysclk * 2) / HSI = PLL_mul
    uint32_t multiplier = (sysclk_target_hz * 2) / HSI_VALUE;

    switch(multiplier)
    {
        case 2: return (RCC_CFGR_PLLMUL2);
        case 3: return (RCC_CFGR_PLLMUL3);
        case 4: return (RCC_CFGR_PLLMUL4);
        case 5: return (RCC_CFGR_PLLMUL5);
        case 6: return (RCC_CFGR_PLLMUL6);
        case 7: return (RCC_CFGR_PLLMUL7);
        case 8: return (RCC_CFGR_PLLMUL8);
        case 9: return (RCC_CFGR_PLLMUL9);
        case 10: return (RCC_CFGR_PLLMUL10);
        case 11: return (RCC_CFGR_PLLMUL11);
        case 12: return (RCC_CFGR_PLLMUL12);
        case 13: return (RCC_CFGR_PLLMUL13);
        case 14: return (RCC_CFGR_PLLMUL14);
        case 15: return (RCC_CFGR_PLLMUL15);
        case 16: return (RCC_CFGR_PLLMUL16);
        default: break;
    }
}

/*------------------------------------------------------------------------------------------------*/
/*-end-of-module----------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/
