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
#include "clocks_config.hpp"

#include "debug.hpp"

using namespace clocks;
/*------------------------------------------------------------------------------------------------*/
/*-constant-definitions---------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/*
 * Clock configuration                   -> HCLK
 *                                       |
 * HSI[8MHz] -> [/2][4MHz] -> PLL -> AHB -> APB1 -> PCLK1
 *                                |      |
 *                                V      V
 *                            SYSCLK    APB2 -> PCLK2
 */

#define SYSCLK            48000000
#define HCLK_HZ           (SYSCLK)                           // Core clock
#define SYS_TIMER_1MS     (uint32_t)(1 * (HCLK_HZ / 1000))   // 1.000ms

/*------------------------------------------------------------------------------------------------*/
/*-exported-variables-----------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------------------*/
/*-static-variables-------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

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
 * @brief Setup system clocks.
 *
 * @return Error Status.
 */
Error clocks::initialise(void)
{
    // Reset the system to the default state:
    // - HSI ON and used as system clock source
    // - HSE and PLL OFF
    // - AHB, APB1 and APB2 prescaler set to 1.
    // - CSS, MCO OFF
    // - All interrupts disabled
    LL_RCC_DeInit();
    while(!LL_RCC_HSI_IsReady()) {}

    // Setup the PLL.
    if(pll_required())
    {
        LL_RCC_PLL_SetMainSource(LL_RCC_PLLSOURCE_HSI_DIV_2);
        LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, get_pll_prescaler());
        LL_RCC_PLL_Enable();
        while(!LL_RCC_PLL_IsReady()) {}
    }

    // Setup the bus prescalers.
    LL_RCC_SetAHBPrescaler(get_ahb_prescaler());
    LL_RCC_SetAPB1Prescaler(get_apb1_prescaler());
    LL_RCC_SetAPB2Prescaler(get_apb2_prescaler());

    // The prefetch buffer can be switched off if HCLK == SYSCLK.
    if(LL_RCC_GetAHBPrescaler() == LL_RCC_SYSCLK_DIV_1)
    {
        LL_FLASH_DisablePrefetch();
    }

    // Calaculate the number of needed flash wait states.
    if(hclk_target_hz <= 24'000'000)
    {
        LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
    }
    else if(hclk_target_hz <= 48'000'000)
    {
        LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
    }
    else
    {
        LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
    }

    // Switch the clock source over to PLL.
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {}

    // Varify clocks match the targets.
    LL_RCC_ClocksTypeDef clocks;
    LL_RCC_GetSystemClocksFreq(&clocks);
    if(clocks.SYSCLK_Frequency != sysclk_target_hz)
    {
        return (ERROR_SYSCLK_TARGET);
    }
    if(clocks.HCLK_Frequency != hclk_target_hz)
    {
        return (ERROR_HCLK_TARGET);
    }
    if(clocks.PCLK1_Frequency != pclk1_target_hz)
    {
        return (ERROR_PCLK1_TARGET);
    }
    if(clocks.PCLK2_Frequency != pclk2_target_hz)
    {
        return (ERROR_PCLK2_TARGET);
    }

    // Enable peripheral clocks.
    // TODO move these to peripheral init.
    LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK2);
    while(LL_RCC_GetUSARTClockSource(LL_RCC_USART1_CLKSOURCE) != LL_RCC_USART1_CLKSOURCE_PCLK2) {}

    LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_SYSCLK);
    while(LL_RCC_GetI2CClockSource(LL_RCC_I2C1_CLKSOURCE) != LL_RCC_I2C1_CLKSOURCE_SYSCLK) {}

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA | LL_AHB1_GRP1_PERIPH_GPIOB |
                             LL_AHB1_GRP1_PERIPH_GPIOC | LL_AHB1_GRP1_PERIPH_GPIOD |
                             LL_AHB1_GRP1_PERIPH_GPIOE | LL_AHB1_GRP1_PERIPH_GPIOF |
                             LL_AHB1_GRP1_PERIPH_DMA1);

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1 | LL_APB1_GRP1_PERIPH_WWDG);

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG | LL_APB2_GRP1_PERIPH_USART1 |
                             LL_APB2_GRP1_PERIPH_SPI1);

    SysTick_Config(SYS_TIMER_1MS);

    return (OK);
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

/*------------------------------------------------------------------------------------------------*/
/*-end-of-module----------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/
