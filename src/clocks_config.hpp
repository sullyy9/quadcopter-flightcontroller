#pragma once
/**
 * -------------------------------------------------------------------------------------------------
 * @author  Ryan Sullivan (ryansullivan@googlemail.com)
 *
 * @file    clock_config.hpp
 * @brief   Clock frequency configuration. For inclusion in clocks.cpp only.
 *
 * @date    2022-02-26
 * -------------------------------------------------------------------------------------------------
 */

#include "types.hpp"

#include "stm32f3xx_ll_rcc.h"

namespace clocks {
/*------------------------------------------------------------------------------------------------*/
/*-constant-definitions---------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

constexpr uint32_t sysclk_target_hz = 48'000'000;
constexpr uint32_t hclk_target_hz   = 48'000'000;
constexpr uint32_t pclk1_target_hz  = 48'000'000;
constexpr uint32_t pclk2_target_hz  = 48'000'000;

// Varify validity of clock targets.
static_assert(
    sysclk_target_hz == HSI_VALUE || sysclk_target_hz == ((HSI_VALUE / 2) * 4) ||
        sysclk_target_hz == ((HSI_VALUE / 2) * 5) || sysclk_target_hz == ((HSI_VALUE / 2) * 6) ||
        sysclk_target_hz == ((HSI_VALUE / 2) * 7) || sysclk_target_hz == ((HSI_VALUE / 2) * 8) ||
        sysclk_target_hz == ((HSI_VALUE / 2) * 9) || sysclk_target_hz == ((HSI_VALUE / 2) * 11) ||
        sysclk_target_hz == ((HSI_VALUE / 2) * 12) || sysclk_target_hz == ((HSI_VALUE / 2) * 13) ||
        sysclk_target_hz == ((HSI_VALUE / 2) * 14) || sysclk_target_hz == ((HSI_VALUE / 2) * 15) ||
        sysclk_target_hz == ((HSI_VALUE / 2) * 16),
    "SYSCLK target invalid");

static_assert((hclk_target_hz == (sysclk_target_hz / 1)) ||
                  (hclk_target_hz == (sysclk_target_hz / 2)) ||
                  (hclk_target_hz == (sysclk_target_hz / 4)) ||
                  (hclk_target_hz == (sysclk_target_hz / 8)) ||
                  (hclk_target_hz == (sysclk_target_hz / 16)) ||
                  (hclk_target_hz == (sysclk_target_hz / 64)) ||
                  (hclk_target_hz == (sysclk_target_hz / 128)) ||
                  (hclk_target_hz == (sysclk_target_hz / 256)) ||
                  (hclk_target_hz == (sysclk_target_hz / 512)),
              "HCLK target invalid");

// static_assert(pclk1_target_hz <= 36'000'000, "PCLK1 target Invalid");
static_assert((pclk1_target_hz == (hclk_target_hz / 1)) ||
                  (pclk1_target_hz == (hclk_target_hz / 2)) ||
                  (pclk1_target_hz == (hclk_target_hz / 4)) ||
                  (pclk1_target_hz == (hclk_target_hz / 8)) ||
                  (pclk1_target_hz == (hclk_target_hz / 16)),
              "PCLK1 target Invalid");

static_assert(pclk1_target_hz <= 72'000'000, "PCLK1 target Invalid");
static_assert((pclk2_target_hz == (hclk_target_hz / 1)) ||
                  (pclk2_target_hz == (hclk_target_hz / 2)) ||
                  (pclk2_target_hz == (hclk_target_hz / 4)) ||
                  (pclk2_target_hz == (hclk_target_hz / 8)) ||
                  (pclk2_target_hz == (hclk_target_hz / 16)),
              "PCLK2 target Invalid");

/*------------------------------------------------------------------------------------------------*/
/*-exported-variables-----------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------------------*/
/*-exported-functions-----------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/**
 * @brief  Determine whether the PLL is required to reach the target SYSCLK frequency.
 *
 * @return bool True:  PLL is required.
 *              False: PLL is not required.
 */
constexpr bool pll_required(void)
{
    if(sysclk_target_hz == HSI_VALUE)
    {
        return (false);
    }
    else
    {
        return (true);
    }
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief  Calculate the PLL multiplier required to match the target SYSCLK frequency.
 *
 * @return uint32_t A PLL multiplier value. @ingroup RCC_LL_EC_PLL_MUL.
 */
constexpr uint32_t get_pll_prescaler(void)
{
    uint32_t multiplier = (sysclk_target_hz * 2) / HSI_VALUE;

    switch(multiplier)
    {
        case 2: return (LL_RCC_PLL_MUL_2);
        case 3: return (LL_RCC_PLL_MUL_3);
        case 4: return (LL_RCC_PLL_MUL_4);
        case 5: return (LL_RCC_PLL_MUL_5);
        case 6: return (LL_RCC_PLL_MUL_6);
        case 7: return (LL_RCC_PLL_MUL_7);
        case 8: return (LL_RCC_PLL_MUL_8);
        case 9: return (LL_RCC_PLL_MUL_9);
        case 10: return (LL_RCC_PLL_MUL_10);
        case 11: return (LL_RCC_PLL_MUL_11);
        case 12: return (LL_RCC_PLL_MUL_12);
        case 13: return (LL_RCC_PLL_MUL_13);
        case 14: return (LL_RCC_PLL_MUL_14);
        case 15: return (LL_RCC_PLL_MUL_15);
        case 16: return (LL_RCC_PLL_MUL_16);
        default: return (0);
    }
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief Calculate the AHB divisor required to match the target HCLK frequency.
 *
 * @return uint32_t An AHB divisor value. @ingroup RCC_LL_EC_SYSCLK_DIV.
 */
constexpr uint32_t get_ahb_prescaler(void)
{
    uint32_t divisor = sysclk_target_hz / hclk_target_hz;

    switch(divisor)
    {
        case 1: return (LL_RCC_SYSCLK_DIV_1);
        case 2: return (LL_RCC_SYSCLK_DIV_2);
        case 4: return (LL_RCC_SYSCLK_DIV_4);
        case 8: return (LL_RCC_SYSCLK_DIV_8);
        case 16: return (LL_RCC_SYSCLK_DIV_16);
        case 64: return (LL_RCC_SYSCLK_DIV_64);
        case 128: return (LL_RCC_SYSCLK_DIV_128);
        case 256: return (LL_RCC_SYSCLK_DIV_256);
        case 512: return (LL_RCC_SYSCLK_DIV_512);
        default: return (0);
    }
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief Calculate the APB1 divisor required to match the target PCLK1 frequency.
 *
 * @return uint32_t An APB1 divisor value. @ingroup RCC_LL_EC_APB1_DIV.
 */
constexpr uint32_t get_apb1_prescaler(void)
{
    uint32_t divisor = hclk_target_hz / pclk1_target_hz;

    switch(divisor)
    {
        case 1: return (LL_RCC_APB1_DIV_1);
        case 2: return (LL_RCC_APB1_DIV_2);
        case 4: return (LL_RCC_APB1_DIV_4);
        case 8: return (LL_RCC_APB1_DIV_8);
        case 16: return (LL_RCC_APB1_DIV_16);
        default: return (0);
    }
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief Calculate the APB2 divisor required to match the target PCLK2 frequency.
 *
 * @return uint32_t An APB2 divisor value. @ingroup RCC_LL_EC_APB2_DIV.
 */
constexpr uint32_t get_apb2_prescaler(void)
{
    uint32_t divisor = hclk_target_hz / pclk2_target_hz;

    switch(divisor)
    {
        case 1: return (LL_RCC_APB2_DIV_1);
        case 2: return (LL_RCC_APB2_DIV_2);
        case 4: return (LL_RCC_APB2_DIV_4);
        case 8: return (LL_RCC_APB2_DIV_8);
        case 16: return (LL_RCC_APB2_DIV_16);
        default: return (0);
    }
}

/*------------------------------------------------------------------------------------------------*/
/*-end-of-module----------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/
}
