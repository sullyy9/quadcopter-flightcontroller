/**
 * -------------------------------------------------------------------------------------------------
 * @author  Ryan Sullivan (ryansullivan@googlemail.com)
 *
 * @file    watchdog.cpp
 * @brief   Implementation of the watchdog module.
 *
 * @date    2022-02-26
 * -------------------------------------------------------------------------------------------------
 */

#include "types.hpp"

#include <algorithm>
#include <chrono>
#include <iterator>
#include <utility>

#include "stm32f3xx_ll_iwdg.h"
#include "stm32f3xx_ll_rcc.h"

#include "watchdog.hpp"

using namespace watchdog;
/*------------------------------------------------------------------------------------------------*/
/*-constant-definitions---------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

static constexpr
std::pair<float, uint32_t> prescaler_list[] = 
{
    {4,   LL_IWDG_PRESCALER_4},
    {8,   LL_IWDG_PRESCALER_8},
    {16,  LL_IWDG_PRESCALER_16},  
    {32,  LL_IWDG_PRESCALER_32},
    {64,  LL_IWDG_PRESCALER_64},  
    {128, LL_IWDG_PRESCALER_128},
    {256, LL_IWDG_PRESCALER_256}
};

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
 * @brief Calculate the prescaler and reload values to match the timeout period and enable the
 *        watchdog.
 *
 * @param timeout_period Seconds after which the watchdog will trigger a reset.
 */
Watchdog::Watchdog(float timeout_period)
{
    LL_RCC_LSI_Enable();
    while (!LL_RCC_LSI_IsReady());

    // Find a prescaler value that allows us to achieve timeout period's greater than the target.
    const uint32_t reload_value_max = IWDG_RLR_RL_Msk;
    const float prescaler_target = (timeout_period * LSI_VALUE) / reload_value_max;

    // Find the first prescaler value that is less than the target.
    auto prescaler = *std::find_if(
        std::begin(prescaler_list),
        std::end(prescaler_list),
        [prescaler_target](auto prescaler_pair){return(prescaler_target >= prescaler_pair.first);}
        );

    // Find a reload value that gives a time period equal or greater than the target.
    uint32_t reload_value = static_cast<uint32_t>((timeout_period * LSI_VALUE) / prescaler.first);

    LL_IWDG_Enable(IWDG);

    LL_IWDG_EnableWriteAccess(IWDG);
    LL_IWDG_SetPrescaler(IWDG, prescaler.second);
    LL_IWDG_SetReloadCounter(IWDG, reload_value);
    LL_IWDG_DisableWriteAccess(IWDG);

    while(!LL_IWDG_IsReady(IWDG));
    LL_IWDG_ReloadCounter(IWDG);
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief Once started, the watchdog cannot be disabled, however disabling LSI will stop it from
 *        counting down.
 */
Watchdog::~Watchdog()
{
    LL_IWDG_ReloadCounter(IWDG);

    LL_RCC_LSI_Disable();
    while (LL_RCC_LSI_IsReady());
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief Reset the watchdog countdown to the timeout_period.
 */
void
Watchdog::update(void)
{
    LL_IWDG_ReloadCounter(IWDG);
}

/*------------------------------------------------------------------------------------------------*/
/*-static-functions-------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------------------*/
/*-end-of-module----------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/
