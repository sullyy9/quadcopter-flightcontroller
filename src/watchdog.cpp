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

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <optional>
#include <type_traits>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "stm32f3xx_ll_iwdg.h"
#include "stm32f3xx_ll_rcc.h"
#pragma GCC diagnostic pop

#include "watchdog.hpp"

using namespace wdg;
/*------------------------------------------------------------------------------------------------*/
/*-constant-definitions---------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

constexpr std::pair<const uint32_t, const uint32_t> prescaler_list[] = 
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

static bool watchdog_instance_created = false;

/*------------------------------------------------------------------------------------------------*/
/*-forward-declarations---------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------------------*/
/*-public-methods---------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/**
 * @brief Create a watchdog instance, within an optional container, if one doesn't already exist.
 *
 * @param timeout_period Time period after which the watchdog will trigger a system reset.
 *
 * @return Container which may or may not conatin a watchdog instance.
 */
std::optional<Watchdog* const> 
Watchdog::get_instance(const std::chrono::milliseconds timeout_period)
{
    if(watchdog_instance_created)
    {
        return(std::nullopt);
    }
    else
    {
        watchdog_instance_created = true;
        
        static Watchdog watchdog(timeout_period);
        std::optional<Watchdog*> watchdog_container(&watchdog);
        return(watchdog_container);
    }
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
/*-private-methods--------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/**
 * @brief Calculate the prescaler and reload values to match the timeout period and enable the
 *        watchdog.
 *
 * @param timeout_period Seconds after which the watchdog will trigger a reset.
 */
Watchdog::Watchdog(const std::chrono::milliseconds timeout_period)
{
    constexpr auto prescaler = calculate_prescaler_value(timeout_period);
    const auto reload_value = calculate_reload_value(timeout_period, prescaler.first);

    LL_RCC_LSI_Enable();
    while (! LL_RCC_LSI_IsReady());

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
 * @brief Calculate the mimimum prescaler value that allows the timeout_period to be reached.
 *
 * @param timeout_period  .
 *
 * @return Reference to the prescaler_list element containing the prescaler values.
 */
constexpr std::pair<const uint32_t, const uint32_t>
Watchdog::calculate_prescaler_value(const std::chrono::milliseconds timeout_period)
{
    constexpr auto test = timeout_period;

    // Find a prescaler value that allows us to achieve a timeout period greater than the target.
    constexpr uint32_t timeout_ms        = static_cast<uint32_t>(timeout_period.count());
    constexpr uint32_t reload_value_max  = IWDG_RLR_RL_Msk;
    constexpr uint32_t lsi_cycles_per_ms = LSI_VALUE / 1000;
    constexpr uint32_t prescaler_target  = ((timeout_ms * lsi_cycles_per_ms + 1) / reload_value_max);

    // Find the first prescaler value that is less than the target.
    constexpr auto prescaler = *std::find_if(
        std::begin(prescaler_list),
        std::end(prescaler_list),
        [prescaler_target](auto prescaler_pair){return(prescaler_pair.first >= prescaler_target);}
        );

    return(prescaler);
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief Calculate a reload value that results in the timeout_period, given a presaler_value.
 *
 * @param timeout_period  .
 * @param prescaler_value .
 *
 * @return Reload value.
 */
constexpr uint32_t
Watchdog::calculate_reload_value(const std::chrono::milliseconds timeout_period, 
                                 const uint32_t                  prescaler_value)
{
    const uint32_t timeout_ms        = static_cast<uint32_t>(timeout_period.count());
    const uint32_t lsi_cycles_per_ms = LSI_VALUE / 1000;

    return((timeout_ms * lsi_cycles_per_ms + 1) / prescaler_value);
}

/*------------------------------------------------------------------------------------------------*/
/*-end-of-module----------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/
