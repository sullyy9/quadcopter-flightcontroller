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

#include <ranges>
#include <chrono>
#include <memory>
#include <ranges>

#include "stm32f3xx_ll_rcc.h"
#include "stm32f3xx_ll_iwdg.h"

#include "system_info.hpp"
#include "watchdog.hpp"

/*------------------------------------------------------------------------------------------------*/
/*-forward-declarations---------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------------------*/
/*-constant-definitions---------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

static constexpr uint32_t reload_value_max {IWDG_RLR_RL_Msk};

struct Prescaler {
    uint32_t key   {};
    uint32_t value {};

    constexpr auto max_period() -> sys::Microseconds {
        return sys::lso::period * value * reload_value_max;
    }

    constexpr auto min_period() -> sys::Microseconds {
        return sys::lso::period * value;
    }
};
static constexpr std::array prescaler_map {
    Prescaler{LL_IWDG_PRESCALER_4,   4  },
    Prescaler{LL_IWDG_PRESCALER_8,   8  },
    Prescaler{LL_IWDG_PRESCALER_16,  16 },
    Prescaler{LL_IWDG_PRESCALER_32,  32 },
    Prescaler{LL_IWDG_PRESCALER_64,  64 },
    Prescaler{LL_IWDG_PRESCALER_128, 128},
    Prescaler{LL_IWDG_PRESCALER_256, 256}
};

/*------------------------------------------------------------------------------------------------*/
/*-exported-variables-----------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------------------*/
/*-static-variables-------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------------------*/
/*-public-methods---------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/// @brief Return an instance of the Independant Watchdog, running and intialised with the specified
///        timeout period.
auto iwdg::Watchdog::with_timeout(const sys::Microseconds timeout) -> cpp::result<Watchdog&, Error> {
    auto suitable = [timeout] (Prescaler prescaler) {
        return prescaler.min_period() <= timeout && timeout <= prescaler.max_period();
    };

    auto prescaler {std::ranges::find_if(prescaler_map, suitable)};

    if (prescaler == prescaler_map.end()) {
        return cpp::fail(iwdg::Error::InvalidTimeout);
    }

    auto reload_value {timeout / (sys::lso::period * prescaler->value)};

    static Watchdog watchdog {prescaler->key, reload_value};

    return watchdog;
}

/*------------------------------------------------------------------------------------------------*/

// Return an instance of the Independant Watchdog without any intialisation.
//
auto iwdg::Watchdog::uninitialised() -> Watchdog& {
    static Watchdog watchdog;
    return watchdog;
}

/*------------------------------------------------------------------------------------------------*/

iwdg::Watchdog::Watchdog(const uint32_t prescaler_register_value, const uint32_t reload_register_value) {

    LL_RCC_LSI_Enable();
    while (!LL_RCC_LSI_IsReady());

    LL_IWDG_Enable(IWDG);

    LL_IWDG_EnableWriteAccess(IWDG);
    LL_IWDG_SetPrescaler(IWDG, prescaler_register_value);
    LL_IWDG_SetReloadCounter(IWDG, reload_register_value);
    LL_IWDG_DisableWriteAccess(IWDG);

    while(!LL_IWDG_IsReady(IWDG));
    LL_IWDG_ReloadCounter(IWDG);
}

/*------------------------------------------------------------------------------------------------*/

/// @brief Reset the watchdog countdown to the timeout_period.
///
void iwdg::Watchdog::update(void) {
    LL_IWDG_ReloadCounter(IWDG);
}

/*------------------------------------------------------------------------------------------------*/
/*-private-methods--------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------------------*/
/*-end-of-module----------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/
