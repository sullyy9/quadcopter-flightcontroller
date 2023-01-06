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
#include <optional>

#include "stm32f3xx_ll_rcc.h"
#include "stm32f3xx_ll_iwdg.h"

#include "system_info.hpp"
#include "watchdog.hpp"


struct IWDGStatusCategory : std::error_category
{
    const char* name() const noexcept override;
    std::string message(int ev) const override;
};
const IWDGStatusCategory WATCHDOG_STATUS_CATEGORY {};

static constexpr uint32_t RELOAD_VALUE_MAX {IWDG_RLR_RL_Msk};

struct Prescaler {
    uint32_t key   {};
    uint32_t value {};

    constexpr auto max_period() -> sys::Microseconds {
        return sys::lso::period * value * RELOAD_VALUE_MAX;
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

/// @brief Return an instance of the independant watchdog, running and intialised with the specified
///        timeout period.
///
/// @param timeout Time period after which a processor reset will be triggered.
///
/// @return Tuple containing an error code, detailling the status, and a watchdog instance if the
///         status is Ok.
///
auto iwdg::Watchdog::with_timeout(const sys::Microseconds timeout) -> std::tuple<std::optional<Watchdog>, std::error_code> {
    auto suitable = [timeout] (Prescaler prescaler) {
        return prescaler.min_period() <= timeout && timeout <= prescaler.max_period();
    };

    auto prescaler {std::ranges::find_if(prescaler_map, suitable)};
    if (prescaler == prescaler_map.end()) {
        return {std::nullopt, StatusCode::InvalidTimeout};
    }

    auto reload_value {timeout / (sys::lso::period * prescaler->value)};

    return {Watchdog{prescaler->key, reload_value}, StatusCode::Ok};
}

/*------------------------------------------------------------------------------------------------*/

iwdg::Watchdog::Watchdog(const uint32_t prescaler_value, const uint32_t reload_value) {

    LL_RCC_LSI_Enable();
    while (!LL_RCC_LSI_IsReady());

    LL_IWDG_Enable(IWDG);

    LL_IWDG_EnableWriteAccess(IWDG);
    LL_IWDG_SetPrescaler(IWDG, prescaler_value);
    LL_IWDG_SetReloadCounter(IWDG, reload_value);
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
 
const char* IWDGStatusCategory::name() const noexcept {
    return "Watchdog";
}

/*------------------------------------------------------------------------------------------------*/

std::string IWDGStatusCategory::message(int status) const {
    using enum iwdg::StatusCode;
    switch (static_cast<iwdg::StatusCode>(status)) {
        case Ok:             return "Ok";
        case InvalidTimeout: return "Invalid timeout";
        default:             return "Unknown";
    }
}

/*------------------------------------------------------------------------------------------------*/

std::error_code iwdg::make_error_code(iwdg::StatusCode e) {
    return {static_cast<int>(e), WATCHDOG_STATUS_CATEGORY};
}
