#pragma once
/**
 * -------------------------------------------------------------------------------------------------
 * @author  Ryan Sullivan (ryansullivan@googlemail.com)
 * 
 * @file    watchdog.hpp
 * @brief   header
 * 
 * @date    2022-02-26
 * -------------------------------------------------------------------------------------------------
 */

#include <optional>
#include <utility>
#include <chrono>

namespace wdg {
/*----------------------------------------------------------------------------*/
/*-constant-definitions-------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-exported-variables---------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-exported-functions---------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

class Watchdog {
public:

    static std::optional<Watchdog* const> get_instance(const std::chrono::milliseconds timeout_period);

    void start(void);
    void update(void);
    

private:

    Watchdog(const std::chrono::milliseconds timeout_period);

    constexpr std::pair<const uint32_t, const uint32_t>
    calculate_prescaler_value(const std::chrono::milliseconds timeout_period);
    
    constexpr uint32_t
    calculate_reload_value(const std::chrono::milliseconds timeout_period,
                            const uint32_t                  prescaler_value);

};

/*----------------------------------------------------------------------------*/
/*-end-of-module--------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
}
