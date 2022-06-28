/**
 * -------------------------------------------------------------------------------------------------
 * @author  Ryan Sullivan (ryansullivan@googlemail.com)
 *
 * @file    system_info.hpp
 * @brief   Constants which define system specifications.
 *
 * @date    2022-06-04
 * -------------------------------------------------------------------------------------------------
 */

#pragma once

#include <chrono>
#include <cstdint>
#include <ratio>
#include <sys/_stdint.h>

#include "stm32f3xx_ll_rcc.h"

namespace sys {
using clock_period    = std::chrono::duration<float>;
using clock_frequency = uint32_t;

    namespace lso {

        inline constexpr clock_frequency frequency {LSI_VALUE};
        inline constexpr clock_period    period    {1.0 / frequency};       

        inline constexpr uint32_t ticks_ms  {frequency / 1'000};
        
    }

    namespace utils {
        consteval auto calculate_clock_period(const clock_frequency frequency, 
                                              const uint32_t        prescaler, 
                                              const uint32_t        reload_value) -> clock_period {

            sys::clock_period period {prescaler * sys::lso::period};
            return(period * reload_value);
        }
    }

}
