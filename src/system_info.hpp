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

#include "stm32f3xx_ll_rcc.h"

using Frequency = uint32_t;

namespace sys {
    using Nanoseconds = std::chrono::duration<uint32_t, std::nano>;
    using Microseconds = std::chrono::duration<uint32_t, std::micro>;
    using Milliseconds = std::chrono::duration<uint32_t, std::milli>;
    using Seconds = std::chrono::duration<uint32_t>;

    namespace lso {
        inline constexpr Frequency    frequency {LSI_VALUE};
        inline constexpr Microseconds period {1'000'000 / frequency};        
    }
}
