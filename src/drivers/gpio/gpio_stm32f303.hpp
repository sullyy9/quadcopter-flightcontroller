////////////////////////////////////////////////////////////////////////////////////////////////////
/// @author  Ryan Sullivan (ryansullivan@googlemail.com)
///
/// @file    gpio_stm32f303.hpp
/// @brief   GPIO driver module for the STM32F303.
////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once

#include <cstdint>
#include <system_error>

#include "gpio.hpp"

/*------------------------------------------------------------------------------------------------*/
// Class declarations.
/*------------------------------------------------------------------------------------------------*/

namespace gpio {

struct GPIOSTM32F303 {
    [[nodiscard]] static auto init(Pin pin, const Config& config) noexcept -> std::error_code;

    static auto set(Pin pin) noexcept -> void;
    static auto reset(Pin pin) noexcept -> void;

    [[nodiscard]] static auto is_set(Pin pin) noexcept -> bool;
    [[nodiscard]] static auto is_reset(Pin pin) noexcept -> bool;

};

static_assert(GPIODriver<GPIOSTM32F303>);

}

/*------------------------------------------------------------------------------------------------*/
