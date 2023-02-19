////////////////////////////////////////////////////////////////////////////////////////////////////
/// @author  Ryan Sullivan (ryansullivan@googlemail.com)
///
/// @file    gpio_simulator.hpp
/// @brief   GPIO driver module for simulation.
////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once

#include <cstdint>
#include <system_error>
#include <array>

#include "gpio.hpp"

/*------------------------------------------------------------------------------------------------*/
// Class declarations.
/*------------------------------------------------------------------------------------------------*/

namespace gpio {


struct GPIOSimulator {
    [[nodiscard]] static auto init(Pin pin, const Config& config) -> std::error_code;

    static auto set(Pin pin) -> void;
    static auto reset(Pin pin) -> void;

    [[nodiscard]] static auto is_set(Pin pin) -> bool;
    [[nodiscard]] static auto is_reset(Pin pin) -> bool;

};

static_assert(GPIODriver<GPIOSimulator>);

/*------------------------------------------------------------------------------------------------*/

enum class PinState : uint8_t {
    Strong_Pull_Up,
    Weak_Pull_Up,
    Strong_Pull_Down,
    Weak_Pull_Down,
    Floating,
};

/*------------------------------------------------------------------------------------------------*/

struct SimControl {
    static auto reset() -> void;

    static auto get_internal_state(Pin pin) -> PinState;
    static auto set_external_state(Pin pin, PinState state) -> void;
    static auto get_external_state(Pin pin) -> PinState;
};

}


/*------------------------------------------------------------------------------------------------*/
// Function definitions.
/*------------------------------------------------------------------------------------------------*/



/*------------------------------------------------------------------------------------------------*/
