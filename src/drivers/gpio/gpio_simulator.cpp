////////////////////////////////////////////////////////////////////////////////////////////////////
/// @author  Ryan Sullivan (ryansullivan@googlemail.com)
///
/// @file    gpio_simulator.cpp
/// @brief   GPIO driver module for simulation.
////////////////////////////////////////////////////////////////////////////////////////////////////

#include <cstdint>
#include <optional>
#include <stdint-gcc.h>
#include <system_error>
#include <array>
#include <utility>

#include "gpio.hpp"
#include "gpio_simulator.hpp"

/*------------------------------------------------------------------------------------------------*/
// Module private objects.
/*------------------------------------------------------------------------------------------------*/

constexpr auto PIN_COUNT = static_cast<size_t>(gpio::Pin::PinCount);

enum class OutputState {
    Set,
    Reset,
};

static constinit std::array<gpio::Config,   PIN_COUNT> configuration  {};
static constinit std::array<OutputState,    PIN_COUNT> output_state   {};
static constinit std::array<gpio::PinState, PIN_COUNT> external_state {};

struct SetTableField {
    gpio::PinState internal {};
    gpio::PinState external {};
    bool           is_set   {};
};
constexpr std::array IS_SET_TRUTH_TABLE {
    SetTableField{gpio::PinState::Strong_Pull_Up, gpio::PinState::Strong_Pull_Up,     true        },
    SetTableField{gpio::PinState::Strong_Pull_Up, gpio::PinState::Weak_Pull_Up,       true        },
    SetTableField{gpio::PinState::Strong_Pull_Up, gpio::PinState::Strong_Pull_Down,   false       }, // Throw?
    SetTableField{gpio::PinState::Strong_Pull_Up, gpio::PinState::Weak_Pull_Down,     true        },
    SetTableField{gpio::PinState::Strong_Pull_Up, gpio::PinState::Floating,           true        },

    SetTableField{gpio::PinState::Weak_Pull_Up, gpio::PinState::Strong_Pull_Up,       true        },
    SetTableField{gpio::PinState::Weak_Pull_Up, gpio::PinState::Weak_Pull_Up,         true        },
    SetTableField{gpio::PinState::Weak_Pull_Up, gpio::PinState::Strong_Pull_Down,     false       },
    SetTableField{gpio::PinState::Weak_Pull_Up, gpio::PinState::Weak_Pull_Down,       true        },
    SetTableField{gpio::PinState::Weak_Pull_Up, gpio::PinState::Floating,             true        },

    SetTableField{gpio::PinState::Strong_Pull_Down, gpio::PinState::Strong_Pull_Up,   false       }, // Throw?
    SetTableField{gpio::PinState::Strong_Pull_Down, gpio::PinState::Weak_Pull_Up,     false       },
    SetTableField{gpio::PinState::Strong_Pull_Down, gpio::PinState::Strong_Pull_Down, false       },
    SetTableField{gpio::PinState::Strong_Pull_Down, gpio::PinState::Weak_Pull_Down,   false       },
    SetTableField{gpio::PinState::Strong_Pull_Down, gpio::PinState::Floating,         false       },

    SetTableField{gpio::PinState::Weak_Pull_Down, gpio::PinState::Strong_Pull_Up,     true        },
    SetTableField{gpio::PinState::Weak_Pull_Down, gpio::PinState::Weak_Pull_Up,       true        },
    SetTableField{gpio::PinState::Weak_Pull_Down, gpio::PinState::Strong_Pull_Down,   false       },
    SetTableField{gpio::PinState::Weak_Pull_Down, gpio::PinState::Weak_Pull_Down,     false       },
    SetTableField{gpio::PinState::Weak_Pull_Down, gpio::PinState::Floating,           false       },

    SetTableField{gpio::PinState::Floating, gpio::PinState::Strong_Pull_Up,           true        },
    SetTableField{gpio::PinState::Floating, gpio::PinState::Weak_Pull_Up,             true        },
    SetTableField{gpio::PinState::Floating, gpio::PinState::Strong_Pull_Down,         false       },
    SetTableField{gpio::PinState::Floating, gpio::PinState::Weak_Pull_Down,           false       },
    SetTableField{gpio::PinState::Floating, gpio::PinState::Floating,                 false       },
};

/*------------------------------------------------------------------------------------------------*/
// Class definitions.
/*------------------------------------------------------------------------------------------------*/

auto gpio::GPIOSimulator::init(const Pin pin, const Config& config) -> std::error_code {

    configuration.at(std::to_underlying(pin)) = config;

    return StatusCode::Ok;
}

/*------------------------------------------------------------------------------------------------*/

auto gpio::GPIOSimulator::set(const Pin pin) -> void {
    output_state.at(std::to_underlying(pin)) = OutputState::Set;
}

/*------------------------------------------------------------------------------------------------*/

auto gpio::GPIOSimulator::reset(const Pin pin) -> void {
    output_state.at(std::to_underlying(pin)) = OutputState::Reset;
}

/*------------------------------------------------------------------------------------------------*/

auto gpio::GPIOSimulator::is_set(const Pin pin) -> bool {
    auto pin_internal_state = SimControl::get_internal_state(pin);
    auto pin_external_state = SimControl::get_external_state(pin);

    for(const auto field: IS_SET_TRUTH_TABLE) {
        if(field.internal == pin_internal_state && field.external == pin_external_state) {
            return field.is_set;
        }
    }

    return false;
}

/*------------------------------------------------------------------------------------------------*/

auto gpio::GPIOSimulator::is_reset(const Pin pin) -> bool {
    auto pin_internal_state = SimControl::get_internal_state(pin);
    auto pin_external_state = SimControl::get_external_state(pin);

    for(const auto field: IS_SET_TRUTH_TABLE) {
        if(field.internal == pin_internal_state && field.external == pin_external_state) {
            return !field.is_set;
        }
    }

    return false;
}

/*------------------------------------------------------------------------------------------------*/

/// @brief Reset all interanl and external pin states to floating.
///
auto gpio::SimControl::reset() -> void {
    configuration.fill(gpio::Analog());
    output_state.fill(OutputState::Reset);
    external_state.fill(PinState::Floating);
}

/*------------------------------------------------------------------------------------------------*/

/// @brief Return the state thatr is a combination of the output state and resistor pull
///        configuration.
///
auto gpio::SimControl::get_internal_state(Pin pin) -> PinState {
    auto internal_state = std::visit(Overload{
        [](const Input& cfg) {
            switch(cfg.pull.value_or(Pull::None)) {
                case Pull::None: return PinState::Floating;
                case Pull::Down: return PinState::Weak_Pull_Down;
                case Pull::Up:   return PinState::Weak_Pull_Up;
                default:         return PinState::Floating;
            }
        },

        [pin](const Output& cfg) {
            bool pin_is_set = output_state.at(std::to_underlying(pin)) == OutputState::Set;

            if(cfg.output_type == OutputType::PushPull && pin_is_set) {
                return PinState::Strong_Pull_Up;
            }

            if(cfg.output_type == OutputType::OpenDrain && pin_is_set) {
                switch(cfg.pull.value_or(Pull::None)) {
                    case Pull::None: return PinState::Floating;
                    case Pull::Down: return PinState::Weak_Pull_Down;
                    case Pull::Up:   return PinState::Weak_Pull_Up;
                }
            }

            return PinState::Strong_Pull_Down;
        },

        [](const Analog&) {
            return PinState::Floating;
        },

        [pin](const AltFunction& cfg) {
            bool pin_is_set = output_state.at(std::to_underlying(pin)) == OutputState::Set;
            
            if(cfg.output_type == OutputType::PushPull && pin_is_set) {
                return PinState::Strong_Pull_Up;
            }

            if(cfg.output_type == OutputType::OpenDrain && pin_is_set) {
                switch(cfg.pull.value_or(Pull::None)) {
                    case Pull::None: return PinState::Floating;
                    case Pull::Down: return PinState::Weak_Pull_Down;
                    case Pull::Up:   return PinState::Weak_Pull_Up;
                }
            }

            return PinState::Strong_Pull_Down;
        },

    }, configuration.at(std::to_underlying(pin)));

    return internal_state;
}

/*------------------------------------------------------------------------------------------------*/

/// @brief Set the external state of a pin.
///
auto gpio::SimControl::set_external_state(Pin pin, PinState state) -> void {
    external_state.at(std::to_underlying(pin)) = state;
}

/*------------------------------------------------------------------------------------------------*/

/// @brief Return the external state of a pin.
///
auto gpio::SimControl::get_external_state(Pin pin) -> PinState {
    return external_state.at(std::to_underlying(pin));
}

/*------------------------------------------------------------------------------------------------*/


