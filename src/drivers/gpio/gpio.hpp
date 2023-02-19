////////////////////////////////////////////////////////////////////////////////////////////////////
/// @author  Ryan Sullivan (ryansullivan@googlemail.com)
///
/// @file    gpio.hpp
/// @brief   GPIO driver module.
////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once

#include <cstdint>
#include <optional>
#include <variant>
#include <system_error>

/*------------------------------------------------------------------------------------------------*/
// Error handling.
/*------------------------------------------------------------------------------------------------*/

namespace gpio {

enum class StatusCode: uint32_t {
    Ok,
    Unimplemented,

    UnderlyingDriverError,
    InvalidAltFunction,

};

auto make_error_code(StatusCode) -> std::error_code;

}

namespace std {

template <>
struct is_error_code_enum<gpio::StatusCode> : true_type {};

}

/*------------------------------------------------------------------------------------------------*/
// Driver configuration.
/*------------------------------------------------------------------------------------------------*/

namespace gpio {

enum class Pin: uint32_t {
    A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15,
    B0, B1, B2, B3, B4, B5, B6, B7, B8, B9, B10, B11, B12, B13, B14, B15,
    C0, C1, C2, C3, C4, C5, C6, C7, C8, C9, C10, C11, C12, C13, C14, C15,
    D0, D1, D2, D3, D4, D5, D6, D7, D8, D9, D10, D11, D12, D13, D14, D15,
    E0, E1, E2, E3, E4, E5, E6, E7, E8, E9, E10, E11, E12, E13, E14, E15,
    F0, F1, F2, F3, F4, F5, F6, F7, F8, F9, F10, F11, F12, F13, F14, F15,

    PinCount,
};

enum class Mode: uint32_t {
    Input,
    Output,
    Analog,
    AltFunction,
};

enum class OutputType: uint32_t {
    PushPull,
    OpenDrain,
};

enum class Pull: uint32_t {
    None,
    Up,
    Down,
};

struct Input {
    std::optional<Pull> pull {};
};

struct Output {
    OutputType output_type {};
    std::optional<Pull> pull {};
};

struct Analog {};

struct AltFunction {
    OutputType output_type {};
    std::optional<Pull> pull {};
    uint32_t function {};
};

template<class... Types> struct Overload : Types... { using Types::operator()...; };
template<class... Ts> Overload(Ts...) -> Overload<Ts...>;

using Config = std::variant<Input, Output, Analog, AltFunction>;

}

/*------------------------------------------------------------------------------------------------*/
// Driver Concepts.
/*------------------------------------------------------------------------------------------------*/

namespace gpio {

template<typename T>
concept GPIODriver = requires(const Pin pin, const Config& config) {
    {T::init(pin, config)} -> std::same_as<std::error_code>;

    {T::set(pin)}   -> std::same_as<void>;
    {T::reset(pin)} -> std::same_as<void>;

    {T::is_set(pin)}   -> std::same_as<bool>;
    {T::is_reset(pin)} -> std::same_as<bool>;
};

}

/*------------------------------------------------------------------------------------------------*/
