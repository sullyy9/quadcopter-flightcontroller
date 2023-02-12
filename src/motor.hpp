////////////////////////////////////////////////////////////////////////////////////////////////////
/// @author  Ryan Sullivan (ryansullivan@googlemail.com)
///
/// @file    motor.hpp
/// @brief   Module for controlling a motor via a Hobbywing X-rotor ECS.
////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>

#include "esc.hpp"
#include "pwm.hpp"

#include "result.hpp"

/*------------------------------------------------------------------------------------------------*/
// Error handling.
/*------------------------------------------------------------------------------------------------*/

namespace motor {

enum class StatusCode: uint32_t {
    Ok,
    Unimplemented,
    
    InvalidPWMFrequency,
    InvalidThrottle,
};

auto make_error_code(StatusCode) -> std::error_code;

}

namespace std {

template <>
struct is_error_code_enum<motor::StatusCode> : true_type {};

}

/*------------------------------------------------------------------------------------------------*/
// Class declarations.
/*------------------------------------------------------------------------------------------------*/

namespace motor {

template<pwm::PWMInstance PWM, esc::ESCDefinition ESC>
struct Motor {
    Motor() = delete;
    Motor(Motor&) = delete;
    Motor(Motor&&) noexcept = default;
    
    ~Motor() = default;

    auto operator=(const Motor&) -> Motor& = delete;
    auto operator=(Motor&&) noexcept -> Motor& = default;
    
    [[nodiscard]] static auto init() -> Result<Motor>;
    auto set_throttle(float throttle) -> std::error_code;

private:
    Motor(auto comp_value_at_min_throttle, auto comp_value_at_max_throttle) :
        _comp_value_at_min_throttle{comp_value_at_min_throttle},
        _comp_value_at_max_throttle{comp_value_at_max_throttle} {}

    const uint32_t _comp_value_at_min_throttle {0};
    const uint32_t _comp_value_at_max_throttle {0};
};

}

/*------------------------------------------------------------------------------------------------*/

namespace motor {

template<pwm::PWMInstance PWM, esc::ESCDefinition ESC>
auto Motor<PWM, ESC>::init() -> Result<Motor> {
    const auto pwm_freq = PWM::get_frequency();
    const auto pwm_period_us = float(1'000'000.0 / pwm_freq);

    const auto period_per_count = pwm_period_us / PWM::get_max_compare_value();

    auto min_comp_value = static_cast<uint32_t>(ceilf(ESC::MIN_PULSE_WIDTH.count() / period_per_count));
    auto max_comp_value = static_cast<uint32_t>(floorf(ESC::MAX_PULSE_WIDTH.count() / period_per_count));

    return {Motor<PWM, ESC>(min_comp_value, max_comp_value), StatusCode::Ok};
}

/*------------------------------------------------------------------------------------------------*/

/// @brief Set the mottor speed as a percentage of its maximum.
///
template<pwm::PWMInstance PWM, esc::ESCDefinition ESC>
auto Motor<PWM, ESC>::set_throttle(const float throttle) -> std::error_code {
    if(100.0F < throttle) {
        return StatusCode::InvalidThrottle;
    }

    const float scaler = (_comp_value_at_max_throttle - _comp_value_at_min_throttle) / 100.0F;
    const uint32_t compare_value = (throttle * scaler) + _comp_value_at_min_throttle;

    PWM::set_compare_value(compare_value);
    return StatusCode::Ok;
}

}

/*------------------------------------------------------------------------------------------------*/
