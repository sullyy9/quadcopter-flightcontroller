////////////////////////////////////////////////////////////////////////////////////////////////////
/// @author  Ryan Sullivan (ryansullivan@googlemail.com)
///
/// @file    pwm.hpp
/// @brief   Module for controlling a timer peripheral in PWM mode.
////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once

#include <cstdint>
#include <tuple>
#include <optional>
#include <system_error>
#include <array>
#include <cmath>

#include "stm32f303xc.h"
#include "stm32f3xx_ll_tim.h"
#include "stm32f3xx_ll_rcc.h"

/*------------------------------------------------------------------------------------------------*/
// Error handling.
/*------------------------------------------------------------------------------------------------*/

namespace pwm {

enum class StatusCode: uint32_t {
    Ok,
    Unimplemented,
    InvalidFrequency,
    ConfigError,

    InvalidInstance,
    InvalidDutyCycle,
    InvalidCompareValue,
};

auto make_error_code(StatusCode) -> std::error_code;

}

namespace std {

template <>
struct is_error_code_enum<pwm::StatusCode> : true_type {};

}

/*------------------------------------------------------------------------------------------------*/
// Class declarations.
/*------------------------------------------------------------------------------------------------*/

namespace pwm {

enum class PWMChannel: uint32_t {
    MOTOR1 = LL_TIM_CHANNEL_CH1,
    MOTOR2 = LL_TIM_CHANNEL_CH2,
    MOTOR3 = LL_TIM_CHANNEL_CH3,
    MOTOR4 = LL_TIM_CHANNEL_CH4,
};

/*------------------------------------------------------------------------------------------------*/

struct PWMConfig {
    uint32_t frequency {1'000};
};

/*------------------------------------------------------------------------------------------------*/

template<typename T>
concept PWMInstance = requires(T instance, PWMConfig config, uint32_t duty_cycle, uint32_t compare_value) {
    {instance.set_compare_value(compare_value)} -> std::same_as<std::error_code>;
    {instance.get_max_compare_value()} -> std::same_as<uint32_t>;
};

/*------------------------------------------------------------------------------------------------*/

template<PWMChannel Channel>
struct Timer2PWM {
    Timer2PWM() = delete;
    Timer2PWM(Timer2PWM&) = delete;
    Timer2PWM(Timer2PWM&&) noexcept = delete;
    auto operator=(const Timer2PWM&) -> Timer2PWM& = delete;
    auto operator=(Timer2PWM&&) -> Timer2PWM& = delete;
    ~Timer2PWM() = delete;

    [[nodiscard]] static auto set_duty_cycle(uint32_t duty_cycle) -> std::error_code;
    [[nodiscard]] static auto set_compare_value(uint32_t value) -> std::error_code;

    [[nodiscard]] static auto get_frequency() -> uint32_t;
    [[nodiscard]] static auto get_max_compare_value() -> uint32_t;

};

using Motor1PWM = pwm::Timer2PWM<pwm::PWMChannel::MOTOR1>;
using Motor2PWM = pwm::Timer2PWM<pwm::PWMChannel::MOTOR2>;
using Motor3PWM = pwm::Timer2PWM<pwm::PWMChannel::MOTOR3>;
using Motor4PWM = pwm::Timer2PWM<pwm::PWMChannel::MOTOR4>;

static_assert(PWMInstance<Motor1PWM>);
static_assert(PWMInstance<Motor2PWM>);
static_assert(PWMInstance<Motor3PWM>);
static_assert(PWMInstance<Motor4PWM>);

/*------------------------------------------------------------------------------------------------*/

struct Timer2 {
    Timer2() = delete;
    Timer2(Timer2&) = delete;
    Timer2(Timer2&&) = delete;
    auto operator=(const Timer2&) -> Timer2& = delete;
    auto operator=(Timer2&&) -> Timer2& = delete;
    ~Timer2() = delete;

    [[nodiscard]] static auto init(const PWMConfig& config) -> std::error_code;

private:
    static constexpr uint32_t RELOAD_REGISTER_VALUE_MIN {0};
    static constexpr uint32_t RELOAD_REGISTER_VALUE_MAX {0xFFFF};

    static constexpr uint32_t PRESCALER_REGISTER_VALUE_MIN {0};
    static constexpr uint32_t PRESCALER_REGISTER_VALUE_MAX {0xFFFF};
};

/*------------------------------------------------------------------------------------------------*/

template<pwm::PWMChannel Channel>
auto pwm::Timer2PWM<Channel>::get_frequency() -> uint32_t {
    LL_RCC_ClocksTypeDef clocks;
    LL_RCC_GetSystemClocksFreq(&clocks);
    return clocks.PCLK1_Frequency / (LL_TIM_GetAutoReload(TIM2) * (LL_TIM_GetPrescaler(TIM2) + 1));
}

/*------------------------------------------------------------------------------------------------*/

template<pwm::PWMChannel Channel>
auto pwm::Timer2PWM<Channel>::get_max_compare_value() -> uint32_t {
    return LL_TIM_GetAutoReload(TIM2);
}


}

/*------------------------------------------------------------------------------------------------*/
