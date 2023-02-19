////////////////////////////////////////////////////////////////////////////////////////////////////
/// @author  Ryan Sullivan (ryansullivan@googlemail.com)
///
/// @file    pwm.cpp
/// @brief   Module for controlling a timer peripheral in PWM mode.
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "stm32f3xx_ll_tim.h"
#include "stm32f3xx_ll_rcc.h"
#include <optional>

#include "pwm.hpp"

auto pwm::Timer2::init(const PWMConfig& config) -> std::error_code {

    // Using timer 2.
    // 4 channels on pins A0 - A3.
    // Clocked from APB1.
    LL_RCC_ClocksTypeDef clocks;
    LL_RCC_GetSystemClocksFreq(&clocks);

    const auto source_frequency = clocks.PCLK1_Frequency;

    // Calculate the reload value as high as possible to maximize the resolution of the compare
    // registers.
    auto prescaler = PRESCALER_REGISTER_VALUE_MIN + 1;
    while(source_frequency / (RELOAD_REGISTER_VALUE_MAX * prescaler) > config.frequency) {
        if(prescaler >= (PRESCALER_REGISTER_VALUE_MAX + 1)) {
            return StatusCode::InvalidFrequency;
        }
        prescaler++;
    }

    auto reload_value = RELOAD_REGISTER_VALUE_MAX;
    while(source_frequency / (reload_value * prescaler) > config.frequency) {
        if(reload_value <= RELOAD_REGISTER_VALUE_MIN) {
            return StatusCode::InvalidFrequency;
        }
        reload_value--;
    }

    LL_TIM_InitTypeDef timer {
        .Prescaler = static_cast<uint16_t>(prescaler - 1),
        .CounterMode = LL_TIM_COUNTERMODE_UP,
        .Autoreload = reload_value,
        .ClockDivision = LL_TIM_CLOCKDIVISION_DIV1,
        .RepetitionCounter = 0,
    };

    if(LL_TIM_Init(TIM2, &timer) == ERROR) {
        return StatusCode::ConfigError;
    }

    LL_TIM_OC_InitTypeDef compare {
        .OCMode = LL_TIM_OCMODE_PWM1,
        .OCState = LL_TIM_OCSTATE_ENABLE,
        .OCNState = LL_TIM_OCSTATE_DISABLE,
        .CompareValue = 0,
        .OCPolarity = LL_TIM_OCPOLARITY_HIGH,
        .OCNPolarity = LL_TIM_OCPOLARITY_LOW,
        .OCIdleState = LL_TIM_OCIDLESTATE_LOW,
        .OCNIdleState = LL_TIM_OCIDLESTATE_LOW,
    };

    if(LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH1, &compare) == ERROR ||
       LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH2, &compare) == ERROR ||
       LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH3, &compare) == ERROR ||
       LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH4, &compare) == ERROR)
    {
        return StatusCode::ConfigError;
    }

    LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH3);
    LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH4);
    LL_TIM_EnableARRPreload(TIM2);

    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH3);
    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH4);

    LL_TIM_GenerateEvent_UPDATE(TIM2);
    LL_TIM_EnableCounter(TIM2);

    return StatusCode::Ok;
}

/*------------------------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------------------------*/

template<pwm::PWMChannel Channel>
auto pwm::Timer2PWM<Channel>::set_duty_cycle(const uint32_t duty_cycle) -> std::error_code {
    if(duty_cycle > 100) {
        return StatusCode::InvalidDutyCycle;
    }
    
    const auto max_reload = static_cast<float>(LL_TIM_GetAutoReload(TIM2));
    const auto duty_cycle_f = static_cast<float>(duty_cycle);

    const auto comp_value = static_cast<uint32_t>((max_reload / 100) * duty_cycle_f);
    switch(static_cast<uint32_t>(Channel)) {
        case LL_TIM_CHANNEL_CH1: LL_TIM_OC_SetCompareCH1(TIM2, comp_value); break;
        case LL_TIM_CHANNEL_CH2: LL_TIM_OC_SetCompareCH2(TIM2, comp_value); break;
        case LL_TIM_CHANNEL_CH3: LL_TIM_OC_SetCompareCH3(TIM2, comp_value); break;
        case LL_TIM_CHANNEL_CH4: LL_TIM_OC_SetCompareCH4(TIM2, comp_value); break;
        default: return StatusCode::InvalidInstance;
    }

    LL_TIM_GenerateEvent_UPDATE(TIM2);
    return StatusCode::Ok;
}

/*------------------------------------------------------------------------------------------------*/

template<pwm::PWMChannel Channel>
auto pwm::Timer2PWM<Channel>::set_compare_value(const uint32_t value) -> std::error_code {
    if(value > LL_TIM_GetAutoReload(TIM2)) {
        return StatusCode::InvalidCompareValue;
    }

    switch(static_cast<uint32_t>(Channel)) {
        case LL_TIM_CHANNEL_CH1: LL_TIM_OC_SetCompareCH1(TIM2, value); break;
        case LL_TIM_CHANNEL_CH2: LL_TIM_OC_SetCompareCH2(TIM2, value); break;
        case LL_TIM_CHANNEL_CH3: LL_TIM_OC_SetCompareCH3(TIM2, value); break;
        case LL_TIM_CHANNEL_CH4: LL_TIM_OC_SetCompareCH4(TIM2, value); break;
        default: return StatusCode::InvalidInstance;
    }

    LL_TIM_GenerateEvent_UPDATE(TIM2);
    return StatusCode::Ok;
}

/*------------------------------------------------------------------------------------------------*/
// Error handling.
/*------------------------------------------------------------------------------------------------*/

struct PWMStatusCategory : std::error_category {
    [[nodiscard]] auto name() const noexcept -> const char* override;
    [[nodiscard]] auto message(int status) const -> std::string override;
};
const PWMStatusCategory PWM_STATUS_CATEGORY {};

/*------------------------------------------------------------------------------------------------*/
 
auto PWMStatusCategory::name() const noexcept -> const char* {
    return "PWM";
}

/*------------------------------------------------------------------------------------------------*/

auto PWMStatusCategory::message(int status) const -> std::string {
    using enum pwm::StatusCode;
    switch (static_cast<pwm::StatusCode>(status)) {
        case Ok:                  return "Ok";
        case Unimplemented:       return "Use of unimplemented feature";
        case ConfigError:         return "Configuration error";
        case InvalidFrequency:    return "Invalid frequency";
        case InvalidInstance:     return "Invalid channel";
        case InvalidDutyCycle:    return "Invalid duty cycle";
        case InvalidCompareValue: return "Invalid compare value";
        default:                  return "Unknown";
    }
}

/*------------------------------------------------------------------------------------------------*/

auto pwm::make_error_code(StatusCode err) -> std::error_code {
    return {static_cast<int>(err), PWM_STATUS_CATEGORY};
}

/*------------------------------------------------------------------------------------------------*/
