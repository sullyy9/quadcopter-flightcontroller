////////////////////////////////////////////////////////////////////////////////////////////////////
/// @author  Ryan Sullivan (ryansullivan@googlemail.com)
///
/// @file    gpio_stm32f303.cpp
/// @brief   GPIO driver module for the STM32F303.
////////////////////////////////////////////////////////////////////////////////////////////////////

#include <stm32f3xx.h>
#include <variant>
#include <functional>

#include "stm32f3xx_ll_gpio.h"

#include "gpio.hpp"
#include "gpio_stm32f303.hpp"


/*------------------------------------------------------------------------------------------------*/
// Module static function declarations.
/*------------------------------------------------------------------------------------------------*/

static auto port_mask(gpio::Pin pin) noexcept -> GPIO_TypeDef*;
static constexpr auto pin_mask(gpio::Pin pin) noexcept -> decltype(LL_GPIO_InitTypeDef::Pin);

static auto get_output_type(gpio::OutputType output_type) noexcept -> uint32_t;
static auto get_pull(gpio::Pull pull) noexcept -> uint32_t;


/*------------------------------------------------------------------------------------------------*/
// Class method definitions.
/*------------------------------------------------------------------------------------------------*/

auto gpio::GPIOSTM32F303::init(const Pin pin, const Config& config) noexcept -> std::error_code {

    LL_GPIO_InitTypeDef gpio_config {};
    LL_GPIO_StructInit(&gpio_config);

    auto status = std::visit(Overload{
        [&gpio_config](const Input& cfg) {
            gpio_config.Mode = LL_GPIO_MODE_INPUT; 
            gpio_config.Pull = get_pull(cfg.pull.value_or(Pull::None));
            return StatusCode::Ok;
        },
        [&gpio_config](const Output& cfg) {
            gpio_config.Mode = LL_GPIO_MODE_OUTPUT; 
            gpio_config.OutputType = get_output_type(cfg.output_type);
            gpio_config.Pull = get_pull(cfg.pull.value_or(Pull::None));
            return StatusCode::Ok;
        },
        [&gpio_config](const Analog&) {
            gpio_config.Mode = LL_GPIO_MODE_ANALOG;
            return StatusCode::Ok;
        },
        [&gpio_config](const AltFunction& cfg) {
            if(cfg.function > 15) {
                return StatusCode::InvalidAltFunction;
            }

            gpio_config.Mode = LL_GPIO_MODE_ALTERNATE;
            gpio_config.Pull = get_pull(cfg.pull.value_or(Pull::None));
            gpio_config.OutputType = get_output_type(cfg.output_type);
            gpio_config.Alternate = cfg.function;
            return StatusCode::Ok;
        },
    }, config);

    if(status != StatusCode::Ok) {
        return status;
    }

    gpio_config.Pin = pin_mask(pin);
    gpio_config.Speed = LL_GPIO_SPEED_FREQ_HIGH;

    if(LL_GPIO_Init(port_mask(pin), &gpio_config) != SUCCESS) {
        return StatusCode::UnderlyingDriverError;
    }

    return StatusCode::Ok;
}

/*------------------------------------------------------------------------------------------------*/

auto gpio::GPIOSTM32F303::set(const Pin pin) noexcept -> void {
    LL_GPIO_SetOutputPin(port_mask(pin), pin_mask(pin));
}

/*------------------------------------------------------------------------------------------------*/

auto gpio::GPIOSTM32F303::reset(const Pin pin) noexcept -> void {
    LL_GPIO_ResetOutputPin(port_mask(pin), pin_mask(pin));
}

/*------------------------------------------------------------------------------------------------*/

auto gpio::GPIOSTM32F303::is_set(const Pin pin) noexcept -> bool {
    return LL_GPIO_IsInputPinSet(port_mask(pin), pin_mask(pin)) == 0;
}

/*------------------------------------------------------------------------------------------------*/

auto gpio::GPIOSTM32F303::is_reset(const Pin pin) noexcept -> bool {
    return LL_GPIO_IsInputPinSet(port_mask(pin), pin_mask(pin)) != 0;
}

/*------------------------------------------------------------------------------------------------*/
// Module static function definitions.
/*------------------------------------------------------------------------------------------------*/

static auto port_mask(const gpio::Pin pin) noexcept -> GPIO_TypeDef* {
    return reinterpret_cast<GPIO_TypeDef*>(GPIOA_BASE + ((static_cast<uint32_t>(pin) / 16) * 0x400));
}

/*------------------------------------------------------------------------------------------------*/

static constexpr auto pin_mask(const gpio::Pin pin) noexcept -> decltype(LL_GPIO_InitTypeDef::Pin) {
    return 1 << (static_cast<uint32_t>(pin) % 16);
}

/*------------------------------------------------------------------------------------------------*/

static auto get_output_type(const gpio::OutputType output_type) noexcept -> uint32_t {
    switch(output_type) {
        default: [[fallthrough]];
        case gpio::OutputType::PushPull: return LL_GPIO_OUTPUT_PUSHPULL;
        case gpio::OutputType::OpenDrain: return LL_GPIO_OUTPUT_OPENDRAIN;
    }
}

/*------------------------------------------------------------------------------------------------*/

static auto get_pull(const gpio::Pull pull) noexcept -> uint32_t {
    switch(pull) {
        default: [[fallthrough]];
        case gpio::Pull::None: return LL_GPIO_PULL_NO;
        case gpio::Pull::Up: return LL_GPIO_PULL_UP;
        case gpio::Pull::Down: return LL_GPIO_PULL_DOWN;
    }
}

/*------------------------------------------------------------------------------------------------*/
