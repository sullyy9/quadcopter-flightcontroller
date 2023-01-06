////////////////////////////////////////////////////////////////////////////////////////////////////
/// @author  Ryan Sullivan (ryansullivan@googlemail.com)
/// 
/// @file    port.cpp
/// @brief   Module for controlling GPIO pins.
/// 
/// @date    2021-04-09
////////////////////////////////////////////////////////////////////////////////////////////////////

#include <optional>

#include "stm32f3xx_ll_gpio.h"

#include "port.hpp"

/*------------------------------------------------------------------------------------------------*/

static auto port_mask(const port::Pin pin) -> GPIO_TypeDef*;
static constexpr auto pin_mask(const port::Pin pin) -> decltype(LL_GPIO_InitTypeDef::Pin);

/*------------------------------------------------------------------------------------------------*/

/// @brief                    Initialise a pin to the specified mode.
/// @param pin                Pin to be initialised.
/// @param mode               Mode the pin will be initialised to.
/// @param alternate_function Alternate function of the pin.
/// 
void port::initialise_pin(const Pin pin, const Mode mode, const uint32_t alternate_function)
{
    LL_GPIO_InitTypeDef gpio_pin;
    gpio_pin.Alternate = alternate_function;
    gpio_pin.Pin       = (1 << (static_cast<uint32_t>(pin) % 16));
    gpio_pin.Speed     = LL_GPIO_SPEED_FREQ_HIGH;

    using enum Mode;
    switch(mode) {
        case ANALOG: {
            gpio_pin.Mode       = LL_GPIO_MODE_ANALOG;
            gpio_pin.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
            gpio_pin.Pull       = LL_GPIO_PULL_NO;
            break;
        }

        case FLOATING: {
            gpio_pin.Mode       = LL_GPIO_MODE_INPUT;
            gpio_pin.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
            gpio_pin.Pull       = LL_GPIO_PULL_NO;
            break;
        }

        case INPUT_PULLDOWN: {
            gpio_pin.Mode       = LL_GPIO_MODE_INPUT;
            gpio_pin.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
            gpio_pin.Pull       = LL_GPIO_PULL_DOWN;
            break;
        }

        case INPUT_PULLUP: {
            gpio_pin.Mode       = LL_GPIO_MODE_INPUT;
            gpio_pin.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
            gpio_pin.Pull       = LL_GPIO_PULL_UP;
            break;
        }

        case OPEN_DRAIN: {
            gpio_pin.Mode       = LL_GPIO_MODE_OUTPUT;
            gpio_pin.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
            gpio_pin.Pull       = LL_GPIO_PULL_NO;
            break;
        }

        case PUSH_PULL: {
            gpio_pin.Mode       = LL_GPIO_MODE_OUTPUT;
            gpio_pin.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
            gpio_pin.Pull       = LL_GPIO_PULL_NO;
            break;
        }

        case ALT_OPEN_DRAIN: {
            gpio_pin.Mode       = LL_GPIO_MODE_ALTERNATE;
            gpio_pin.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
            gpio_pin.Pull       = LL_GPIO_PULL_NO;
            break;
        }

        case ALT_OUTPUT: {
            gpio_pin.Mode       = LL_GPIO_MODE_ALTERNATE;
            gpio_pin.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
            gpio_pin.Pull       = LL_GPIO_PULL_NO;
            break;
        }

        default: {
            gpio_pin.Mode       = LL_GPIO_MODE_ANALOG;
            gpio_pin.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
            gpio_pin.Pull       = LL_GPIO_PULL_NO;
            break;
        }
    }

    LL_GPIO_Init(port_mask(pin), &gpio_pin);
}

/*------------------------------------------------------------------------------------------------*/

/// @brief     Set a pin to a high logic level.
/// @param pin Pin to set.
/// 
void port::set(const Pin pin)
{
    LL_GPIO_SetOutputPin(port_mask(pin), pin_mask(pin));
}

/*------------------------------------------------------------------------------------------------*/

/// @brief     Set a pin to a low logic level.
/// @param pin Pin to set.
/// 
void port::clear(const Pin pin)
{
    LL_GPIO_ResetOutputPin(port_mask(pin), pin_mask(pin));
}

/*------------------------------------------------------------------------------------------------*/

/// @brief           Read a pins logic level.
/// @param pin       Pin to read.
/// @return uint32_t Value of the pin. 1 or 0.
/// 
uint32_t port::read(const Pin pin)
{
    uint32_t port_value;
    port_value = LL_GPIO_ReadInputPort(port_mask(pin));

    if((port_value & pin_mask(pin)) == 0)
    {
        return (0);
    }
    else
    {
        return (1);
    }
}

/*------------------------------------------------------------------------------------------------*/

static auto port_mask(const port::Pin pin) -> GPIO_TypeDef* {
    return reinterpret_cast<GPIO_TypeDef*>(GPIOA_BASE + ((static_cast<uint32_t>(pin) / 16) * 0x400));
}

/*------------------------------------------------------------------------------------------------*/

static constexpr auto pin_mask(const port::Pin pin) -> decltype(LL_GPIO_InitTypeDef::Pin) {
    return 1 << (static_cast<uint32_t>(pin) % 16);
}
