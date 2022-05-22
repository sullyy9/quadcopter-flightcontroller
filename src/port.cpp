/**
 * -------------------------------------------------------------------------------------------------
 * @author  Ryan Sullivan (ryansullivan@googlemail.com)
 *
 * @file    port.c
 * @brief   Module for controlling GPIO pins.
 *
 * @date    2021-04-09
 * -------------------------------------------------------------------------------------------------
 */

#include "stm32f3xx_ll_gpio.h"

#include "port.hpp"

using namespace port;
/*------------------------------------------------------------------------------------------------*/
/*-constant-definitions---------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------------------*/
/*-exported-variables-----------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------------------*/
/*-static-variables-------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------------------*/
/*-forward-declarations---------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------------------*/
/*-exported-functions-----------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/**
 * @brief                    Initialise a pin to the specified mode.
 * @param pin                Pin to be initialised.
 * @param mode               Mode the pin will be initialised to.
 * @param alternate_function Alternate function of the pin.
 */
void port::initialise_pin(pin_t pin, pin_mode_t mode, uint32_t alternate_function)
{
    LL_GPIO_InitTypeDef gpio_pin;
    gpio_pin.Alternate = alternate_function;
    gpio_pin.Pin       = (1 << (pin % 16));
    gpio_pin.Speed     = LL_GPIO_SPEED_FREQ_HIGH;

    switch(mode)
    {
        case MODE_ANALOG:
        {
            gpio_pin.Mode       = LL_GPIO_MODE_ANALOG;
            gpio_pin.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
            gpio_pin.Pull       = LL_GPIO_PULL_NO;
            break;
        }

        case MODE_FLOATING:
        {
            gpio_pin.Mode       = LL_GPIO_MODE_INPUT;
            gpio_pin.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
            gpio_pin.Pull       = LL_GPIO_PULL_NO;
            break;
        }

        case MODE_INPUT_PULLDOWN:
        {
            gpio_pin.Mode       = LL_GPIO_MODE_INPUT;
            gpio_pin.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
            gpio_pin.Pull       = LL_GPIO_PULL_DOWN;
            break;
        }

        case MODE_INPUT_PULLUP:
        {
            gpio_pin.Mode       = LL_GPIO_MODE_INPUT;
            gpio_pin.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
            gpio_pin.Pull       = LL_GPIO_PULL_UP;
            break;
        }

        case MODE_OPEN_DRAIN:
        {
            gpio_pin.Mode       = LL_GPIO_MODE_OUTPUT;
            gpio_pin.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
            gpio_pin.Pull       = LL_GPIO_PULL_NO;
            break;
        }

        case MODE_PUSH_PULL:
        {
            gpio_pin.Mode       = LL_GPIO_MODE_OUTPUT;
            gpio_pin.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
            gpio_pin.Pull       = LL_GPIO_PULL_NO;
            break;
        }

        case MODE_ALT_OPEN_DRAIN:
        {
            gpio_pin.Mode       = LL_GPIO_MODE_ALTERNATE;
            gpio_pin.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
            gpio_pin.Pull       = LL_GPIO_PULL_NO;
            break;
        }

        case MODE_ALT_OUTPUT:
        {
            gpio_pin.Mode       = LL_GPIO_MODE_ALTERNATE;
            gpio_pin.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
            gpio_pin.Pull       = LL_GPIO_PULL_NO;
            break;
        }

        default:
        {
            gpio_pin.Mode       = LL_GPIO_MODE_ANALOG;
            gpio_pin.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
            gpio_pin.Pull       = LL_GPIO_PULL_NO;
            break;
        }
    }

    LL_GPIO_Init(PORT_MASK(pin), &gpio_pin);
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief     Set a pin to a high logic level.
 * @param pin Pin to set.
 */
void port::set(pin_t pin)
{
    LL_GPIO_SetOutputPin(PORT_MASK(pin), PIN_MASK(pin));
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief     Set a pin to a low logic level.
 * @param pin Pin to set.
 */
void port::clear(pin_t pin)
{
    LL_GPIO_ResetOutputPin(PORT_MASK(pin), PIN_MASK(pin));
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief           Read a pins logic level.
 * @param pin       Pin to read.
 * @return uint32_t Value of the pin. 1 or 0.
 */
uint32_t port::read(pin_t pin)
{
    uint32_t port_value;
    port_value = LL_GPIO_ReadInputPort(PORT_MASK(pin));

    if((port_value & PIN_MASK(pin)) == 0)
    {
        return (0);
    }
    else
    {
        return (1);
    }
}

/*------------------------------------------------------------------------------------------------*/
/*-static-functions-------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------------------*/
/*-end-of-module----------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/
