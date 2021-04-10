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

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "stm32f3xx_ll_gpio.h"

#include "port.h"

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
void port_initialise_pin(pin_t pin, pin_mode_t mode, uint32_t alternate_function)
{
    LL_GPIO_InitTypeDef gpio_initialisation_structure;
    gpio_initialisation_structure.Alternate = alternate_function;
    gpio_initialisation_structure.Pin       = (1 << (pin % 16));
    gpio_initialisation_structure.Speed     = LL_GPIO_SPEED_FREQ_HIGH;

    switch(mode)
    {
        case PORT_MODE_ANALOG:
        {
            gpio_initialisation_structure.Mode       = LL_GPIO_MODE_ANALOG;
            gpio_initialisation_structure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
            gpio_initialisation_structure.Pull       = LL_GPIO_PULL_NO;
            break;
        }

        case PORT_MODE_FLOATING:
        {
            gpio_initialisation_structure.Mode       = LL_GPIO_MODE_INPUT;
            gpio_initialisation_structure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
            gpio_initialisation_structure.Pull       = LL_GPIO_PULL_NO;
            break;
        }

        case PORT_MODE_INPUT_PULLDOWN:
        {
            gpio_initialisation_structure.Mode       = LL_GPIO_MODE_INPUT;
            gpio_initialisation_structure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
            gpio_initialisation_structure.Pull       = LL_GPIO_PULL_DOWN;
            break;
        }

        case PORT_MODE_INPUT_PULLUP:
        {
            gpio_initialisation_structure.Mode       = LL_GPIO_MODE_INPUT;
            gpio_initialisation_structure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
            gpio_initialisation_structure.Pull       = LL_GPIO_PULL_UP;
            break;
        }

        case PORT_MODE_OPEN_DRAIN:
        {
            gpio_initialisation_structure.Mode       = LL_GPIO_MODE_OUTPUT;
            gpio_initialisation_structure.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
            gpio_initialisation_structure.Pull       = LL_GPIO_PULL_NO;
            break;
        }

        case PORT_MODE_PUSH_PULL:
        {
            gpio_initialisation_structure.Mode       = LL_GPIO_MODE_OUTPUT;
            gpio_initialisation_structure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
            gpio_initialisation_structure.Pull       = LL_GPIO_PULL_NO;
            break;
        }

        case PORT_MODE_ALT_OPEN_DRAIN:
        {
            gpio_initialisation_structure.Mode       = LL_GPIO_MODE_ALTERNATE;
            gpio_initialisation_structure.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
            gpio_initialisation_structure.Pull       = LL_GPIO_PULL_NO;
            break;
        }

        case PORT_MODE_ALT_OUTPUT:
        {
            gpio_initialisation_structure.Mode       = LL_GPIO_MODE_ALTERNATE;
            gpio_initialisation_structure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
            gpio_initialisation_structure.Pull       = LL_GPIO_PULL_NO;
            break;
        }

        default:
        {
            gpio_initialisation_structure.Mode       = LL_GPIO_MODE_ANALOG;
            gpio_initialisation_structure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
            gpio_initialisation_structure.Pull       = LL_GPIO_PULL_NO;
            break;
        }
    }

    LL_GPIO_Init(PORT_MASK(pin), &gpio_initialisation_structure);
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief     Set a pin to a high logic level.
 * @param pin Pin to set.
 */
void port_set(pin_t pin)
{
    LL_GPIO_SetOutputPin(PORT_MASK(pin), PIN_MASK(pin));
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief     Set a pin to a low logic level.
 * @param pin Pin to set.
 */
void port_clear(pin_t pin)
{
    LL_GPIO_ResetOutputPin(PORT_MASK(pin), PIN_MASK(pin));
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief           Read a pins logic level.
 * @param pin       Pin to read.
 * @return uint32_t Value of the pin. 1 or 0.
 */
uint32_t port_read(pin_t pin)
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
