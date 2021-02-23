/*----------------------------------------------------------------------------*/
/*
    Ryan Sullivan

    Module Name     :
    Description     :
*/
/*----------------------------------------------------------------------------*/

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "stm32f3xx_ll_gpio.h"

#include "port.h"

/*----------------------------------------------------------------------------*/
/*-constant-definitions-------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-exported-variables---------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-static-variables-----------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-forward-declarations-------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-exported-functions---------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*
 * @brief           initialise a pin to the specified mode
 * @param port_pin: port and pin to be initialised. refer to port.h for
 *                  possible arguments
 * @param mode:     mode pin is to be initialised to. refer to port.h
 *                  for possible arguments
 * @retval          none
 */
void port_initialise_pin( uint8_t port_pin, uint8_t mode, uint32_t alternate_function )
{
    LL_GPIO_InitTypeDef gpio_initialisation_structure;
    gpio_initialisation_structure.Alternate     = alternate_function;
    gpio_initialisation_structure.Pin           = ( 1 << ( port_pin % 16 ) );
    gpio_initialisation_structure.Speed         = LL_GPIO_SPEED_FREQ_HIGH;

    switch( mode )
    {
        case PORT_MODE_ANALOG :
        {
            gpio_initialisation_structure.Mode          = LL_GPIO_MODE_ANALOG;
            gpio_initialisation_structure.OutputType    = LL_GPIO_OUTPUT_PUSHPULL;
            gpio_initialisation_structure.Pull          = LL_GPIO_PULL_NO;
            break;
        }

        case PORT_MODE_FLOATING :
        {
            gpio_initialisation_structure.Mode          = LL_GPIO_MODE_INPUT;
            gpio_initialisation_structure.OutputType    = LL_GPIO_OUTPUT_PUSHPULL;
            gpio_initialisation_structure.Pull          = LL_GPIO_PULL_NO;
            break;
        }

        case PORT_MODE_INPUT_PULLDOWN :
        {
            gpio_initialisation_structure.Mode          = LL_GPIO_MODE_INPUT;
            gpio_initialisation_structure.OutputType    = LL_GPIO_OUTPUT_PUSHPULL;
            gpio_initialisation_structure.Pull          = LL_GPIO_PULL_DOWN;
            break;
        }

        case PORT_MODE_INPUT_PULLUP :
        {
            gpio_initialisation_structure.Mode          = LL_GPIO_MODE_INPUT;
            gpio_initialisation_structure.OutputType    = LL_GPIO_OUTPUT_PUSHPULL;
            gpio_initialisation_structure.Pull          = LL_GPIO_PULL_UP;
            break;
        }

        case PORT_MODE_OPEN_DRAIN :
        {
            gpio_initialisation_structure.Mode          = LL_GPIO_MODE_OUTPUT;
            gpio_initialisation_structure.OutputType    = LL_GPIO_OUTPUT_OPENDRAIN;
            gpio_initialisation_structure.Pull          = LL_GPIO_PULL_NO;
            break;
        }

        case PORT_MODE_PUSH_PULL :
        {
            gpio_initialisation_structure.Mode          = LL_GPIO_MODE_OUTPUT;
            gpio_initialisation_structure.OutputType    = LL_GPIO_OUTPUT_PUSHPULL;
            gpio_initialisation_structure.Pull          = LL_GPIO_PULL_NO;
            break;
        }

        case PORT_MODE_ALT_OPEN_DRAIN :
        {
            gpio_initialisation_structure.Mode          = LL_GPIO_MODE_ALTERNATE;
            gpio_initialisation_structure.OutputType    = LL_GPIO_OUTPUT_OPENDRAIN;
            gpio_initialisation_structure.Pull          = LL_GPIO_PULL_NO;
            break;
        }

        case PORT_MODE_ALT_OUTPUT :
        {
            gpio_initialisation_structure.Mode          = LL_GPIO_MODE_ALTERNATE;
            gpio_initialisation_structure.OutputType    = LL_GPIO_OUTPUT_PUSHPULL;
            gpio_initialisation_structure.Pull          = LL_GPIO_PULL_NO;
            break;
        }

        default:
        {
            gpio_initialisation_structure.Mode          = LL_GPIO_MODE_ANALOG;
            gpio_initialisation_structure.OutputType    = LL_GPIO_OUTPUT_PUSHPULL;
            gpio_initialisation_structure.Pull          = LL_GPIO_PULL_NO;
            break;
        }
    }

    LL_GPIO_Init
    (
        (GPIO_TypeDef*)( GPIOA_BASE + ( ( port_pin / 16 ) * 0x400 ) ),
        &gpio_initialisation_structure
    );
}

/*----------------------------------------------------------------------------*/

void port_set( uint32_t port_pin )
{
    LL_GPIO_SetOutputPin
    (
        (GPIO_TypeDef*)( GPIOA_BASE + ( ( port_pin / 16 ) * 0x400 ) ),
        ( 1 << ( port_pin % 16 ) )
    );
}

/*----------------------------------------------------------------------------*/

void port_clear( uint32_t port_pin )
{
    LL_GPIO_ResetOutputPin
    (
        (GPIO_TypeDef*)( GPIOA_BASE + ( ( port_pin / 16 ) * 0x400 ) ),
        ( 1 << ( port_pin % 16 ) )
    );
}

/*----------------------------------------------------------------------------*/
/*-static-functions-----------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-end-of-module--------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
