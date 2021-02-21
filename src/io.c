/*----------------------------------------------------------------------------*/
/*
    Ryan Sullivan

    Module Name     :   io.c
    Description     :   functions for controlling peripherals
*/
/*----------------------------------------------------------------------------*/

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "stm32f3xx_ll_gpio.h"

#include "io.h"
#include "port.h"
#include "commonio.h"
#include "comms.h"

/*----------------------------------------------------------------------------*/
/*-constant-definitions-------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

#define DEBUG_TX    PORT_C4
#define DEBUG_RX    PORT_C5

#define LED1        PORT_E15

/*----------------------------------------------------------------------------*/
/*-exported-variables---------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-static-variables-----------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

static uint32_t led_timer   = 0;
static bool     led_on      = false;

/*----------------------------------------------------------------------------*/
/*-forward-declarations-------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

void    initialise_gpio( void );
void    initialise_usart( void );
void    initialise_io_pin( uint8_t port_pin, uint8_t mode, uint32_t alternate_function );

/*----------------------------------------------------------------------------*/
/*-exported-functions---------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

void io_initialise( void )
{

    commonio_initialise_clocks( );

    initialise_gpio( );
    initialise_usart( );

    //comms_initialise( );

}

/*----------------------------------------------------------------------------*/

void io_1ms_poll( void )
{
    if( led_timer >= 1000 )
    {
        led_timer = 0;
        if( led_on )
        {
            port_clear( LED1 );
            led_on = false;
        }
        else
        {
            port_set( LED1 );
            led_on = true;
        }
    }
    led_timer++;
}

/*----------------------------------------------------------------------------*/
/*-static-functions-----------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*
 * @brief           initialise any gpio pins
 * @param           none
 * @retval          none
 */
void initialise_gpio( void )
{
    initialise_io_pin( LED1, PORT_MODE_PUSH_PULL, 0 );
    port_clear( LED1 );
}

/*----------------------------------------------------------------------------*/

/*
 * @brief           initialise any pins used by usart
 * @param           none
 * @retval          none
 */
void initialise_usart( void )
{

    initialise_io_pin( DEBUG_TX, PORT_MODE_ALT_OUTPUT, 7 );
    initialise_io_pin( DEBUG_TX, PORT_MODE_INPUT_PULLUP, 7 );

}

/*----------------------------------------------------------------------------*/

/*
 * @brief           initialise a pin to the specified mode
 * @param port_pin: port and pin to be initialised. refer to port.h for
 *                  possible arguments
 * @param mode:     mode pin is to be initialised to. refer to port.h
 *                  for possible arguments
 * @retval          none
 */
void initialise_io_pin( uint8_t port_pin, uint8_t mode, uint32_t alternate_function )
{
    uint32_t        pin     = ( 1 << ( port_pin % 16 ) );
    GPIO_TypeDef    *port   = NULL;

    if( port_pin <= PORT_A15 )
    {
        port = GPIOA;
    }
    else if( port_pin <= PORT_B15 )
    {
        port = GPIOB;
    }
    else if( port_pin <= PORT_C15 )
    {
        port = GPIOC;
    }
    else if( port_pin <= PORT_D15 )
    {
        port = GPIOD;
    }
    else if( port_pin <= PORT_E15 )
    {
        port = GPIOE;
    }
    else if( port_pin <= PORT_F10 )
    {
        port = GPIOF;
    }
    else
    {

    }

    switch( mode )
    {
        case PORT_MODE_ANALOG :
        {
            LL_GPIO_SetPinMode  ( port, pin, LL_GPIO_MODE_ANALOG );
            LL_GPIO_SetPinSpeed ( port, pin, LL_GPIO_SPEED_FREQ_HIGH );
            break;
        }

        case PORT_MODE_FLOATING :
        {
            LL_GPIO_SetPinMode  ( port, pin, LL_GPIO_MODE_INPUT );
            LL_GPIO_SetPinPull  ( port, pin, LL_GPIO_PULL_NO );
            LL_GPIO_SetPinSpeed ( port, pin, LL_GPIO_SPEED_FREQ_HIGH );
            break;
        }

        case PORT_MODE_INPUT_PULLDOWN :
        {
            LL_GPIO_SetPinMode  ( port, pin, LL_GPIO_MODE_INPUT );
            LL_GPIO_SetPinPull  ( port, pin, LL_GPIO_PULL_DOWN );
            LL_GPIO_SetPinSpeed ( port, pin, LL_GPIO_SPEED_FREQ_HIGH );
            break;
        }

        case PORT_MODE_INPUT_PULLUP :
        {
            LL_GPIO_SetPinMode  ( port, pin, LL_GPIO_MODE_INPUT );
            LL_GPIO_SetPinPull  ( port, pin, LL_GPIO_PULL_UP );
            LL_GPIO_SetPinSpeed ( port, pin, LL_GPIO_SPEED_FREQ_HIGH );
            break;
        }

        case PORT_MODE_OPEN_DRAIN :
        {
            LL_GPIO_SetPinMode      ( port, pin, LL_GPIO_MODE_OUTPUT );
            LL_GPIO_SetPinOutputType( port, pin, LL_GPIO_OUTPUT_OPENDRAIN );
            LL_GPIO_SetPinSpeed     ( port, pin, LL_GPIO_SPEED_FREQ_HIGH );
            break;
        }

        case PORT_MODE_PUSH_PULL :
        {
            LL_GPIO_SetPinMode      ( port, pin, LL_GPIO_MODE_OUTPUT );
            LL_GPIO_SetPinOutputType( port, pin, LL_GPIO_OUTPUT_PUSHPULL );
            LL_GPIO_SetPinSpeed     ( port, pin, LL_GPIO_SPEED_FREQ_HIGH );
            break;
        }

        case PORT_MODE_ALT_OPEN_DRAIN :
        {
            LL_GPIO_SetPinMode      ( port, pin, LL_GPIO_MODE_ALTERNATE );
            LL_GPIO_SetPinOutputType( port, pin, LL_GPIO_OUTPUT_OPENDRAIN );
            LL_GPIO_SetPinSpeed     ( port, pin, LL_GPIO_SPEED_FREQ_HIGH );

            if( ( port_pin % 16 ) <= 7 )
            {
                LL_GPIO_SetAFPin_0_7( port, pin, alternate_function );
            }
            else
            {
                LL_GPIO_SetAFPin_8_15( port, pin, alternate_function );
            }
            break;
        }

        case PORT_MODE_ALT_OUTPUT :
        {
            LL_GPIO_SetPinMode      ( port, pin, LL_GPIO_MODE_ALTERNATE );
            LL_GPIO_SetPinOutputType( port, pin, LL_GPIO_OUTPUT_PUSHPULL );
            LL_GPIO_SetPinSpeed     ( port, pin, LL_GPIO_SPEED_FREQ_HIGH );

            if( ( port_pin % 16 ) <= 7 )
            {
                LL_GPIO_SetAFPin_0_7( port, pin, alternate_function );
            }
            else
            {
                LL_GPIO_SetAFPin_8_15( port, pin, alternate_function );
            }
            break;
        }

        default:
        {
            break;
        }
    }
}

/*----------------------------------------------------------------------------*/
/*-end-of-module--------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
