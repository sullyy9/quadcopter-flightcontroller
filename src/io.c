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

#define LEDNW       PORT_E8
#define LEDN        PORT_E9
#define LEDNE       PORT_E10
#define LEDE        PORT_E11
#define LEDSE       PORT_E12
#define LEDS        PORT_E13
#define LEDSW       PORT_E14
#define LEDW        PORT_E15

/*----------------------------------------------------------------------------*/
/*-exported-variables---------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-static-variables-----------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

static volatile uint32_t led_timer   = 0;
static volatile bool     led_n_on    = false;
static          bool     led_w_on    = false;

/*----------------------------------------------------------------------------*/
/*-forward-declarations-------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

void    initialise_gpio( void );
void    initialise_usart( void );

/*----------------------------------------------------------------------------*/
/*-exported-functions---------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

void io_initialise( void )
{

    commonio_initialise_clocks( );

    initialise_gpio( );
    initialise_usart( );

    comms_initialise( );

}

/*----------------------------------------------------------------------------*/

void io_toggle_w_led( void )
{
    if( led_w_on )
    {
        port_clear( LEDW );
        led_w_on = false;
    }
    else
    {
        port_set( LEDW );
        led_w_on = true;
    }
}

/*----------------------------------------------------------------------------*/

void io_1ms_poll( void )
{
    if( led_timer >= 200 )
    {
        led_timer = 0;
        if( led_n_on )
        {
            port_clear( LEDN );
            led_n_on = false;
        }
        else
        {
            port_set( LEDN );
            led_n_on = true;
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
    port_initialise_pin( LEDN, PORT_MODE_PUSH_PULL, 0 );
    port_initialise_pin( LEDW, PORT_MODE_PUSH_PULL, 0 );
}

/*----------------------------------------------------------------------------*/

/*
 * @brief           initialise any pins used by usart
 * @param           none
 * @retval          none
 */
void initialise_usart( void )
{
    port_initialise_pin( DEBUG_TX, PORT_MODE_ALT_OUTPUT, 7 );
    port_initialise_pin( DEBUG_RX, PORT_MODE_INPUT_PULLUP, 7 );
}

/*----------------------------------------------------------------------------*/
/*-end-of-module--------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
