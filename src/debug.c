/*----------------------------------------------------------------------------*/
/*
    Ryan Sullivan

    Module Name     : debug.c
    Description     : functions for printing debug messages
*/
/*----------------------------------------------------------------------------*/

#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>

#include "stm32f3xx.h"
#include "core_cm4.h"

#include "debug.h"

#include "usart.h"

/*----------------------------------------------------------------------------*/
/*-constant-definitions-------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

#define CORE_SPEED 48000000

/*----------------------------------------------------------------------------*/
/*-exported-variables---------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-static-variables-----------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

static uint32_t stopwatch_start = 0;
static uint32_t stopwatch_stop  = 0;

/*----------------------------------------------------------------------------*/
/*-forward-declarations-------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

void    print_character( char character );
void    printf_number( uint32_t number );

/*----------------------------------------------------------------------------*/
/*-exported-functions---------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

void debug_printf( char const *string_ptr, ... )
{
    va_list argument_list;

    va_start( argument_list, string_ptr );
    while( *string_ptr != 0 )
    {
        if( *string_ptr != '%' )
        {
            print_character( *string_ptr );
        }
        else
        {
            /*
             * get the next character to find the format of the argument
             */
            string_ptr++;

            switch( *string_ptr )
            {
                case 'c':
                {
                    print_character( va_arg( argument_list, int ) );
                    break;
                }

                case 's':
                {
                    char *ptr = va_arg( argument_list, char* );
                    while( *ptr != 0 )
                    {
                        print_character( *ptr );
                        ptr++;
                    }
                    break;
                }

                case 'd':
                {
                    int32_t number = va_arg( argument_list, int32_t );
                    if( number < 0 )
                    {
                        print_character( '-' );
                        number = 0 - number;
                    }
                    printf_number( (uint32_t)number );
                    break;
                }

                case 'u':
                {
                    printf_number( va_arg( argument_list, uint32_t ) );
                    break;
                }

                default:
                {
                    break;
                }
            }
        }

        string_ptr++;
    }
}

/*----------------------------------------------------------------------------*/

void debug_stopwatch_initialise( void )
{
    /*
     * Setup the CPU cycle counter
     */
    SET_BIT( CoreDebug->DEMCR, CoreDebug_DEMCR_TRCENA_Msk );
    SET_BIT( DWT->CTRL, DWT_CTRL_CYCCNTENA_Msk );
}

/*----------------------------------------------------------------------------*/

void debug_stopwatch_start( void )
{
    stopwatch_start = 0;
    WRITE_REG( DWT->CYCCNT, 0 );
}

/*----------------------------------------------------------------------------*/

/*
 * @brief       Return the time elapsed in nano seconds
 * @param       none
 * @retval      time elapsed (nS)
 */
uint32_t debug_stopwatch_stop( void )
{
    stopwatch_stop = READ_REG( DWT->CYCCNT );

    return( ( stopwatch_stop - stopwatch_start ) / ( (double)CORE_SPEED / 1000000000 ) );
}

/*----------------------------------------------------------------------------*/
/*-static-functions-----------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

void print_character( char character )
{
    while( usart1_tx_free( ) == 0 );
    usart1_tx_byte( character );
}

/*----------------------------------------------------------------------------*/

void printf_number( uint32_t number )
{
    char buffer[ 10 ];
    uint8_t buffer_ptr = 0;

    do
    {
        buffer[ buffer_ptr ] = ( '0' + ( number % 10 ) );
        number = number / 10;
        buffer_ptr++;
    }
    while( number != 0 );

    while( buffer_ptr > 0 )
    {
        buffer_ptr--;
        print_character( buffer[ buffer_ptr ] );
    }
}

/*----------------------------------------------------------------------------*/
/*-end-of-module--------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
