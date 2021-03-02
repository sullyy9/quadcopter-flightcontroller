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

#include "debug.h"

#include "usart.h"

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
