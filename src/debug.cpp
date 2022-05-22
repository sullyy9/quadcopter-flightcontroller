/**
 * -------------------------------------------------------------------------------------------------
 * @author  Ryan Sullivan (ryansullivan@googlemail.com)
 *
 * @file    debug.c
 * @brief   Module for debugging.
 *
 * @date    2021-04-04
 * -------------------------------------------------------------------------------------------------
 */

#include <cstdarg>

#include "stm32f3xx.h"
#include "core_cm4.h"

#include "usart.hpp"

#include "debug.hpp"

using namespace debug;
/*------------------------------------------------------------------------------------------------*/
/*-constant-definitions---------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

#define CORE_SPEED 48000000

/*------------------------------------------------------------------------------------------------*/
/*-exported-variables-----------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------------------*/
/*-static-variables-------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------------------*/
/*-forward-declarations---------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

void print_character(char character);
void print_number(uint32_t number);

/*------------------------------------------------------------------------------------------------*/
/*-exported-functions-----------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/**
 * @brief               Printf-like function for debugging.
 * @param string_ptr    String to print.
 * @param ...           Variable embedded in the string.
 */
void debug::printf(char const *string_ptr, ...)
{
    va_list argument_list;

    va_start(argument_list, string_ptr);
    while(*string_ptr != 0)
    {
        /*
         * Print a character or if its an embedded variable, process it.
         */
        if(*string_ptr != '%')
        {
            print_character(*string_ptr);
        }
        else
        {
            /*
             * Depending on the type of variable, process it differently.
             */
            string_ptr++;
            switch(*string_ptr)
            {
                /*
                 * Character.
                 */
                case 'c':
                {
                    char c = (char)va_arg(argument_list, int);
                    print_character(c);
                    break;
                }

                /*
                 * String.
                 */
                case 's':
                {
                    char *ptr = va_arg(argument_list, char *);
                    while(*ptr != 0)
                    {
                        print_character(*ptr);
                        ptr++;
                    }
                    break;
                }

                /*
                 * Signed integer.
                 */
                case 'd':
                {
                    int32_t number = va_arg(argument_list, int32_t);
                    if(number < 0)
                    {
                        print_character('-');
                        number = 0 - number;
                    }
                    print_number((uint32_t)number);
                    break;
                }

                /*
                 * Unsigned integer.
                 */
                case 'u':
                {
                    print_number(va_arg(argument_list, uint32_t));
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

    va_end(argument_list);
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief Setup the CPU cycle counter.
 */
void debug::stopwatch_initialise(void)
{
    // Enable the cycle counter. 
    SET_BIT(CoreDebug->DEMCR, CoreDebug_DEMCR_TRCENA_Msk);
    SET_BIT(DWT->CTRL, DWT_CTRL_CYCCNTENA_Msk);
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief Start counting CPU cycles.
 */
void debug::stopwatch_start(void)
{
    WRITE_REG(DWT->CYCCNT, 0);
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief           Stop counting CPU cycles and figure out how many nano-seconds they equate to.
                    Will be accurate assuming less than 89 seconds has passed and core is running
                    at 48MHz.
 * @return uint32_t Time elapsed in nano-seconds.
 */
uint32_t debug::stopwatch_stop(void)
{
    uint32_t stop_time = READ_REG(DWT->CYCCNT);

    return ((stop_time * 1000) / (CORE_SPEED / 1000000));
}

/*------------------------------------------------------------------------------------------------*/
/*-static-functions-------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/**
 * @brief           Print a character via UART.
 * @param character Character to print.
 */
void print_character(char character)
{
    while(usart::tx_free() == 0) {}
    usart::tx_byte(character);
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief        Print a number via UART.
 * @param number Number to print.
 */
void print_number(uint32_t number)
{
    char    buffer[10];
    uint8_t buffer_ptr = 0;

    do
    {
        buffer[buffer_ptr] = ('0' + (number % 10));
        number             = number / 10;
        buffer_ptr++;
    } while(number != 0);

    while(buffer_ptr > 0)
    {
        buffer_ptr--;
        print_character(buffer[buffer_ptr]);
    }
}

/*------------------------------------------------------------------------------------------------*/
/*-end-of-module----------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/
