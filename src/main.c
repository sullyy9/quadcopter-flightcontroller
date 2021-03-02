/*----------------------------------------------------------------------------*/
/*
    Ryan Sullivan

    Module Name     :   main.c
    Description     :   main program
*/
/*----------------------------------------------------------------------------*/

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "accelerometer.h"

#include "main.h"
#include "io.h"
#include "utils.h"
#include "debug.h"
#include "commonio.h"
#include "usart.h"
#include "i2c.h"

/*----------------------------------------------------------------------------*/
/*-constant-definitions-------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-exported-variables---------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-static-variables-----------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

volatile uint16_t frick_timer = 0;

/*----------------------------------------------------------------------------*/
/*-forward-declarations-------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-exported-functions---------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

int main( void )
{

    io_initialise( );

    commonio_clear_reset_flags( );

    usart_initialise( );
    utils_wait_ms( 100 );
    i2c_initialise( );

    utils_wait_ms( 1000 );

    debug_printf( "\r\n" );
    debug_printf( "\r\n" );
    debug_printf( "quad-copter - start-------------------\r\n" );

    io_initialise_accelerometer( );

    utils_wait_ms( 100 );
    debug_printf( "initialisation complete\r\n" );

    /*
     * begin main loop
     */
    while( 1 )
    {
        /*
         * read acceleration data when its ready
         */
        int16_t accel_x_raw = 0;
        int16_t accel_y_raw = 0;
        int16_t accel_z_raw = 0;
        while( io_accelerometer_data_ready( ) == false );
        io_read_accelerometer( &accel_x_raw, &accel_y_raw, &accel_z_raw );

        /*
         * convert raw data to milli g's
         */
        int16_t accel_x_mg = ( ( (int32_t)accel_x_raw * 1000 ) / (int32_t)( pow( 2, ACCEL_RESOLUTION ) / 8 ) );
        int16_t accel_y_mg = ( ( (int32_t)accel_y_raw * 1000 ) / (int32_t)( pow( 2, ACCEL_RESOLUTION ) / 8 ) );
        int16_t accel_z_mg = ( ( (int32_t)accel_z_raw * 1000 ) / (int32_t)( pow( 2, ACCEL_RESOLUTION ) / 8 ) );

        /*
         * Convert acceleration data to pitch and roll angle.
         * if the modulus of the acceleration values is not near
         * g, we're undergoing some radical high g manoeuvres and
         * the data can't produce accurate pitch and roll values
         */
        int16_t pitch_mrad  = 0;
        int16_t roll_mrad   = 0;
        int32_t pitch_mdeg  = 0;
        int32_t roll_mdeg   = 0;
        int16_t accel_mod   = 0;

        accel_mod = (int16_t)sqrt( pow( accel_x_mg, 2 ) + pow( accel_y_mg, 2 ) + pow( accel_z_mg, 2 ) );
        if( ( accel_mod < 1200 ) && ( accel_mod > 800 ) )
        {
            roll_mrad = (int16_t)( atan2( accel_x_mg, accel_z_mg ) * 1000);

            double z2 = 0;
            z2 = ( ( accel_x_mg * sin( ( (double)roll_mrad / 1000 ) ) ) + ( accel_z_mg * cos( ( (double)roll_mrad / 1000 ) ) ) );
            pitch_mrad = (int16_t)( atan( ( accel_y_mg ) / z2 ) * 1000 );

            roll_mdeg   = (int32_t)( roll_mrad  * ( 180 / M_PI ) ) ;
            pitch_mdeg  = (int32_t)( pitch_mrad * ( 180 / M_PI ) );

            debug_printf( "\r\n" );
            debug_printf( "orientation data ( mRad ): \r\n" );
            debug_printf( "r: %d\t\t\t", ( roll_mdeg / 1000 ) );
            debug_printf( "p: %d\t\t\t", ( pitch_mdeg / 1000 ) );
            debug_printf( "\r\n" );
        }
    }

    return( 0 );
}

/*----------------------------------------------------------------------------*/

void main_1ms_timer_isr( void )
{
    io_1ms_poll( );
    utils_1ms_poll( );
}

/*----------------------------------------------------------------------------*/
/*-static-functions-----------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-end-of-module--------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
