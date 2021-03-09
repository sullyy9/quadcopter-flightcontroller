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

#include "lsm303agr.h"
#include "i3g4250d.h"

#include "main.h"
#include "io.h"
#include "utils.h"
#include "debug.h"
#include "commonio.h"
#include "usart.h"
#include "i2c.h"
#include "spi.h"

/*----------------------------------------------------------------------------*/
/*-constant-definitions-------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

#define OUTPUT_DATA         true

/*----------------------------------------------------------------------------*/
/*-exported-variables---------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-static-variables-----------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

static volatile uint32_t system_runtime_ms = 0;

static struct inertial_data
{
        int16_t accel_x_raw;
        int16_t accel_y_raw;
        int16_t accel_z_raw;

        int16_t accel_x_mg;
        int16_t accel_y_mg;
        int16_t accel_z_mg;

        int16_t mag_x_raw;
        int16_t mag_y_raw;
        int16_t mag_z_raw;

        int16_t mag_x_mG;
        int16_t mag_y_mG;
        int16_t mag_z_mG;

        int16_t gyro_x_raw;
        int16_t gyro_y_raw;
        int16_t gyro_z_raw;

        int16_t gyro_x_dps;
        int16_t gyro_y_dps;
        int16_t gyro_z_dps;

        int16_t roll_mrad;
        int16_t pitch_mrad;
        int16_t yaw_mrad;

        int32_t roll_mdeg;
        int32_t pitch_mdeg;
        int32_t yaw_mdeg;
}
inertial_data;

static bool new_accel_data  = false;
static bool new_mag_data    = false;
static bool new_gyro_data   = false;

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

    debug_stopwatch_initialise( );

    usart_initialise( );
    i2c_initialise( );
    spi_initialise( );

    utils_wait_ms( 1000 );

    debug_printf( "\r\n" );
    debug_printf( "\r\n" );
    debug_printf( "Quad-copter flight controller - start-------------------\r\n" );

    io_accelerometer_initialise( );
    io_magnetometer_initialise( );
    io_gyroscope_initialise( );

    debug_printf( "initialisation complete\r\n" );
    utils_wait_ms( 1000 );

    commonio_initialise_wwdg( 40 );

    /*
     * main loop
     */
    while( 1 )
    {
        commonio_reset_wwdg( );
        debug_stopwatch_start( );

        /*
         * read acceleration data if its ready
         */
        if( io_accelerometer_data_ready( ) == true )
        {
            io_accelerometer_read
            (
                &inertial_data.accel_x_raw,
                &inertial_data.accel_y_raw,
                &inertial_data.accel_z_raw
            );

            /*
             * convert raw accelerometer data to milli g's
             */
            inertial_data.accel_x_mg =
            (
                ( (int32_t)inertial_data.accel_x_raw * 1000 ) /
                (int32_t)( pow( 2, ACCEL_RESOLUTION ) / 8 )
            );
            inertial_data.accel_y_mg =
            (
                ( (int32_t)inertial_data.accel_y_raw * 1000 ) /
                (int32_t)( pow( 2, ACCEL_RESOLUTION ) / 8 )
            );
            inertial_data.accel_z_mg =
            (
                ( (int32_t)inertial_data.accel_z_raw * 1000 ) /
                (int32_t)( pow( 2, ACCEL_RESOLUTION ) / 8 )
            );

            new_accel_data = true;
        }

        /*
         * read magnetometer data if its ready
         */
        if( io_magnetometer_data_ready( ) == true )
        {
            io_magnetometer_read
            (
                &inertial_data.mag_x_raw,
                &inertial_data.mag_y_raw,
                &inertial_data.mag_z_raw
            );

            inertial_data.mag_x_mG = inertial_data.mag_x_raw * 1.5;
            inertial_data.mag_y_mG = inertial_data.mag_y_raw * 1.5;
            inertial_data.mag_z_mG = inertial_data.mag_z_raw * 1.5;

            new_mag_data = true;
        }

        /*
         * read gyroscope data if its ready
         */
        if( io_gyroscope_data_ready( ) == true )
        {
            io_gyroscope_read
            (
                &inertial_data.gyro_x_raw,
                &inertial_data.gyro_y_raw,
                &inertial_data.gyro_z_raw
            );

            /*
             * convert raw gyroscope data to dps
             */
            inertial_data.gyro_x_dps =
            (
                ( (int32_t)inertial_data.gyro_x_raw * GYRO_RANGE ) / 32768
            );
            inertial_data.gyro_y_dps =
            (
                ( (int32_t)inertial_data.gyro_y_raw * GYRO_RANGE ) / 32768
            );
            inertial_data.gyro_z_dps =
            (
                ( (int32_t)inertial_data.gyro_z_raw * GYRO_RANGE ) / 32768
            );

            new_gyro_data = true;
        }

        /*
         * Convert raw data into Roll, Pitch and Yaw
         * takes about 650uS, TODO some optimisation
         */
        if( ( new_accel_data == true ) && ( new_mag_data == true ) )
        {
            new_mag_data    = false;
            new_accel_data  = false;

            int16_t accel_mod;
            accel_mod = (int16_t)sqrt( pow( inertial_data.accel_x_mg, 2 ) +
                                       pow( inertial_data.accel_y_mg, 2 ) +
                                       pow( inertial_data.accel_z_mg, 2 ) );

            int16_t mag_mod;
            mag_mod = (int16_t)sqrt( pow( inertial_data.mag_x_mG, 2 ) +
                                     pow( inertial_data.mag_y_mG, 2 ) +
                                     pow( inertial_data.mag_z_mG, 2 ) );

            /*
             * Convert acceleration data to pitch and roll angle.
             * Convert magnetometer data to yaw angle.
             * if the modulus of the acceleration values is not near
             * g, we're undergoing some radical high g manoeuvres and
             * the data can't produce accurate pitch and roll values
             * See document DT0058 for details
             * The documentation takes x as the forward facing axis, so roll
             * is around the x axis and pitch is around the y axis
             * I have flipped that so that y is the forward facing axis
             */
            if( ( accel_mod < 1200 ) && ( accel_mod > 800 ) )
            {
                double roll_rad;
                double pitch_rad;

                /*
                 * Roll
                 */
                roll_rad = atan2( inertial_data.accel_x_raw, inertial_data.accel_z_raw );

                /*
                 * Pitch
                 */
                double gz2 =
                (
                    ( inertial_data.accel_x_raw * sin( roll_rad ) ) +
                    ( inertial_data.accel_z_raw * cos( roll_rad ) )
                );
                pitch_rad = atan( inertial_data.accel_y_raw / gz2 );

                inertial_data.roll_mrad     = (int16_t)(roll_rad    * 1000);
                inertial_data.pitch_mrad    = (int16_t)(pitch_rad   * 1000);

                if( ( mag_mod < 700 ) && ( mag_mod > 200 ) )
                {
                    double yaw_rad;

                    /*
                     * Yaw
                     */
                    double by2 =
                        (
                            ( inertial_data.mag_z_raw * sin( pitch_rad ) ) -
                            ( inertial_data.mag_y_raw * cos( pitch_rad ) )
                        );
                    double bz2 =
                        (
                            ( inertial_data.mag_y_raw * sin( pitch_rad ) ) +
                            ( inertial_data.mag_z_raw * cos( pitch_rad ) )
                        );
                    double bx3 =
                        (
                            ( inertial_data.mag_x_raw * cos( ( roll_rad * -1 ) ) ) +
                            ( bz2 * sin( ( roll_rad * -1 ) ) )
                        );
                    yaw_rad = atan2( by2, bx3 );

                    inertial_data.yaw_mrad      = (int16_t)(yaw_rad     * 1000);
                }

                inertial_data.roll_mdeg  = (int32_t)( inertial_data.roll_mrad   * ( 180 / M_PI ) );
                inertial_data.pitch_mdeg = (int32_t)( inertial_data.pitch_mrad  * ( 180 / M_PI ) );
                inertial_data.yaw_mdeg   = (int32_t)( inertial_data.yaw_mrad    * ( 180 / M_PI ) );
            }

            static uint32_t print_every = 0;
            print_every++;
            if( ( OUTPUT_DATA == true ) && ( print_every == 10 ) )
            {
                print_every = 0;
                debug_printf( "\r\n" );
                debug_printf( "orientation data\r\n" );
                debug_printf( "DATA:TIME:%d\r\n",     system_runtime_ms );
                debug_printf( "DATA:BANK:%d\r\n",     ( inertial_data.roll_mdeg  / 1000 ) );
                debug_printf( "DATA:ATTITUDE:%d\r\n", ( inertial_data.pitch_mdeg / 1000 ) );
                debug_printf( "DATA:HEADING:%d\r\n",  ( inertial_data.yaw_mdeg   / 1000 ) );
                debug_printf( "DATA:ACCELX:%d\r\n", inertial_data.accel_x_mg );
                debug_printf( "DATA:ACCELY:%d\r\n", inertial_data.accel_y_mg );
                debug_printf( "DATA:ACCELZ:%d\r\n", inertial_data.accel_z_mg );
                debug_printf( "DATA:MAGX:%d\r\n",   inertial_data.mag_x_mG );
                debug_printf( "DATA:MAGY:%d\r\n",   inertial_data.mag_y_mG );
                debug_printf( "DATA:MAGZ:%d\r\n",   inertial_data.mag_z_mG );
                debug_printf( "DATA:GYROX:%d\r\n",  inertial_data.gyro_x_dps );
                debug_printf( "DATA:GYROY:%d\r\n",  inertial_data.gyro_y_dps );
                debug_printf( "DATA:GYROZ:%d\r\n",  inertial_data.gyro_z_dps );
                debug_printf( "\r\n" );
            }

            //debug_printf( "main loop runtime: %d\r\n", debug_stopwatch_stop( ) );
        }
    }

    return( 0 );
}

/*----------------------------------------------------------------------------*/

void main_1ms_timer_isr( void )
{
    io_1ms_poll( );
    utils_1ms_poll( );
    system_runtime_ms++;
}

/*----------------------------------------------------------------------------*/
/*-static-functions-----------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-end-of-module--------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
