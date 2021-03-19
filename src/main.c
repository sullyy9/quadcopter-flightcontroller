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

        int16_t gyro_pitch_dps;
        int16_t gyro_roll_dps;
        int16_t gyro_yaw_dps;

        int16_t roll_mrad;
        int16_t pitch_mrad;
        int16_t yaw_mrad;

        int32_t roll_mdeg;
        int32_t pitch_mdeg;
        int32_t yaw_mdeg;

        uint32_t time_current_us;
        uint32_t time_previous_us;
        uint32_t time_change_us;
}
inertial_data;

static struct kalman_filter
{
        int32_t roll_mdeg;
        int32_t pitch_mdeg;
        int32_t yaw_mdeg;

        int32_t roll_drift;
        int32_t pitch_drift;
        int32_t yaw_drift;

        int32_t roll_error_covariance[2][2];
        int32_t pitch_error_covariance[2][2];
        int32_t yaw_error_covariance[2][2];
}
kalman_filter;

static bool new_accel_data  = false;
static bool new_mag_data    = false;
static bool new_gyro_data   = false;

/*----------------------------------------------------------------------------*/
/*-forward-declarations-------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

void apply_kalman_filter( void );

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
            inertial_data.gyro_roll_dps =
            (
                ( (int32_t)inertial_data.gyro_y_raw * GYRO_RANGE ) / 32768
            );
            inertial_data.gyro_pitch_dps =
            (
                ( (int32_t)inertial_data.gyro_x_raw * GYRO_RANGE ) / 32768
            );
            inertial_data.gyro_yaw_dps =
            (
                ( (int32_t)inertial_data.gyro_z_raw * GYRO_RANGE ) / 32768
            );

            new_gyro_data = true;
        }

        /*
         * Process the raw data to get a roll, pitch and heading angle
         * Takes about 1.1ms - 1.25ms, TODO some optimisation
         */
        if( ( new_accel_data == true ) && ( new_mag_data == true ) && ( new_gyro_data == true ) )
        {
            new_accel_data  = false;
            new_mag_data    = false;
            new_gyro_data   = false;

            /*
             * Calculate the change in time
             * TODO can the data rates of each device be synchronised?
             */
            inertial_data.time_current_us =
            (
                ( system_runtime_ms * 1000 ) + commonio_get_systick_us( )
            );
            inertial_data.time_change_us =
            (
                inertial_data.time_current_us - inertial_data.time_previous_us
            );
            inertial_data.time_previous_us = inertial_data.time_current_us;

            /*
             * Convert acceleration and magnetometer data to roll, pitch and yaw angle.
             * If the modulus of the acceleration or magnetometer values is not near 1g or earth's
             * field strength respectively, we can't use the data.
             *
             * See document DT0058 for calculation details however:
             * The documentation takes x as the forward facing axis, so roll is around the x axis
             * and pitch is around the y axis. I have flipped that so that y is the forward facing
             * axis.
             */
            int16_t accel_mod;
            accel_mod = (int16_t)sqrt( pow( inertial_data.accel_x_mg, 2 ) +
                                       pow( inertial_data.accel_y_mg, 2 ) +
                                       pow( inertial_data.accel_z_mg, 2 ) );
            if( ( accel_mod > 800 ) && ( accel_mod < 1200 ) )
            {
                double roll_rad;
                double pitch_rad;

                /*
                 * Calculate accelerometer roll angle
                 */
                roll_rad = atan2( inertial_data.accel_x_raw, inertial_data.accel_z_raw );

                /*
                 * Calculate accelerometer pitch angle. Must first calculate Z in respect to earth
                 * rather than its own orientation
                 */
                double gz2 =
                (
                    ( inertial_data.accel_x_raw * sin( roll_rad ) ) +
                    ( inertial_data.accel_z_raw * cos( roll_rad ) )
                );
                pitch_rad = atan( inertial_data.accel_y_raw / gz2 );

                inertial_data.roll_mrad  = (int16_t)( roll_rad  * 1000 );
                inertial_data.pitch_mrad = (int16_t)( pitch_rad * 1000 );

                /*
                 * Calculate accelerometer and magnetometer yaw angle if the data is good.
                 * Earth's normal field strength is between 300mG ( southern hemisphere ) and
                 * 600mG ( northern hemisphere ).
                 */
                int16_t mag_mod;
                mag_mod = (int16_t)sqrt( pow( inertial_data.mag_x_mG, 2 ) +
                                         pow( inertial_data.mag_y_mG, 2 ) +
                                         pow( inertial_data.mag_z_mG, 2 ) );
                if( ( mag_mod > 200 ) && ( mag_mod < 700 ) )
                {
                    double yaw_rad;

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

                    inertial_data.yaw_mrad = (int16_t)( yaw_rad * 1000 );
                }

                inertial_data.roll_mdeg  = (int32_t)( inertial_data.roll_mrad   * ( 180 / M_PI ) );
                inertial_data.pitch_mdeg = (int32_t)( inertial_data.pitch_mrad  * ( 180 / M_PI ) );
                inertial_data.yaw_mdeg   = (int32_t)( inertial_data.yaw_mrad    * ( 180 / M_PI ) );

                apply_kalman_filter( );
                //debug_printf( "loop runtime: %u \r\n", debug_stopwatch_stop( ) );
            }

            static uint32_t print_every = 0;
            print_every++;
            if( ( OUTPUT_DATA == true ) && ( print_every == 10 ) )
            {
                print_every = 0;
                debug_printf( "\r\n" );
                debug_printf( "orientation data\r\n" );
                debug_printf( "DATA:TIME:%d\r\n",     system_runtime_ms );
                debug_printf( "DATA:ABANK:%d\r\n",    ( inertial_data.roll_mdeg   / 1000 ) );
                debug_printf( "DATA:KBANK:%d\r\n",    ( kalman_filter.roll_mdeg   / 1000 ) );
                debug_printf( "DATA:AATTITUDE:%d\r\n", ( inertial_data.pitch_mdeg / 1000 ) );
                debug_printf( "DATA:KATTITUDE:%d\r\n", ( kalman_filter.pitch_mdeg / 1000 ) );
                debug_printf( "DATA:MHEADING:%d\r\n",  ( inertial_data.yaw_mdeg   / 1000 ) );
                debug_printf( "DATA:KHEADING:%d\r\n",  ( kalman_filter.yaw_mdeg   / 1000 ) );
                debug_printf( "DATA:ACCELX:%d\r\n", inertial_data.accel_x_mg );
                debug_printf( "DATA:ACCELY:%d\r\n", inertial_data.accel_y_mg );
                debug_printf( "DATA:ACCELZ:%d\r\n", inertial_data.accel_z_mg );
                debug_printf( "DATA:MAGX:%d\r\n",   inertial_data.mag_x_mG );
                debug_printf( "DATA:MAGY:%d\r\n",   inertial_data.mag_y_mG );
                debug_printf( "DATA:MAGZ:%d\r\n",   inertial_data.mag_z_mG );
                debug_printf( "DATA:GYROROLL:%d\r\n",  inertial_data.gyro_roll_dps );
                debug_printf( "DATA:GYROPITCH:%d\r\n",  inertial_data.gyro_pitch_dps );
                debug_printf( "DATA:GYROYAW:%d\r\n",  inertial_data.gyro_yaw_dps );
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

/*
 * Kalman filter tutorial:
 * http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/
 */
void apply_kalman_filter( void )
{
    /*
     * Roll angle
     */
    int32_t q_bias_roll     = 0;
    int32_t q_angle_roll    = 0;
    int32_t r_measure_roll  = 0;

    /*
     * Estimate angle from gyroscope data
     */
    int32_t roll_rate_dps;
    roll_rate_dps = inertial_data.gyro_roll_dps - kalman_filter.roll_drift;

    kalman_filter.roll_mdeg += ( ( roll_rate_dps * inertial_data.time_change_us ) / 1000 );

    /*
     * Calculate the error covariance ( estimate trust in estimated angle )
     */
    kalman_filter.roll_error_covariance[0][0] +=
    (
        inertial_data.time_change_us *
        (
            ( inertial_data.time_change_us * kalman_filter.roll_error_covariance[1][1] ) -
            kalman_filter.roll_error_covariance[0][1] -
            kalman_filter.roll_error_covariance[1][0] +
            q_angle_roll
        )
    );
    kalman_filter.roll_error_covariance[0][1] -=
    (
        inertial_data.time_change_us * kalman_filter.roll_error_covariance[1][1]
    );
    kalman_filter.roll_error_covariance[1][0] -=
    (
        inertial_data.time_change_us * kalman_filter.roll_error_covariance[1][1]
    );
    kalman_filter.roll_error_covariance[1][1] += q_bias_roll * inertial_data.time_change_us;

    /*
     * Calculate innovation ( difference between estimated angle and accelerometer angle ).
     */
    int32_t roll_innovation;
    roll_innovation = inertial_data.roll_mdeg - kalman_filter.roll_mdeg;

    /*
     * Estimate trust in the accelerometer measurement
     */
    int32_t roll_innovation_covariance;
    roll_innovation_covariance = kalman_filter.roll_error_covariance[0][0] + r_measure_roll;

    /*
     * Calculate Kalman gain
     */
    double roll_kalman_gain[2];
    roll_kalman_gain[0] = (double)kalman_filter.roll_error_covariance[0][0] / roll_innovation_covariance;
    roll_kalman_gain[1] = (double)kalman_filter.roll_error_covariance[1][0] / roll_innovation_covariance;

    /*
     * Update the estimate angle
     */
    kalman_filter.roll_mdeg += roll_kalman_gain[0] * roll_innovation;
    kalman_filter.roll_drift += roll_kalman_gain[1] * roll_innovation;

    /*
     * Update the error covariance
     */
    int32_t hold_00 = kalman_filter.roll_error_covariance[0][0];
    int32_t hold_01 = kalman_filter.roll_error_covariance[0][1];

    kalman_filter.roll_error_covariance[0][0] -= roll_kalman_gain[0] * hold_00;
    kalman_filter.roll_error_covariance[0][1] -= roll_kalman_gain[0] * hold_01;
    kalman_filter.roll_error_covariance[1][0] -= roll_kalman_gain[1] * hold_00;
    kalman_filter.roll_error_covariance[1][1] -= roll_kalman_gain[1] * hold_01;


    /*
     * Pitch angle
     */
    int32_t q_bias_pitch     = 0;
    int32_t q_angle_pitch    = 0;
    int32_t r_measure_pitch  = 0;

    /*
     * Estimate angle from gyroscope data
     */
    int32_t pitch_rate_dps;
    pitch_rate_dps = inertial_data.gyro_pitch_dps - kalman_filter.pitch_drift;

    kalman_filter.pitch_mdeg += ( ( pitch_rate_dps * inertial_data.time_change_us ) / 1000 );

    /*
     * Calculate the error covariance ( estimate trust in estimated angle )
     */
    kalman_filter.pitch_error_covariance[0][0] +=
    (
        inertial_data.time_change_us *
        (
            ( inertial_data.time_change_us * kalman_filter.pitch_error_covariance[1][1] ) -
            kalman_filter.pitch_error_covariance[0][1] -
            kalman_filter.pitch_error_covariance[1][0] +
            q_angle_pitch
        )
    );
    kalman_filter.pitch_error_covariance[0][1] -=
    (
        inertial_data.time_change_us * kalman_filter.pitch_error_covariance[1][1]
    );
    kalman_filter.pitch_error_covariance[1][0] -=
    (
        inertial_data.time_change_us * kalman_filter.pitch_error_covariance[1][1]
    );
    kalman_filter.pitch_error_covariance[1][1] += q_bias_pitch * inertial_data.time_change_us;

    /*
     * Calculate innovation ( difference between estimated angle and accelerometer angle ).
     */
    int32_t pitch_innovation;
    pitch_innovation = inertial_data.pitch_mdeg - kalman_filter.pitch_mdeg;

    /*
     * Estimate trust in the accelerometer measurement
     */
    int32_t pitch_innovation_covariance;
    pitch_innovation_covariance = kalman_filter.pitch_error_covariance[0][0] + r_measure_pitch;

    /*
     * Calculate Kalman gain
     */
    double pitch_kalman_gain[2];
    pitch_kalman_gain[0] = (double)kalman_filter.pitch_error_covariance[0][0] / pitch_innovation_covariance;
    pitch_kalman_gain[1] = (double)kalman_filter.pitch_error_covariance[1][0] / pitch_innovation_covariance;

    /*
     * Update the estimate angle
     */
    kalman_filter.pitch_mdeg +=  pitch_kalman_gain[0] * pitch_innovation;
    kalman_filter.pitch_drift += pitch_kalman_gain[1] * pitch_innovation;

    /*
     * Update the error covariance
     */
    hold_00 = kalman_filter.pitch_error_covariance[0][0];
    hold_01 = kalman_filter.pitch_error_covariance[0][1];

    kalman_filter.pitch_error_covariance[0][0] -= pitch_kalman_gain[0] * hold_00;
    kalman_filter.pitch_error_covariance[0][1] -= pitch_kalman_gain[0] * hold_01;
    kalman_filter.pitch_error_covariance[1][0] -= pitch_kalman_gain[1] * hold_00;
    kalman_filter.pitch_error_covariance[1][1] -= pitch_kalman_gain[1] * hold_01;

    /*
     * yaw angle
     */
    int32_t q_bias_yaw     = 0;
    int32_t q_angle_yaw    = 0;
    int32_t r_measure_yaw  = 0;

    /*
     * Estimate angle from gyroscope data
     */
    int32_t yaw_rate_dps;
    yaw_rate_dps = inertial_data.gyro_yaw_dps - kalman_filter.yaw_drift;

    kalman_filter.yaw_mdeg += ( ( yaw_rate_dps * inertial_data.time_change_us ) / 1000 );

    /*
     * Calculate the error covariance ( estimate trust in estimated angle )
     */
    kalman_filter.yaw_error_covariance[0][0] +=
    (
        inertial_data.time_change_us *
        (
            ( inertial_data.time_change_us * kalman_filter.yaw_error_covariance[1][1] ) -
            kalman_filter.yaw_error_covariance[0][1] -
            kalman_filter.yaw_error_covariance[1][0] +
            q_angle_yaw
        )
    );
    kalman_filter.yaw_error_covariance[0][1] -=
    (
        inertial_data.time_change_us * kalman_filter.yaw_error_covariance[1][1]
    );
    kalman_filter.yaw_error_covariance[1][0] -=
    (
        inertial_data.time_change_us * kalman_filter.yaw_error_covariance[1][1]
    );
    kalman_filter.yaw_error_covariance[1][1] += q_bias_yaw * inertial_data.time_change_us;

    /*
     * Calculate innovation ( difference between estimated angle and accelerometer angle ).
     */
    int32_t yaw_innovation;
    yaw_innovation = inertial_data.yaw_mdeg - kalman_filter.yaw_mdeg;

    /*
     * Estimate trust in the accelerometer measurement
     */
    int32_t yaw_innovation_covariance;
    yaw_innovation_covariance = kalman_filter.yaw_error_covariance[0][0] + r_measure_yaw;

    /*
     * Calculate Kalman gain
     */
    double yaw_kalman_gain[2];
    yaw_kalman_gain[0] = (double)kalman_filter.yaw_error_covariance[0][0] / yaw_innovation_covariance;
    yaw_kalman_gain[1] = (double)kalman_filter.yaw_error_covariance[1][0] / yaw_innovation_covariance;

    /*
     * Update the estimate angle
     */
    kalman_filter.yaw_mdeg +=  yaw_kalman_gain[0] * yaw_innovation;
    kalman_filter.yaw_drift += yaw_kalman_gain[1] * yaw_innovation;

    /*
     * Update the error covariance
     */
    hold_00 = kalman_filter.yaw_error_covariance[0][0];
    hold_01 = kalman_filter.yaw_error_covariance[0][1];

    kalman_filter.yaw_error_covariance[0][0] -= yaw_kalman_gain[0] * hold_00;
    kalman_filter.yaw_error_covariance[0][1] -= yaw_kalman_gain[0] * hold_01;
    kalman_filter.yaw_error_covariance[1][0] -= yaw_kalman_gain[1] * hold_00;
    kalman_filter.yaw_error_covariance[1][1] -= yaw_kalman_gain[1] * hold_01;
}

/*----------------------------------------------------------------------------*/
/*-end-of-module--------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
