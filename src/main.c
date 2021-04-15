/**
 * -------------------------------------------------------------------------------------------------
 * @author  Ryan Sullivan (ryansullivan@googlemail.com)
 *
 * @file    main.c
 * @brief   Main function.
 *
 * @date    2021-04-09
 * -------------------------------------------------------------------------------------------------
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "lsm303agr.h"
#include "i3g4250d.h"

#include "main.h"
#include "io.h"
#include "utils.h"
#include "debug.h"
#include "system.h"

/*------------------------------------------------------------------------------------------------*/
/*-constant-definitions---------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

#define PI 3.141592f

#define OUTPUT_DATA true

/*------------------------------------------------------------------------------------------------*/
/*-exported-variables-----------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------------------*/
/*-static-variables-------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

static volatile bool     run_program       = true;
static volatile uint32_t system_runtime_ms = 0;

typedef struct
{
    int32_t x_raw; // x in the range of the register size, reflecting the acceleration in gs
    int32_t y_raw; // y in the range of the register size, reflecting the acceleration in gs
    int32_t z_raw; // z in the range of the register size, reflecting the acceleration in gs

    float x_g; // x in terms of earth gravity
    float y_g; // y in terms of earth gravity
    float z_g; // z in terms of earth gravity

    float attitude_rad; // attitude in radians
    float bank_rad;     // bank in radians

    float attitude_deg; // attitude in degrees
    float bank_deg;     // bank in degrees

} accel_data_t;

typedef struct
{
    int32_t x_raw; // x in milli-Gauss / 1.5
    int32_t y_raw; // y in milli-Gauss / 1.5
    int32_t z_raw; // z in milli-Gauss / 1.5

    float x_gauss; // x in gauss
    float y_gauss; // y in gauss
    float z_gauss; // z in gauss

    float heading_rad; // heading in radians
    float heading_deg; // heading in degrees

} mag_data_t;

typedef struct
{
    int32_t x_raw; // x in the range of the register size, reflecting rotation in degrees per second
    int32_t y_raw; // y in the range of the register size, reflecting rotation in degrees per second
    int32_t z_raw; // z in the range of the register size, reflecting rotation in degrees per second

    float pitch_dps; // pitch in degrees per second
    float roll_dps;  // roll in degrees per second
    float yaw_dps;   // yaw in degrees per second

} gyro_data_t;

typedef struct
{
    int32_t current_us;
    int32_t previous_us;
    int32_t change_us;

} time_data_t;

typedef struct
{
    float bank_deg;
    float attitude_deg;
    float heading_deg;

    float bank_drift;
    float attitude_drift;
    float heading_drift;

    float bank_error_covariance[2][2];
    float attitude_error_covariance[2][2];
    float heading_error_covariance[2][2];

} kalman_data_t;

static bool new_accel_data = false;
static bool new_mag_data   = false;
static bool new_gyro_data  = false;

/*------------------------------------------------------------------------------------------------*/
/*-forward-declarations---------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

void apply_kalman_filter(void);

/*------------------------------------------------------------------------------------------------*/
/*-exported-functions-----------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/**
 * @brief      Main function.
 * @return int Unused.
 */
int main(void)
{
    io_initialise();
    system_clear_reset_flags();

    debug_stopwatch_initialise();

    utils_wait_ms(1000);

    io_accelerometer_initialise();
    io_magnetometer_initialise();
    io_gyroscope_initialise();

    debug_printf("\r\n");
    debug_printf("\r\n");
    debug_printf("Quad-copter flight controller - start-------------------\r\n");
    debug_printf("initialisation complete\r\n");
    utils_wait_ms(1000);

    system_initialise_wwdg(40);

    accel_data_t accel_data;
    mag_data_t   mag_data;
    gyro_data_t  gyro_data;
    time_data_t  time_data;

    kalman_data_t kalman_data;

    /*
     * main loop
     */
    while(run_program)
    {
        system_reset_wwdg();
        debug_stopwatch_start();

        /*
         * read acceleration data if its ready
         */
        if(io_accelerometer_data_ready() == true)
        {
            io_accelerometer_read(&accel_data.x_raw, &accel_data.y_raw, &accel_data.z_raw);

            /*
             * convert raw accelerometer data to g's
             */
            accel_data.x_g = ((float)accel_data.x_raw / (powf(2, ACCEL_RESOLUTION) / 8));
            accel_data.y_g = ((float)accel_data.y_raw / (powf(2, ACCEL_RESOLUTION) / 8));
            accel_data.z_g = ((float)accel_data.z_raw / (powf(2, ACCEL_RESOLUTION) / 8));

            new_accel_data = true;
        }

        /*
         * read magnetometer data if its ready
         */
        if(io_magnetometer_data_ready() == true)
        {
            io_magnetometer_read(&mag_data.x_raw, &mag_data.y_raw, &mag_data.z_raw);

            mag_data.x_gauss = (((float)mag_data.x_raw * 1.5f) / 1000);
            mag_data.y_gauss = (((float)mag_data.y_raw * 1.5f) / 1000);
            mag_data.z_gauss = (((float)mag_data.z_raw * 1.5f) / 1000);

            new_mag_data = true;
        }

        /*
         * read gyroscope data if its ready
         */
        if(io_gyroscope_data_ready() == true)
        {
            io_gyroscope_read(&gyro_data.x_raw, &gyro_data.y_raw, &gyro_data.z_raw);

            /*
             * convert raw gyroscope data to dps
             */
            gyro_data.roll_dps  = ((float)(gyro_data.y_raw * GYRO_RANGE) / 32768);
            gyro_data.pitch_dps = ((float)(gyro_data.x_raw * GYRO_RANGE) / 32768);
            gyro_data.yaw_dps   = ((float)(gyro_data.z_raw * GYRO_RANGE) / 32768);

            new_gyro_data = true;
        }

        /*
         * Process the raw data to get a roll, pitch and heading angle
         * Takes about 1.1ms - 1.25ms, TODO some optimisation
         */
        if((new_accel_data == true) && (new_mag_data == true) && (new_gyro_data == true))
        {
            new_accel_data = false;
            new_mag_data   = false;
            new_gyro_data  = false;

            /*
             * Calculate the change in time
             * TODO can the data rates of each device be synchronised?
             */
            time_data.current_us =
                (int32_t)((system_runtime_ms * 1000) + system_get_system_timer_us());

            time_data.change_us = (time_data.current_us - time_data.previous_us);

            time_data.previous_us = time_data.current_us;

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
            float accel_modulus;
            accel_modulus = powf(accel_data.x_g, 2);
            accel_modulus += powf(accel_data.y_g, 2);
            accel_modulus += powf(accel_data.z_g, 2);
            accel_modulus = sqrtf(accel_modulus);

            if((accel_modulus > 0.8f) && (accel_modulus < 1.2f))
            {
                /*
                 * Calculate accelerometer roll angle
                 */
                accel_data.bank_rad = atan2f(accel_data.x_g, accel_data.z_g);

                /*
                 * Calculate accelerometer pitch angle. Must first calculate Z in respect to earth
                 * rather than its own orientation
                 */
                float gz2 = ((accel_data.x_g * sinf(accel_data.bank_rad)) +
                             (accel_data.z_g * cosf(accel_data.bank_rad)));

                accel_data.attitude_rad = atanf(accel_data.y_g / gz2);

                /*
                 * Calculate accelerometer and magnetometer yaw angle if the data is good.
                 * Earth's normal field strength is between 300mG ( southern hemisphere ) and
                 * 600mG ( northern hemisphere ).
                 */
                float mag_mod;
                mag_mod = powf(mag_data.x_gauss, 2);
                mag_mod += powf(mag_data.y_gauss, 2);
                mag_mod += powf(mag_data.z_gauss, 2);
                mag_mod = sqrtf(mag_mod);

                if((mag_mod > 0.2f) && (mag_mod < 0.7f))
                {
                    float by2 = ((mag_data.z_gauss * sinf(accel_data.attitude_rad)) -
                                 (mag_data.y_gauss * cosf(accel_data.attitude_rad)));

                    float bz2 = ((mag_data.y_gauss * sinf(accel_data.attitude_rad)) +
                                 (mag_data.z_gauss * cosf(accel_data.attitude_rad)));

                    float bx3 = ((mag_data.x_gauss * cosf(accel_data.bank_rad * -1)) +
                                 (bz2 * sinf(accel_data.bank_rad * -1)));

                    mag_data.heading_rad = atan2f(by2, bx3);
                }

                accel_data.bank_deg     = (accel_data.bank_rad * (180 / PI));
                accel_data.attitude_deg = (accel_data.attitude_rad * (180 / PI));
                mag_data.heading_deg    = (mag_data.heading_rad * (180 / PI));

                // apply_kalman_filter();
                // debug_printf("loop runtime: %u \r\n", debug_stopwatch_stop());
            }

            static uint32_t print_every = 0;
            print_every++;
            if((OUTPUT_DATA == true) && (print_every == 10))
            {
                print_every = 0;
                debug_printf("\r\n");
                debug_printf("orientation data\r\n");
                debug_printf("DATA:TIME:%d\r\n", system_runtime_ms);
                debug_printf("DATA:ABANK:%d\r\n", (int32_t)accel_data.bank_deg);
                debug_printf("DATA:KBANK:%d\r\n", (int32_t)kalman_data.bank_deg);
                debug_printf("DATA:AATTITUDE:%d\r\n", (int32_t)accel_data.attitude_deg);
                debug_printf("DATA:KATTITUDE:%d\r\n", (int32_t)kalman_data.attitude_deg);
                debug_printf("DATA:MHEADING:%d\r\n", (int32_t)(mag_data.heading_deg));
                debug_printf("DATA:KHEADING:%d\r\n", (int32_t)kalman_data.heading_deg);
                debug_printf("DATA:ACCELX:%d\r\n", (int32_t)(accel_data.x_g * 1000));
                debug_printf("DATA:ACCELY:%d\r\n", (int32_t)(accel_data.y_g * 1000));
                debug_printf("DATA:ACCELZ:%d\r\n", (int32_t)(accel_data.z_g * 1000));
                debug_printf("DATA:MAGX:%d\r\n", (int32_t)(mag_data.x_gauss * 1000));
                debug_printf("DATA:MAGY:%d\r\n", (int32_t)(mag_data.y_gauss * 1000));
                debug_printf("DATA:MAGZ:%d\r\n", (int32_t)(mag_data.z_gauss * 1000));
                debug_printf("DATA:GYROROLL:%d\r\n", (int32_t)gyro_data.roll_dps);
                debug_printf("DATA:GYROPITCH:%d\r\n", (int32_t)gyro_data.pitch_dps);
                debug_printf("DATA:GYROYAW:%d\r\n", (int32_t)gyro_data.yaw_dps);
                debug_printf("\r\n");
            }
        }
    }

    return (0);
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief Main poll. Called by the systick interupt.
 */
void main_1ms_timer_isr(void)
{
    io_1ms_poll();    // NOLINT - ignore clangd warning
    utils_1ms_poll(); // NOLINT - ignore clangd warning
    system_runtime_ms++;
}

/*------------------------------------------------------------------------------------------------*/
/*-static-functions-------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/**
 * @brief Apply a kalman filter to the acceleroeter, magnetometer and gyroscope data sets.
 *
 * http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/
 */
// void apply_kalman_filter(void)
// {
//     /*
//      * Roll angle
//      */
//     int32_t q_bias_roll    = 0;
//     int32_t q_angle_roll   = 0;
//     int32_t r_measure_roll = 0;

//     /*
//      * Estimate angle from gyroscope data
//      */
//     int32_t roll_rate_dps;
//     roll_rate_dps = inertial_data.gyro_roll_dps - kalman_filter.roll_drift;

//     kalman_filter.roll_mdeg += ((roll_rate_dps * inertial_data.time_change_us) / 1000);

//     /*
//      * Calculate the error covariance ( estimate trust in estimated angle )
//      */
//     kalman_filter.roll_error_covariance[0][0] +=
//         (inertial_data.time_change_us *
//          ((inertial_data.time_change_us * kalman_filter.roll_error_covariance[1][1]) -
//           kalman_filter.roll_error_covariance[0][1] - kalman_filter.roll_error_covariance[1][0] +
//           q_angle_roll));

//     kalman_filter.roll_error_covariance[0][1] -=
//         (inertial_data.time_change_us * kalman_filter.roll_error_covariance[1][1]);

//     kalman_filter.roll_error_covariance[1][0] -=
//         (inertial_data.time_change_us * kalman_filter.roll_error_covariance[1][1]);

//     kalman_filter.roll_error_covariance[1][1] += (q_bias_roll * inertial_data.time_change_us);

//     /*
//      * Calculate innovation ( difference between estimated angle and accelerometer angle ).
//      */
//     int32_t roll_innovation;
//     roll_innovation = inertial_data.roll_mdeg - kalman_filter.roll_mdeg;

//     /*
//      * Estimate trust in the accelerometer measurement
//      */
//     int32_t roll_innovation_covariance;
//     roll_innovation_covariance = kalman_filter.roll_error_covariance[0][0] + r_measure_roll;

//     /*
//      * Calculate Kalman gain
//      */
//     double roll_kalman_gain[2];
//     roll_kalman_gain[0] =
//         (double)kalman_filter.roll_error_covariance[0][0] / roll_innovation_covariance;
//     roll_kalman_gain[1] =
//         (double)kalman_filter.roll_error_covariance[1][0] / roll_innovation_covariance;

//     /*
//      * Update the estimate angle
//      */
//     kalman_filter.roll_mdeg += (int32_t)(roll_kalman_gain[0] * roll_innovation);
//     kalman_filter.roll_drift += (int32_t)(roll_kalman_gain[1] * roll_innovation);

//     /*
//      * Update the error covariance
//      */
//     int32_t hold_00 = kalman_filter.roll_error_covariance[0][0];
//     int32_t hold_01 = kalman_filter.roll_error_covariance[0][1];

//     kalman_filter.roll_error_covariance[0][0] -= (int32_t)(roll_kalman_gain[0] * hold_00);
//     kalman_filter.roll_error_covariance[0][1] -= (int32_t)(roll_kalman_gain[0] * hold_01);
//     kalman_filter.roll_error_covariance[1][0] -= (int32_t)(roll_kalman_gain[1] * hold_00);
//     kalman_filter.roll_error_covariance[1][1] -= (int32_t)(roll_kalman_gain[1] * hold_01);

//     /*
//      * Pitch angle
//      */
//     int32_t q_bias_pitch    = 0;
//     int32_t q_angle_pitch   = 0;
//     int32_t r_measure_pitch = 0;

//     /*
//      * Estimate angle from gyroscope data
//      */
//     int32_t pitch_rate_dps;
//     pitch_rate_dps = inertial_data.gyro_pitch_dps - kalman_filter.pitch_drift;

//     kalman_filter.pitch_mdeg += ((pitch_rate_dps * inertial_data.time_change_us) / 1000);

//     /*
//      * Calculate the error covariance ( estimate trust in estimated angle )
//      */
//     kalman_filter.pitch_error_covariance[0][0] +=
//         (inertial_data.time_change_us *
//          ((inertial_data.time_change_us * kalman_filter.pitch_error_covariance[1][1]) -
//           kalman_filter.pitch_error_covariance[0][1] - kalman_filter.pitch_error_covariance[1][0]
//           + q_angle_pitch));

//     kalman_filter.pitch_error_covariance[0][1] -=
//         (inertial_data.time_change_us * kalman_filter.pitch_error_covariance[1][1]);

//     kalman_filter.pitch_error_covariance[1][0] -=
//         (inertial_data.time_change_us * kalman_filter.pitch_error_covariance[1][1]);

//     kalman_filter.pitch_error_covariance[1][1] += (q_bias_pitch * inertial_data.time_change_us);

//     /*
//      * Calculate innovation ( difference between estimated angle and accelerometer angle ).
//      */
//     int32_t pitch_innovation;
//     pitch_innovation = (inertial_data.pitch_mdeg - kalman_filter.pitch_mdeg);

//     /*
//      * Estimate trust in the accelerometer measurement
//      */
//     int32_t pitch_innovation_covariance;
//     pitch_innovation_covariance = kalman_filter.pitch_error_covariance[0][0] + r_measure_pitch;

//     /*
//      * Calculate Kalman gain
//      */
//     double pitch_kalman_gain[2];
//     pitch_kalman_gain[0] =
//         (double)kalman_filter.pitch_error_covariance[0][0] / pitch_innovation_covariance;

//     pitch_kalman_gain[1] =
//         (double)kalman_filter.pitch_error_covariance[1][0] / pitch_innovation_covariance;

//     /*
//      * Update the estimate angle
//      */
//     kalman_filter.pitch_mdeg += (int32_t)(pitch_kalman_gain[0] * pitch_innovation);
//     kalman_filter.pitch_drift += (int32_t)(pitch_kalman_gain[1] * pitch_innovation);

//     /*
//      * Update the error covariance
//      */
//     hold_00 = kalman_filter.pitch_error_covariance[0][0];
//     hold_01 = kalman_filter.pitch_error_covariance[0][1];

//     kalman_filter.pitch_error_covariance[0][0] -= (int32_t)(pitch_kalman_gain[0] * hold_00);
//     kalman_filter.pitch_error_covariance[0][1] -= (int32_t)(pitch_kalman_gain[0] * hold_01);
//     kalman_filter.pitch_error_covariance[1][0] -= (int32_t)(pitch_kalman_gain[1] * hold_00);
//     kalman_filter.pitch_error_covariance[1][1] -= (int32_t)(pitch_kalman_gain[1] * hold_01);

//     /*
//      * yaw angle
//      */
//     int32_t q_bias_yaw    = 0;
//     int32_t q_angle_yaw   = 0;
//     int32_t r_measure_yaw = 0;

//     /*
//      * Estimate angle from gyroscope data
//      */
//     int32_t yaw_rate_dps;
//     yaw_rate_dps = (inertial_data.gyro_yaw_dps - kalman_filter.yaw_drift);

//     kalman_filter.yaw_mdeg += ((yaw_rate_dps * inertial_data.time_change_us) / 1000);

//     /*
//      * Calculate the error covariance ( estimate trust in estimated angle )
//      */
//     kalman_filter.yaw_error_covariance[0][0] +=
//         (inertial_data.time_change_us *
//          ((inertial_data.time_change_us * kalman_filter.yaw_error_covariance[1][1]) -
//           kalman_filter.yaw_error_covariance[0][1] - kalman_filter.yaw_error_covariance[1][0] +
//           q_angle_yaw));

//     kalman_filter.yaw_error_covariance[0][1] -=
//         (inertial_data.time_change_us * kalman_filter.yaw_error_covariance[1][1]);

//     kalman_filter.yaw_error_covariance[1][0] -=
//         (inertial_data.time_change_us * kalman_filter.yaw_error_covariance[1][1]);

//     kalman_filter.yaw_error_covariance[1][1] += (q_bias_yaw * inertial_data.time_change_us);

//     /*
//      * Calculate innovation ( difference between estimated angle and accelerometer angle ).
//      */
//     int32_t yaw_innovation;
//     yaw_innovation = (inertial_data.yaw_mdeg - kalman_filter.yaw_mdeg);

//     /*
//      * Estimate trust in the accelerometer measurement
//      */
//     int32_t yaw_innovation_covariance;
//     yaw_innovation_covariance = (kalman_filter.yaw_error_covariance[0][0] + r_measure_yaw);

//     /*
//      * Calculate Kalman gain
//      */
//     double yaw_kalman_gain[2];
//     yaw_kalman_gain[0] =
//         (double)kalman_filter.yaw_error_covariance[0][0] / yaw_innovation_covariance;

//     yaw_kalman_gain[1] =
//         (double)kalman_filter.yaw_error_covariance[1][0] / yaw_innovation_covariance;

//     /*
//      * Update the estimate angle
//      */
//     kalman_filter.yaw_mdeg += (int32_t)(yaw_kalman_gain[0] * yaw_innovation);
//     kalman_filter.yaw_drift += (int32_t)(yaw_kalman_gain[1] * yaw_innovation);

//     /*
//      * Update the error covariance
//      */
//     hold_00 = kalman_filter.yaw_error_covariance[0][0];
//     hold_01 = kalman_filter.yaw_error_covariance[0][1];

//     kalman_filter.yaw_error_covariance[0][0] -= (int32_t)(yaw_kalman_gain[0] * hold_00);
//     kalman_filter.yaw_error_covariance[0][1] -= (int32_t)(yaw_kalman_gain[0] * hold_01);
//     kalman_filter.yaw_error_covariance[1][0] -= (int32_t)(yaw_kalman_gain[1] * hold_00);
//     kalman_filter.yaw_error_covariance[1][1] -= (int32_t)(yaw_kalman_gain[1] * hold_01);
// }

/*------------------------------------------------------------------------------------------------*/
/*-end-of-module----------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/
