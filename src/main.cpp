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

#include <cstdint>
#include <cmath>
#include <optional>
#include <system_error>
#include <utility>

#include "usart.hpp"
#include "usart_stm32f303.hpp"

#include "esc.hpp"
#include "i3g4250d.hpp"
#include "lsm303agr.hpp"

#include "io.hpp"
#include "main.hpp"
#include "clocks.hpp"
#include "system_info.hpp"
#include "utils.hpp"
#include "watchdog.hpp"
#include "debug.hpp"
#include "pwm.hpp"
#include "motor.hpp"

/*------------------------------------------------------------------------------------------------*/
/*-constant-definitions---------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

#define PI 3.141592f

#define OUTPUT_DATA true

using SerialDebug = debug::Serial<usart::stm32f303::USART>;

using Motor1 = motor::Motor<pwm::Motor1PWM, esc::HobbywingXRotor>;
using Motor2 = motor::Motor<pwm::Motor2PWM, esc::HobbywingXRotor>;
using Motor3 = motor::Motor<pwm::Motor3PWM, esc::HobbywingXRotor>;
using Motor4 = motor::Motor<pwm::Motor4PWM, esc::HobbywingXRotor>;

/*------------------------------------------------------------------------------------------------*/
/*-exported-variables-----------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------------------*/
/*-static-variables-------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

static volatile bool     run_program       = true;
static volatile uint32_t system_runtime_ms = 0;

static volatile decltype(debug::stopwatch_stop()) loop_runtime_ns {0};

typedef struct accel_data
{
    int32_t x_raw; // x in the range of the register size, reflecting the
                   // acceleration in gs
    int32_t y_raw; // y in the range of the register size, reflecting the
                   // acceleration in gs
    int32_t z_raw; // z in the range of the register size, reflecting the
                   // acceleration in gs

    float x_g; // x in terms of earth gravity
    float y_g; // y in terms of earth gravity
    float z_g; // z in terms of earth gravity

    float attitude_rad; // attitude in radians
    float bank_rad;     // bank in radians

    float attitude_deg; // attitude in degrees
    float bank_deg;     // bank in degrees

} accel_data_t;

typedef struct mag_data
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

typedef struct gyro_data
{
    int32_t x_raw; // x in the range of the register size, reflecting rotation in
                   // degrees per second
    int32_t y_raw; // y in the range of the register size, reflecting rotation in
                   // degrees per second
    int32_t z_raw; // z in the range of the register size, reflecting rotation in
                   // degrees per second

    float pitch_dps; // pitch in degrees per second
    float roll_dps;  // roll in degrees per second
    float yaw_dps;   // yaw in degrees per second

} gyro_data_t;

typedef struct time_data
{
    int32_t current_us;
    int32_t previous_us;
    int32_t change_us;

} time_data_t;

typedef struct kalman_data
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

static accel_data_t  accel_data;
static mag_data_t    mag_data;
static gyro_data_t   gyro_data;
static time_data_t   time_data;
static kalman_data_t kalman_data;

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
    // Initialise clocks.
    // Clock configuration can be found in clocks_config.hpp.
    {
        clk::Error status = clk::initialise();
        if(status != clk::OK)
        {
            while(true) {}
        }
    }

    io::initialise();
    clk::clear_reset_flags();
    utils::wait_ms(1000);
    if(SerialDebug::init({
        .baud_rate = 115'200,
        .enable_rx = false,
        .enable_dma = true,
    }) != usart::StatusCode::Ok){
        return -1;
    }

    debug::stopwatch_initialise();

    SerialDebug::print("\r\n");
    SerialDebug::print("\r\n");
    SerialDebug::print("\r\n");
    SerialDebug::print("Quadcopter flight controller\r\n");
    SerialDebug::print("----------------------------------------\r\n");
    SerialDebug::print("initialisation complete\r\n");
    SerialDebug::flush();

    io::accelerometer_initialise();
    io::magnetometer_initialise();
    io::gyroscope_initialise();

    // Setup the PWM timer.
    auto timer2_status {pwm::Timer2::init({.frequency = 400})};
    SerialDebug::println("% Timer - %", timer2_status.category().name(), timer2_status.message().c_str());
    SerialDebug::flush();
    if(timer2_status != pwm::StatusCode::Ok) {
        return -1;
    }

    // Setup Motor instances.
    auto [motor1, motor1_status] = Motor1::init();
    auto [motor2, motor2_status] = Motor2::init();
    auto [motor3, motor3_status] = Motor3::init();
    auto [motor4, motor4_status] = Motor4::init();
    SerialDebug::println("% 1 - %", motor1_status.category().name(), motor1_status.message().c_str());
    SerialDebug::println("% 2 - %", motor2_status.category().name(), motor2_status.message().c_str());
    SerialDebug::println("% 3 - %", motor3_status.category().name(), motor3_status.message().c_str());
    SerialDebug::println("% 4 - %", motor4_status.category().name(), motor4_status.message().c_str());
    SerialDebug::flush();
    if(!(motor1.has_value() && motor2.has_value() && motor3.has_value() && motor4.has_value())) {
        return 1;
    }


    // motor1->set_throttle(0);
    // motor2->set_throttle(0);
    // motor3->set_throttle(0);
    // motor4->set_throttle(0);

    // utils::wait_ms(5000);

    // motor1->set_throttle(6);
    // motor2->set_throttle(6);
    // motor3->set_throttle(6);
    // motor4->set_throttle(6);

    // utils::wait_ms(1000);

    // motor1->set_throttle(0);
    // motor2->set_throttle(0);
    // motor3->set_throttle(0);
    // motor4->set_throttle(0);
    
    // Throttle calibration.
    // utils::wait_ms(5000);
    // motor1->set_throttle(100);
    // motor2->set_throttle(100);
    // motor3->set_throttle(100);
    // motor4->set_throttle(100);
    // utils::wait_ms(2000);
    // motor1->set_throttle(0);
    // motor2->set_throttle(0);
    // motor3->set_throttle(0);
    // motor4->set_throttle(0);

    // while(true);

    // Setup the indipendant watchdog.
    auto [wdg_result, wdg_status] {iwdg::Watchdog::with_timeout(sys::Seconds{1})};
    SerialDebug::println("% - %", wdg_status.category().name(), wdg_status.message().c_str());
    if (!wdg_result.has_value()) {
        SerialDebug::flush();
        return -1;
    }
    auto watchdog {std::exchange(wdg_result, std::nullopt).value()};

    while(run_program)
    {
        watchdog.update();
        
        /*
         * read acceleration data if its ready
         */
        if(io::accelerometer_data_ready())
        {
            io::accelerometer_read(&accel_data.x_raw, &accel_data.y_raw, &accel_data.z_raw);

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
        if(io::magnetometer_data_ready())
        {
            io::magnetometer_read(&mag_data.x_raw, &mag_data.y_raw, &mag_data.z_raw);

            mag_data.x_gauss = (((float)mag_data.x_raw * 1.5f) / 1000);
            mag_data.y_gauss = (((float)mag_data.y_raw * 1.5f) / 1000);
            mag_data.z_gauss = (((float)mag_data.z_raw * 1.5f) / 1000);

            new_mag_data = true;
        }

        /*
         * read gyroscope data if its ready
         */
        if(io::gyroscope_data_ready())
        {
            io::gyroscope_read(&gyro_data.x_raw, &gyro_data.y_raw, &gyro_data.z_raw);

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
            debug::stopwatch_start();
            new_accel_data = false;
            new_mag_data   = false;
            new_gyro_data  = false;

            /*
             * Calculate the change in time
             * TODO can the data rates of each device be synchronised?
             */
            time_data.current_us =
                (int32_t)((system_runtime_ms * 1000) + clk::get_system_timer_us());

            time_data.change_us = (time_data.current_us - time_data.previous_us);

            time_data.previous_us = time_data.current_us;

            /*
             * Convert acceleration and magnetometer data to roll, pitch and yaw
             * angle. If the modulus of the acceleration or magnetometer values is not
             * near 1g or earth's field strength respectively, we can't use the data.
             *
             * See document DT0058 for calculation details however:
             * The documentation takes x as the forward facing axis, so roll is around
             * the x axis and pitch is around the y axis. I have flipped that so that
             * y is the forward facing axis.
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
                 * Calculate accelerometer pitch angle. Must first calculate Z in
                 * respect to earth rather than its own orientation
                 */
                float gz2 = ((accel_data.x_g * sinf(accel_data.bank_rad)) +
                             (accel_data.z_g * cosf(accel_data.bank_rad)));

                accel_data.attitude_rad = atanf(accel_data.y_g / gz2);

                /*
                 * Calculate accelerometer and magnetometer yaw angle if the data is
                 * good. Earth's normal field strength is between 300mG ( southern
                 * hemisphere ) and 600mG ( northern hemisphere ).
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

                apply_kalman_filter();
            }
            loop_runtime_ns = debug::stopwatch_stop();

            static uint32_t print_every = 0;
            print_every++;
            if((OUTPUT_DATA == true) && (print_every == 10)) {
                print_every = 0;
                debug::stopwatch_start();

                SerialDebug::print("\r\n");
                SerialDebug::print("orientation data\r\n");
                SerialDebug::print("D:STIME:%\r\n", system_runtime_ms);
                SerialDebug::print("D:RTIME:%\r\n", loop_runtime_ns);

                SerialDebug::print("D:ABANK:%\r\n", (int32_t)accel_data.bank_deg);
                SerialDebug::print("D:KBANK:%\r\n", (int32_t)kalman_data.bank_deg);
                SerialDebug::print("D:AATTITUDE:%\r\n", (int32_t)accel_data.attitude_deg);
                SerialDebug::print("D:KATTITUDE:%\r\n", (int32_t)kalman_data.attitude_deg);
                SerialDebug::print("D:MHEADING:%\r\n", (int32_t)(mag_data.heading_deg));
                SerialDebug::print("D:KHEADING:%\r\n", (int32_t)kalman_data.heading_deg);

                SerialDebug::print("D:ACCELX:%\r\n", (int32_t)(accel_data.x_g * 1000));
                SerialDebug::print("D:ACCELY:%\r\n", (int32_t)(accel_data.y_g * 1000));
                SerialDebug::print("D:ACCELZ:%\r\n", (int32_t)(accel_data.z_g * 1000));

                SerialDebug::print("D:MAGX:%\r\n", (int32_t)(mag_data.x_gauss * 1000));
                SerialDebug::print("D:MAGY:%\r\n", (int32_t)(mag_data.y_gauss * 1000));
                SerialDebug::print("D:MAGZ:%\r\n", (int32_t)(mag_data.z_gauss * 1000));

                SerialDebug::print("D:GYROR:%\r\n", (int32_t)gyro_data.roll_dps);
                SerialDebug::print("D:GYROP:%\r\n", (int32_t)gyro_data.pitch_dps);
                SerialDebug::print("D:GYROY:%\r\n", (int32_t)gyro_data.yaw_dps);
                SerialDebug::print("\r\n");
                SerialDebug::flush();
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
    io::poll();    // NOLINT - ignore clangd warning
    utils::poll(); // NOLINT - ignore clangd warning
    system_runtime_ms++;
}

/*------------------------------------------------------------------------------------------------*/
/*-static-functions-------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/**
 * @brief Apply a kalman filter to the acceleroeter, magnetometer and gyroscope
 * data sets.
 *
 * http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/
 */
void apply_kalman_filter(void)
{
    float delta_t;
    delta_t = ((float)time_data.change_us / 1000000);

    // debug_printf("delta_t: %d\r\n", (int32_t)(delta_t * 1000));

    /*
     * Roll angle
     */
    const float q_bias_roll    = 0.003f;
    const float q_angle_roll   = 0.001f;
    const float r_measure_roll = 0.03f;

    /*
     * Estimate angle from gyroscope data
     */
    float roll_rate_dps;
    roll_rate_dps = gyro_data.roll_dps - kalman_data.bank_drift;

    kalman_data.bank_deg += (roll_rate_dps * delta_t);

    /*
     * Calculate the error covariance ( estimate trust in estimated angle )
     */
    kalman_data.bank_error_covariance[0][0] +=
        (delta_t * ((delta_t * kalman_data.bank_error_covariance[1][1]) -
                    kalman_data.bank_error_covariance[0][1] -
                    kalman_data.bank_error_covariance[1][0] + q_angle_roll));

    kalman_data.bank_error_covariance[0][1] -= (delta_t * kalman_data.bank_error_covariance[1][1]);
    kalman_data.bank_error_covariance[1][0] -= (delta_t * kalman_data.bank_error_covariance[1][1]);
    kalman_data.bank_error_covariance[1][1] += (q_bias_roll * delta_t);

    /*
     * Calculate innovation ( difference between estimated angle and
     accelerometer angle ).
     */
    float roll_innovation;
    roll_innovation = accel_data.bank_deg - kalman_data.bank_deg;

    /*
     * Estimate trust in the accelerometer measurement
     */
    float roll_innovation_covariance;
    roll_innovation_covariance = kalman_data.bank_error_covariance[0][0] + r_measure_roll;

    /*
     * Calculate Kalman gain
     */
    float roll_kalman_gain[2];
    roll_kalman_gain[0] = kalman_data.bank_error_covariance[0][0] / roll_innovation_covariance;
    roll_kalman_gain[1] = kalman_data.bank_error_covariance[1][0] / roll_innovation_covariance;

    /*
     * Update the estimate angle
     */
    kalman_data.bank_deg += (roll_kalman_gain[0] * roll_innovation);
    kalman_data.bank_drift += (roll_kalman_gain[1] * roll_innovation);

    /*
     * Update the error covariance
     */
    float hold_00 = kalman_data.bank_error_covariance[0][0];
    float hold_01 = kalman_data.bank_error_covariance[0][1];

    kalman_data.bank_error_covariance[0][0] -= (roll_kalman_gain[0] * hold_00);
    kalman_data.bank_error_covariance[0][1] -= (roll_kalman_gain[0] * hold_01);
    kalman_data.bank_error_covariance[1][0] -= (roll_kalman_gain[1] * hold_00);
    kalman_data.bank_error_covariance[1][1] -= (roll_kalman_gain[1] * hold_01);

    /*
     * Pitch angle
     */
    const float q_bias_pitch    = 0.003f;
    const float q_angle_pitch   = 0.001f;
    const float r_measure_pitch = 0.03f;

    /*
     * Estimate angle from gyroscope data
     */
    float pitch_rate_dps;
    pitch_rate_dps = gyro_data.pitch_dps - kalman_data.attitude_drift;

    kalman_data.attitude_deg += ((pitch_rate_dps * delta_t) / 1000);

    /*
     * Calculate the error covariance ( estimate trust in estimated angle )
     */
    kalman_data.attitude_error_covariance[0][0] +=
        (delta_t * ((delta_t * kalman_data.attitude_error_covariance[1][1]) -
                    kalman_data.attitude_error_covariance[0][1] -
                    kalman_data.attitude_error_covariance[1][0] + q_angle_pitch));

    kalman_data.attitude_error_covariance[0][1] -=
        (delta_t * kalman_data.attitude_error_covariance[1][1]);

    kalman_data.attitude_error_covariance[1][0] -=
        (delta_t * kalman_data.attitude_error_covariance[1][1]);

    kalman_data.attitude_error_covariance[1][1] += (q_bias_pitch * delta_t);

    /*
     * Calculate innovation ( difference between estimated angle and
     accelerometer angle ).
     */
    float pitch_innovation;
    pitch_innovation = (accel_data.attitude_deg - kalman_data.attitude_deg);

    /*
     * Estimate trust in the accelerometer measurement
     */
    float pitch_innovation_covariance;
    pitch_innovation_covariance = kalman_data.attitude_error_covariance[0][0] + r_measure_pitch;

    /*
     * Calculate Kalman gain
     */
    float pitch_kalman_gain[2];
    pitch_kalman_gain[0] =
        kalman_data.attitude_error_covariance[0][0] / pitch_innovation_covariance;

    pitch_kalman_gain[1] =
        kalman_data.attitude_error_covariance[1][0] / pitch_innovation_covariance;

    /*
     * Update the estimate angle
     */
    kalman_data.attitude_deg += (pitch_kalman_gain[0] * pitch_innovation);
    kalman_data.attitude_drift += (pitch_kalman_gain[1] * pitch_innovation);

    /*
     * Update the error covariance
     */
    hold_00 = kalman_data.attitude_error_covariance[0][0];
    hold_01 = kalman_data.attitude_error_covariance[0][1];

    kalman_data.attitude_error_covariance[0][0] -= (pitch_kalman_gain[0] * hold_00);
    kalman_data.attitude_error_covariance[0][1] -= (pitch_kalman_gain[0] * hold_01);
    kalman_data.attitude_error_covariance[1][0] -= (pitch_kalman_gain[1] * hold_00);
    kalman_data.attitude_error_covariance[1][1] -= (pitch_kalman_gain[1] * hold_01);

    /*
     * yaw angle
     */
    const float q_bias_yaw    = 0.003f;
    const float q_angle_yaw   = 0.001f;
    const float r_measure_yaw = 0.03f;

    /*
     * Estimate angle from gyroscope data
     */
    float yaw_rate_dps;
    yaw_rate_dps = (gyro_data.yaw_dps - kalman_data.heading_drift);

    kalman_data.heading_deg += ((yaw_rate_dps * delta_t) / 1000);

    /*
     * Calculate the error covariance ( estimate trust in estimated angle )
     */

    kalman_data.heading_error_covariance[0][0] +=
        (delta_t * ((delta_t * kalman_data.heading_error_covariance[1][1]) -
                    kalman_data.heading_error_covariance[0][1] -
                    kalman_data.heading_error_covariance[1][0] + q_angle_yaw));

    kalman_data.heading_error_covariance[0][1] -=
        (delta_t * kalman_data.heading_error_covariance[1][1]);

    kalman_data.heading_error_covariance[1][0] -=
        (delta_t * kalman_data.heading_error_covariance[1][1]);

    kalman_data.heading_error_covariance[1][1] += (q_bias_yaw * delta_t);

    /*
     * Calculate innovation ( difference between estimated angle and
     accelerometer angle ).
     */
    float yaw_innovation;
    yaw_innovation = (mag_data.heading_deg - kalman_data.heading_deg);

    /*
     * Estimate trust in the accelerometer measurement
     */
    float yaw_innovation_covariance;
    yaw_innovation_covariance = (kalman_data.heading_error_covariance[0][0] + r_measure_yaw);

    /*
     * Calculate Kalman gain
     */
    float yaw_kalman_gain[2];
    yaw_kalman_gain[0] = kalman_data.heading_error_covariance[0][0] / yaw_innovation_covariance;
    yaw_kalman_gain[1] = kalman_data.heading_error_covariance[1][0] / yaw_innovation_covariance;

    /*
     * Update the estimate angle
     */
    kalman_data.heading_deg += (yaw_kalman_gain[0] * yaw_innovation);
    kalman_data.heading_drift += (yaw_kalman_gain[1] * yaw_innovation);

    /*
     * Update the error covariance
     */
    hold_00 = kalman_data.heading_error_covariance[0][0];
    hold_01 = kalman_data.heading_error_covariance[0][1];

    kalman_data.heading_error_covariance[0][0] -= (yaw_kalman_gain[0] * hold_00);
    kalman_data.heading_error_covariance[0][1] -= (yaw_kalman_gain[0] * hold_01);
    kalman_data.heading_error_covariance[1][0] -= (yaw_kalman_gain[1] * hold_00);
    kalman_data.heading_error_covariance[1][1] -= (yaw_kalman_gain[1] * hold_01);
}

/*------------------------------------------------------------------------------------------------*/
/*-end-of-module----------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/
