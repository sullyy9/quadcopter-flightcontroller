#pragma once
/**
 * -------------------------------------------------------------------------------------------------
 * @author  Ryan Sullivan (ryansullivan@googlemail.com)
 *
 * @file    io.h
 * @brief   Header.
 *
 * @date    2021-04-09
 * -------------------------------------------------------------------------------------------------
 */

namespace io {
/*------------------------------------------------------------------------------------------------*/
/*-constant-definitions---------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

typedef enum led
{
    LED_NW = 0,
    LED_N  = 1,
    LED_NE = 2,
    LED_E  = 3,
    LED_SE = 4,
    LED_S  = 5,
    LED_SW = 6,
    LED_W  = 7

} led_t;

typedef enum toggle
{
    ON  = true,
    OFF = false

} toggle_t;

/*------------------------------------------------------------------------------------------------*/
/*-exported-variables-----------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------------------*/
/*-exported-functions-----------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

void initialise(void);

void accelerometer_initialise(void);
bool accelerometer_data_ready(void);
void accelerometer_read(int32_t *accel_x, int32_t *accel_y, int32_t *accel_z);

void magnetometer_initialise(void);
bool magnetometer_data_ready(void);
void magnetometer_read(int32_t *mag_x, int32_t *mag_y, int32_t *mag_z);

void gyroscope_initialise(void);
bool gyroscope_data_ready(void);
void gyroscope_read(int32_t *gyro_x, int32_t *gyro_y, int32_t *gyro_z);

void poll(void);
void external_interupt_1_isr(void);
void external_interupt_2_isr(void);
void external_interupt_4_isr(void);

/*------------------------------------------------------------------------------------------------*/
/*-end-of-module----------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/
}
