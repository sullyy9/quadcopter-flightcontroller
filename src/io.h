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

/*------------------------------------------------------------------------------------------------*/
/*-constant-definitions---------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

typedef enum led
{
    IO_LED_NW = 0,
    IO_LED_N  = 1,
    IO_LED_NE = 2,
    IO_LED_E  = 3,
    IO_LED_SE = 4,
    IO_LED_S  = 5,
    IO_LED_SW = 6,
    IO_LED_W  = 7

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

void io_initialise(void);

void io_led_toggle(led_t led, toggle_t mode);

void io_accelerometer_initialise(void);
bool io_accelerometer_data_ready(void);
void io_accelerometer_read(int32_t *accel_x, int32_t *accel_y, int32_t *accel_z);

void io_magnetometer_initialise(void);
bool io_magnetometer_data_ready(void);
void io_magnetometer_read(int32_t *mag_x, int32_t *mag_y, int32_t *mag_z);

void io_gyroscope_initialise(void);
bool io_gyroscope_data_ready(void);
void io_gyroscope_read(int32_t *gyro_x, int32_t *gyro_y, int32_t *gyro_z);

void io_1ms_poll(void);
void external_interupt_1_isr(void);
void external_interupt_2_isr(void);
void external_interupt_4_isr(void);

/*------------------------------------------------------------------------------------------------*/
/*-end-of-module----------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/
