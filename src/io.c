/*----------------------------------------------------------------------------*/
/*
    Ryan Sullivan

    Module Name     :   io.c
    Description     :   functions for controlling peripherals
*/
/*----------------------------------------------------------------------------*/

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "stm32f3xx_ll_gpio.h"
#include "stm32f3xx_ll_exti.h"
#include "stm32f3xx_ll_system.h"
#include "lsm303agr.h"
#include "i3g4250d.h"

#include "io.h"
#include "commonio.h"
#include "port.h"
#include "usart.h"
#include "i2c.h"
#include "spi.h"

#include "debug.h"

/*----------------------------------------------------------------------------*/
/*-constant-definitions-------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

#define LED_NW      PORT_E8
#define LED_N       PORT_E9
#define LED_NE      PORT_E10
#define LED_E       PORT_E11
#define LED_SE      PORT_E12
#define LED_S       PORT_E13
#define LED_SW      PORT_E14
#define LED_W       PORT_E15

#define DEBUG_TX    PORT_C4
#define DEBUG_RX    PORT_C5

#define ACCEL_CLOCK PORT_B6
#define ACCEL_DATA  PORT_B7
#define ACCEL_DRDY  PORT_E2
#define ACCEL_INT1  PORT_E4
#define ACCEL_INT2  PORT_E5

#define GYRO_CLOCK  PORT_A5
#define GYRO_MISO   PORT_A6
#define GYRO_MOSI   PORT_A7
#define GYRO_CS     PORT_E3
#define GYRO_INT1   PORT_E0
#define GYRO_INT2   PORT_E1

/*----------------------------------------------------------------------------*/
/*-exported-variables---------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-static-variables-----------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

static volatile uint32_t    led_timer           = 0;
static volatile bool        led_n_on            = false;
static          bool        led_w_on            = false;

static volatile bool        accel_data_ready    = true;
static volatile bool        mag_data_ready      = true;
static volatile bool        gyro_data_ready     = true;

/*----------------------------------------------------------------------------*/
/*-forward-declarations-------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

void    gyro_slave_select_on( void );
void    gyro_slave_select_off( void );

void    initialise_pins( void );
void    initialise_external_interupts( void );

/*----------------------------------------------------------------------------*/
/*-exported-functions---------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

void io_initialise( void )
{

    commonio_initialise_clocks( );

    initialise_pins( );

    initialise_external_interupts( );

}

/*----------------------------------------------------------------------------*/

void io_toggle_w_led( void )
{
    if( led_w_on )
    {
        port_clear( LED_W );
        led_w_on = false;
    }
    else
    {
        port_set( LED_W );
        led_w_on = true;
    }
}

/*----------------------------------------------------------------------------*/

void io_fault_led_enable( void )
{
    port_set( LED_S );
}

/*----------------------------------------------------------------------------*/

/*
 * @brief           initialise the accelerometer
 * @param           none
 * @retval          none
 */
void io_accelerometer_initialise( void )
{
    while( i2c1_transfer_in_progress( ) == true );

    /*
     * first data byte is the address of the first control register. MSB of this
     * address enables increment mode so each successive data byte will be written
     * to the next control register along ( 6 in total )
     */
    i2c1_tx_buffer_write( ACCEL_INC( ACCEL_CTRL_REG_1_ADDR ) );

    i2c1_tx_buffer_write( ACCEL_CTRL_REG_1_VAL );
    i2c1_tx_buffer_write( ACCEL_CTRL_REG_2_VAL );
    i2c1_tx_buffer_write( ACCEL_CTRL_REG_3_VAL );
    i2c1_tx_buffer_write( ACCEL_CTRL_REG_4_VAL );
    i2c1_tx_buffer_write( ACCEL_CTRL_REG_5_VAL );
    i2c1_tx_buffer_write( ACCEL_CTRL_REG_6_VAL );

    i2c1_tx_data( ACCEL_I2C_ADDR, true );

}

/*----------------------------------------------------------------------------*/

/*
 * @brief           return whether accelerometer data is ready
 * @param           none
 * @retval          none
 */
bool io_accelerometer_data_ready( void )
{
    return( accel_data_ready );
}

/*----------------------------------------------------------------------------*/

/*
 * @brief           read from the accelerometer
 * @param           none
 * @retval          none
 */
void io_accelerometer_read( int16_t *accel_x, int16_t *accel_y, int16_t *accel_z )
{
    accel_data_ready = false;
    while( i2c1_transfer_in_progress( ) == true );

    /*
     * Incrementally read from all 6 output registers
     */
    i2c1_tx_buffer_write( ACCEL_INC( ACCEL_OUT_REG_X_L_ADDR ) );
    i2c1_tx_data( ACCEL_I2C_ADDR, false );
    while( i2c1_transfer_in_progress( ) == true );

    /*
     * read the data. data is left-justified, 10bit and spread over 2 8bit registers
     * preserve the MSb since the data's 2's compliment
     */
    i2c1_rx_data( ACCEL_I2C_ADDR, 6 );
    while( i2c1_transfer_in_progress( ) == true );

    *accel_x =  ( i2c1_rx_buffer_read( ) );
    *accel_x += ( i2c1_rx_buffer_read( ) << 8 );
    *accel_x = *accel_x >> ( 16 - ACCEL_RESOLUTION );

    *accel_y =  ( i2c1_rx_buffer_read( ) );
    *accel_y += ( i2c1_rx_buffer_read( ) << 8 );
    *accel_y = *accel_y >> ( 16 - ACCEL_RESOLUTION );

    *accel_z =  ( i2c1_rx_buffer_read( ) );
    *accel_z += ( i2c1_rx_buffer_read( ) << 8 );
    *accel_z = *accel_z >> ( 16 - ACCEL_RESOLUTION );
}

/*----------------------------------------------------------------------------*/

/*
 * @brief           initialise the magnetometer
 * @param           none
 * @retval          none
 */
void io_magnetometer_initialise( void )
{
    while( i2c1_transfer_in_progress( ) == true );

    /*
     * first data byte is the address of the first control register. MSB of this
     * address enables increment mode so each successive data byte will be written
     * to the next control register along ( 3 in total )
     */
    i2c1_tx_buffer_write( ACCEL_INC( MAG_CONFIG_REG_A_ADDR ) );

    i2c1_tx_buffer_write( MAG_CONFIG_REG_A_VAL );
    i2c1_tx_buffer_write( MAG_CONFIG_REG_B_VAL );
    i2c1_tx_buffer_write( MAG_CONFIG_REG_C_VAL );
    i2c1_tx_buffer_write( MAG_INT_REG_VAL );

    i2c1_tx_data( MAG_I2C_ADDR, true );

}

/*----------------------------------------------------------------------------*/

/*
 * @brief           return whether magnetometer data is ready
 * @param           none
 * @retval          none
 */
bool io_magnetometer_data_ready( void )
{
    return( mag_data_ready );
}

/*----------------------------------------------------------------------------*/

/*
 * @brief           read from the magnetometer
 * @param           none
 * @retval          none
 */
void io_magnetometer_read( int16_t *mag_x, int16_t *mag_y, int16_t *mag_z )
{
    mag_data_ready = false;
    while( i2c1_transfer_in_progress( ) == true );

    /*
     * Incrementally read from all 6 output registers
     */
    i2c1_tx_buffer_write( ACCEL_INC( MAG_OUT_REG_X_L_ADDR ) );
    i2c1_tx_data( MAG_I2C_ADDR, false );
    while( i2c1_transfer_in_progress( ) == true );

    /*
     * read the data. data is 16bit and spread over 2 8bit registers
     * preserve the MSb since the data's 2's compliment
     */
    i2c1_rx_data( MAG_I2C_ADDR, 6 );
    while( i2c1_transfer_in_progress( ) == true );

    *mag_x =  ( i2c1_rx_buffer_read( ) );
    *mag_x += ( i2c1_rx_buffer_read( ) << 8 );
    *mag_y =  ( i2c1_rx_buffer_read( ) );
    *mag_y += ( i2c1_rx_buffer_read( ) << 8 );
    *mag_z =  ( i2c1_rx_buffer_read( ) );
    *mag_z += ( i2c1_rx_buffer_read( ) << 8 );
}

/*----------------------------------------------------------------------------*/

/*
 * @brief           initialise the magnetometer
 * @param           none
 * @retval          none
 */
void io_gyroscope_initialise( void )
{
    while( spi1_transfer_in_progress( ) == true );

    spi1_tx_buffer_write( GYRO_WRITE( GYRO_INC( GYRO_CTRL_REG_1_ADDR ) ) );
    spi1_tx_buffer_write( GYRO_CTRL_REG_1_VAL );
    spi1_tx_buffer_write( GYRO_CTRL_REG_2_VAL );
    spi1_tx_buffer_write( GYRO_CTRL_REG_3_VAL );
    spi1_tx_buffer_write( GYRO_CTRL_REG_4_VAL );
    spi1_tx_buffer_write( GYRO_CTRL_REG_5_VAL );

    gyro_slave_select_on( );
    spi1_transfer_data( 0 );
    while( spi1_transfer_in_progress( ) == true );
    gyro_slave_select_off( );
}

/*----------------------------------------------------------------------------*/

/*
 * @brief           return whether gyroscope data is ready
 * @param           none
 * @retval          none
 */
bool io_gyroscope_data_ready( void )
{
    return( gyro_data_ready );
}

/*----------------------------------------------------------------------------*/

/*
 * @brief           read from the gyroscope
 * @param           none
 * @retval          none
 */
void io_gyroscope_read( int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z )
{
    gyro_data_ready = false;
    while( spi1_transfer_in_progress( ) == true );

    /*
     * Incrementally read from all 6 output registers
     */
    spi1_tx_buffer_write( GYRO_READ( GYRO_INC( GYRO_OUT_REG_X_L_ADDR ) ) );

    gyro_slave_select_on( );
    spi1_transfer_data( 6 );
    while( spi1_transfer_in_progress( ) == true );
    gyro_slave_select_off( );

    /*
     * read the data. data is 16bit and spread over 2 8bit registers
     * preserve the MSb since the data's 2's compliment. first read
     * removes the 0 from the address transmision
     */
    spi1_rx_buffer_read( );
    *gyro_x =  ( spi1_rx_buffer_read( ) );
    *gyro_x += ( spi1_rx_buffer_read( ) << 8 );
    *gyro_y =  ( spi1_rx_buffer_read( ) );
    *gyro_y += ( spi1_rx_buffer_read( ) << 8 );
    *gyro_z =  ( spi1_rx_buffer_read( ) );
    *gyro_z += ( spi1_rx_buffer_read( ) << 8 );
}

/*----------------------------------------------------------------------------*/

void io_1ms_poll( void )
{
    if( led_timer >= 1000 )
    {
        led_timer = 0;
        if( led_n_on )
        {
            port_clear( LED_N );
            led_n_on = false;
        }
        else
        {
            port_set( LED_N );
            led_n_on = true;
        }
    }
    led_timer++;
}

/*----------------------------------------------------------------------------*/

/*
 * ISR for the gyroscope data ready interrupt
 */
void external_interupt_1_isr( void )
{
    LL_EXTI_ClearFlag_0_31( LL_EXTI_LINE_1 );
    gyro_data_ready = true;
}

/*----------------------------------------------------------------------------*/

/*
 * ISR for the magnetometer data ready interrupt
 */
void external_interupt_2_isr( void )
{
    LL_EXTI_ClearFlag_0_31( LL_EXTI_LINE_2 );
    mag_data_ready = true;
}

/*----------------------------------------------------------------------------*/

/*
 * ISR for the accelerometer data ready interrupt
 */
void external_interupt_4_isr( void )
{
    LL_EXTI_ClearFlag_0_31( LL_EXTI_LINE_4 );
    accel_data_ready = true;
}

/*----------------------------------------------------------------------------*/
/*-static-functions-----------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

void gyro_slave_select_on( void )
{
    port_clear( GYRO_CS );
}

/*----------------------------------------------------------------------------*/

void gyro_slave_select_off( void )
{
    port_set( GYRO_CS );
}

/*----------------------------------------------------------------------------*/

/*
 * @brief           initialise any used pins
 * @param           none
 * @retval          none
 */
void initialise_pins( void )
{
    /*
     * GPIO
     */
    port_initialise_pin( LED_N,     PORT_MODE_PUSH_PULL, 0 );
    port_initialise_pin( LED_NE,    PORT_MODE_PUSH_PULL, 0 );
    port_initialise_pin( LED_E,     PORT_MODE_PUSH_PULL, 0 );
    port_initialise_pin( LED_SE,    PORT_MODE_PUSH_PULL, 0 );
    port_initialise_pin( LED_S,     PORT_MODE_PUSH_PULL, 0 );
    port_initialise_pin( LED_SW,    PORT_MODE_PUSH_PULL, 0 );
    port_initialise_pin( LED_W,     PORT_MODE_PUSH_PULL, 0 );
    port_initialise_pin( LED_NW,    PORT_MODE_PUSH_PULL, 0 );

    /*
     * Debug
     */
    port_initialise_pin( DEBUG_TX, PORT_MODE_ALT_OUTPUT, 7 );
    port_initialise_pin( DEBUG_RX, PORT_MODE_INPUT_PULLUP, 7 );

    /*
     * Accelerometer / Magnetometer
     */
    port_initialise_pin( ACCEL_CLOCK,   PORT_MODE_ALT_OPEN_DRAIN,   4 );
    port_initialise_pin( ACCEL_DATA,    PORT_MODE_ALT_OPEN_DRAIN,   4 );
    port_initialise_pin( ACCEL_DRDY,    PORT_MODE_INPUT_PULLDOWN,   0 );
    port_initialise_pin( ACCEL_INT1,    PORT_MODE_INPUT_PULLDOWN,   0 );
    port_initialise_pin( ACCEL_INT2,    PORT_MODE_INPUT_PULLDOWN,   0 );

    /*
     * Gyroscope
     */
    port_set( GYRO_CS );
    port_initialise_pin( GYRO_CLOCK,    PORT_MODE_ALT_OUTPUT,       5 );
    port_initialise_pin( GYRO_MISO,     PORT_MODE_ALT_OUTPUT,       5 );
    port_initialise_pin( GYRO_MOSI,     PORT_MODE_ALT_OUTPUT,       5 );
    port_initialise_pin( GYRO_CS,       PORT_MODE_PUSH_PULL,        0 );
    port_initialise_pin( GYRO_INT1,     PORT_MODE_INPUT_PULLDOWN,   0 );
    port_initialise_pin( GYRO_INT2,     PORT_MODE_INPUT_PULLDOWN,   0 );
}

/*----------------------------------------------------------------------------*/

/*
 * @brief           initialise external interupts
 * @param           none
 * @retval          none
 */
void initialise_external_interupts( void )
{
    /*
     * Accelerometer data ready
     */
    LL_EXTI_InitTypeDef exti_initialisation_structure;
    exti_initialisation_structure.LineCommand   = ENABLE;
    exti_initialisation_structure.Line_0_31     = LL_EXTI_LINE_4;
    exti_initialisation_structure.Line_32_63    = LL_EXTI_LINE_NONE;
    exti_initialisation_structure.Mode          = LL_EXTI_MODE_IT;
    exti_initialisation_structure.Trigger       = LL_EXTI_TRIGGER_RISING;
    LL_EXTI_Init( &exti_initialisation_structure );
    LL_SYSCFG_SetEXTISource( LL_SYSCFG_EXTI_PORTE, LL_SYSCFG_EXTI_LINE4 );
    NVIC_SetPriority( EXTI4_IRQn, 3 );
    NVIC_EnableIRQ( EXTI4_IRQn );

    LL_EXTI_ClearFlag_0_31( LL_EXTI_LINE_4 );

    /*
     * Magnetometer data ready
     */
    exti_initialisation_structure.LineCommand   = ENABLE;
    exti_initialisation_structure.Line_0_31     = LL_EXTI_LINE_2;
    exti_initialisation_structure.Line_32_63    = LL_EXTI_LINE_NONE;
    exti_initialisation_structure.Mode          = LL_EXTI_MODE_IT;
    exti_initialisation_structure.Trigger       = LL_EXTI_TRIGGER_RISING;
    LL_EXTI_Init( &exti_initialisation_structure );
    LL_SYSCFG_SetEXTISource( LL_SYSCFG_EXTI_PORTE, LL_SYSCFG_EXTI_LINE2 );
    NVIC_SetPriority( EXTI2_TSC_IRQn, 3 );
    NVIC_EnableIRQ( EXTI2_TSC_IRQn );

    LL_EXTI_ClearFlag_0_31( LL_EXTI_LINE_2 );

    /*
     * Gyroscope data ready
     */
    exti_initialisation_structure.LineCommand   = ENABLE;
    exti_initialisation_structure.Line_0_31     = LL_EXTI_LINE_1;
    exti_initialisation_structure.Line_32_63    = LL_EXTI_LINE_NONE;
    exti_initialisation_structure.Mode          = LL_EXTI_MODE_IT;
    exti_initialisation_structure.Trigger       = LL_EXTI_TRIGGER_RISING;
    LL_EXTI_Init( &exti_initialisation_structure );
    LL_SYSCFG_SetEXTISource( LL_SYSCFG_EXTI_PORTE, LL_SYSCFG_EXTI_LINE1 );
    NVIC_SetPriority( EXTI1_IRQn, 3 );
    NVIC_EnableIRQ( EXTI1_IRQn );

    LL_EXTI_ClearFlag_0_31( LL_EXTI_LINE_1 );
}

/*----------------------------------------------------------------------------*/
/*-end-of-module--------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
