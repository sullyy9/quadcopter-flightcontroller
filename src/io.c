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
#include "accelerometer.h"

#include "debug.h"
#include "port.h"
#include "commonio.h"
#include "i2c.h"
#include "usart.h"
#include "io.h"

/*----------------------------------------------------------------------------*/
/*-constant-definitions-------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

#define LED_NW              PORT_E8
#define LED_N               PORT_E9
#define LED_NE              PORT_E10
#define LED_E               PORT_E11
#define LED_SE              PORT_E12
#define LED_S               PORT_E13
#define LED_SW              PORT_E14
#define LED_W               PORT_E15

#define DEBUG_TX            PORT_C4
#define DEBUG_RX            PORT_C5

#define ACCELEROMETER_CLOCK PORT_B6
#define ACCELEROMETER_DATA  PORT_B7
#define ACCELEROMETER_DRDY  PORT_E2
#define ACCELEROMETER_INT1  PORT_E4
#define ACCELEROMETER_INT2  PORT_E5

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

/*----------------------------------------------------------------------------*/
/*-forward-declarations-------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

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
 * @brief           initialise the accelerometer
 * @param           none
 * @retval          none
 */
void io_initialise_accelerometer( void )
{
    i2c1_tx_buffer_clear( );

    /*
     * first data byte is the address of the first control register. MSB of this
     * address enables increment mode so each successive data byte will be written
     * to the next control register along ( 6 in total )
     */
    i2c1_tx_buffer_write( INCREMENT( ACCEL_CTRL_REG_1_ADDRESS ) );

    i2c1_tx_buffer_write( ACCEL_CTRL_REG_1_VALUE );
    i2c1_tx_buffer_write( ACCEL_CTRL_REG_2_VALUE );
    i2c1_tx_buffer_write( ACCEL_CTRL_REG_3_VALUE );
    i2c1_tx_buffer_write( ACCEL_CTRL_REG_4_VALUE );
    i2c1_tx_buffer_write( ACCEL_CTRL_REG_5_VALUE );
    i2c1_tx_buffer_write( ACCEL_CTRL_REG_6_VALUE );

    i2c1_tx_data( ACCEL_I2C_ADDRESS, true );

}

/*----------------------------------------------------------------------------*/

/*
 * @brief           read from the accelerometer
 * @param           none
 * @retval          none
 */
void io_read_accelerometer( int16_t *accel_x, int16_t *accel_y, int16_t *accel_z )
{
    accel_data_ready = false;

    i2c1_tx_buffer_clear( );
    i2c1_rx_buffer_clear( );

    /*
     * first data byte is the address of the first control register. MSB of this
     * address enables increment mode so each successive data byte will be written
     * to the next control register along ( 6 in total )
     */
    i2c1_tx_buffer_write( INCREMENT( ACCEL_OUT_REG_X_L_ADDRESS ) );

    i2c1_tx_data( ACCEL_I2C_ADDRESS, false );

    /*
     * read the data. data is left-justified, 10bit and spread over 2 8bit registers
     * preserve the MSb since the data's 2's compliment
     */
    i2c1_rx_data( ACCEL_I2C_ADDRESS, 6 );
    while( i2c1_rx_buffer_available( ) == 0 );

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

void io_1ms_poll( void )
{
    if( led_timer >= 200 )
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

void external_interupt_4_isr( void )
{
    LL_EXTI_ClearFlag_0_31( LL_EXTI_LINE_4 );
    accel_data_ready = true;
}

/*----------------------------------------------------------------------------*/
/*-static-functions-----------------------------------------------------------*/
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
    port_initialise_pin( LED_N, PORT_MODE_PUSH_PULL, 0 );
    port_initialise_pin( LED_W, PORT_MODE_PUSH_PULL, 0 );

    /*
     * Debug
     */
    port_initialise_pin( DEBUG_TX, PORT_MODE_ALT_OUTPUT, 7 );
    port_initialise_pin( DEBUG_RX, PORT_MODE_INPUT_PULLUP, 7 );

    /*
     * Accelerometer
     */
    port_initialise_pin( ACCELEROMETER_CLOCK,   PORT_MODE_ALT_OPEN_DRAIN,   4 );
    port_initialise_pin( ACCELEROMETER_DATA,    PORT_MODE_ALT_OPEN_DRAIN,   4 );
    port_initialise_pin( ACCELEROMETER_DRDY,    PORT_MODE_FLOATING,         0 );
    port_initialise_pin( ACCELEROMETER_INT1,    PORT_MODE_FLOATING,         0 );
    port_initialise_pin( ACCELEROMETER_INT2,    PORT_MODE_FLOATING,         0 );
    port_set( ACCELEROMETER_CLOCK );
    port_set( ACCELEROMETER_DATA );

}

/*----------------------------------------------------------------------------*/

/*
 * @brief           initialise external interupts
 * @param           none
 * @retval          none
 */
void initialise_external_interupts( void )
{
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
}

/*----------------------------------------------------------------------------*/
/*-end-of-module--------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
