/**
 * -------------------------------------------------------------------------------------------------
 * @author  Ryan Sullivan (ryansullivan@googlemail.com)
 *
 * @file    io.c
 * @brief   Module for controlling GPIO and external devices
 *
 * @date    2021-04-05
 * -------------------------------------------------------------------------------------------------
 */

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "stm32f3xx_ll_gpio.h"
#include "stm32f3xx_ll_exti.h"
#include "stm32f3xx_ll_system.h"
#include "lsm303agr.h"
#include "i3g4250d.h"

#include "io.h"
#include "system.h"
#include "port.h"
#include "usart.h"
#include "i2c.h"
#include "spi.h"

#include "debug.h"

/*------------------------------------------------------------------------------------------------*/
/*-constant-definitions---------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

#define LED_NW PORT_E8
#define LED_N  PORT_E9
#define LED_NE PORT_E10
#define LED_E  PORT_E11
#define LED_SE PORT_E12
#define LED_S  PORT_E13
#define LED_SW PORT_E14
#define LED_W  PORT_E15

#define DEBUG_TX PORT_C4
#define DEBUG_RX PORT_C5

#define ACCEL_CLOCK PORT_B6
#define ACCEL_DATA  PORT_B7
#define ACCEL_DRDY  PORT_E2
#define ACCEL_INT1  PORT_E4
#define ACCEL_INT2  PORT_E5

#define GYRO_CLOCK PORT_A5
#define GYRO_MISO  PORT_A6
#define GYRO_MOSI  PORT_A7
#define GYRO_CS    PORT_E3
#define GYRO_INT1  PORT_E0
#define GYRO_INT2  PORT_E1

/*------------------------------------------------------------------------------------------------*/
/*-exported-variables-----------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------------------*/
/*-static-variables-------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

static volatile uint32_t led_timer  = 0;
static volatile uint32_t led_active = LED_NW;

static volatile bool accel_data_ready = true;
static volatile bool mag_data_ready   = true;
static volatile bool gyro_data_ready  = true;

/*------------------------------------------------------------------------------------------------*/
/*-forward-declarations---------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

void gyro_slave_select_toggle(toggle_t mode);

void initialise_pins(void);
void initialise_external_interupts(void);

/*------------------------------------------------------------------------------------------------*/
/*-exported-functions-----------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/**
 * @brief Initialise any IO.
 */
void io_initialise(void)
{
    system_initialise_clocks();

    initialise_pins();

    initialise_external_interupts();

    usart_initialise();
    i2c_initialise();
    spi_initialise();
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief      Toggle an LED on or off.
 * @param led  LED to toggle.
 * @param mode ON or OFF.
 */
void io_led_toggle(led_t led, toggle_t mode)
{
    uint32_t pin = LED_NW + led;

    if(mode == true)
    {
        port_set(pin);
    }
    else
    {
        port_clear(pin);
    }
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief Initialise the accelerometer
 */
void io_accelerometer_initialise(void)
{
    while(i2c1_transfer_in_progress() == true) {}

    /*
     * first data byte is the address of the first control register. MSB of this
     * address enables increment mode so each successive data byte will be written
     * to the next control register along ( 6 in total )
     */
    i2c1_tx_buffer_write(ACCEL_INC(ACCEL_CTRL_REG_1_ADDR));

    i2c1_tx_buffer_write(ACCEL_CTRL_REG_1_VAL);
    i2c1_tx_buffer_write(ACCEL_CTRL_REG_2_VAL);
    i2c1_tx_buffer_write(ACCEL_CTRL_REG_3_VAL);
    i2c1_tx_buffer_write(ACCEL_CTRL_REG_4_VAL);
    i2c1_tx_buffer_write(ACCEL_CTRL_REG_5_VAL);
    i2c1_tx_buffer_write(ACCEL_CTRL_REG_6_VAL);

    i2c1_tx_data(I2C_ADDRESS_ACCEL, true);
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief       Return if the accelerometer has new data.
 * @return bool True or false.
 */
bool io_accelerometer_data_ready(void)
{
    return (accel_data_ready);
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief         Request and read the data from the accelerometer.
 * @param accel_x Acceleration in the x axis.
 * @param accel_y Acceleration in the y axis.
 * @param accel_z Acceleration in the z axis.
 */
void io_accelerometer_read(int32_t *accel_x, int32_t *accel_y, int32_t *accel_z)
{
    int16_t data_x = 0;
    int16_t data_y = 0;
    int16_t data_z = 0;

    accel_data_ready = false;
    while(i2c1_transfer_in_progress() == true) {}

    /*
     * Request transmission of all 6 data buffers.
     */
    i2c1_tx_buffer_write(ACCEL_INC(ACCEL_OUT_REG_X_L_ADDR));
    i2c1_tx_data(I2C_ADDRESS_ACCEL, false);
    while(i2c1_transfer_in_progress() == true) {}

    /*
     * Receive the data, then read it from the buffer. The data is left-justified, 10bit and spread
     * over 2 8bit registers. The MSb must be preserved as we merge the 2 8bit values since the
     * data is 2's compliment.
     */
    i2c1_rx_data(I2C_ADDRESS_ACCEL, 6);
    while(i2c1_transfer_in_progress() == true) {}

    data_x = i2c1_rx_buffer_read();
    data_x += (i2c1_rx_buffer_read() << 8);
    data_x = (data_x >> (16 - ACCEL_RESOLUTION));

    data_y = i2c1_rx_buffer_read();
    data_y += (i2c1_rx_buffer_read() << 8);
    data_y = (data_y >> (16 - ACCEL_RESOLUTION));

    data_z = i2c1_rx_buffer_read();
    data_z += (i2c1_rx_buffer_read() << 8);
    data_z = (data_z >> (16 - ACCEL_RESOLUTION));

    *accel_x = data_x;
    *accel_y = data_y;
    *accel_z = data_z;
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief Initialise the Magnetometer.
 */
void io_magnetometer_initialise(void)
{
    while(i2c1_transfer_in_progress() == true) {}

    /*
     * first data byte is the address of the first control register. MSB of this
     * address enables increment mode so each successive data byte will be written
     * to the next control register along ( 3 in total )
     */
    i2c1_tx_buffer_write(ACCEL_INC(MAG_CONFIG_REG_A_ADDR));

    i2c1_tx_buffer_write(MAG_CONFIG_REG_A_VAL);
    i2c1_tx_buffer_write(MAG_CONFIG_REG_B_VAL);
    i2c1_tx_buffer_write(MAG_CONFIG_REG_C_VAL);
    i2c1_tx_buffer_write(MAG_INT_REG_VAL);

    i2c1_tx_data(I2C_ADDRESS_MAG, true);
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief       Return if the magnetometer has new data.
 * @return bool True or false.
 */
bool io_magnetometer_data_ready(void)
{
    return (mag_data_ready);
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief       Request and read data from the magnetometer.
 * @param mag_x Magnetic attraction in the x axis.
 * @param mag_y Magnetic attraction in the y axis.
 * @param mag_z Magnetic attraction in the z axis.
 */
void io_magnetometer_read(int32_t *mag_x, int32_t *mag_y, int32_t *mag_z)
{
    int16_t data_x = 0;
    int16_t data_y = 0;
    int16_t data_z = 0;

    mag_data_ready = false;
    while(i2c1_transfer_in_progress() == true) {}

    /*
     * Incrementally read from all 6 output registers
     */
    i2c1_tx_buffer_write(ACCEL_INC(MAG_OUT_REG_X_L_ADDR));
    i2c1_tx_data(I2C_ADDRESS_MAG, false);
    while(i2c1_transfer_in_progress() == true) {}

    /*
     * read the data. data is 16bit and spread over 2 8bit registers
     * preserve the MSb since the data's 2's compliment
     */
    i2c1_rx_data(I2C_ADDRESS_MAG, 6);
    while(i2c1_transfer_in_progress() == true) {}

    data_x = i2c1_rx_buffer_read();
    data_x += (i2c1_rx_buffer_read() << 8);

    data_y = i2c1_rx_buffer_read();
    data_y += (i2c1_rx_buffer_read() << 8);

    data_z = i2c1_rx_buffer_read();
    data_z += (i2c1_rx_buffer_read() << 8);

    *mag_x = data_x;
    *mag_y = data_y;
    *mag_z = data_z;
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief Initialise the gyroscope.
 */
void io_gyroscope_initialise(void)
{
    while(spi1_transfer_in_progress() == true) {}

    uint8_t address;
    address = GYRO_CTRL_REG_1_ADDR;
    address = GYRO_SET_INCREMENT_BIT(address);
    address = GYRO_SET_WRITE_BIT(address);

    spi1_tx_buffer_write(address);
    spi1_tx_buffer_write(GYRO_CTRL_REG_1_VAL);
    spi1_tx_buffer_write(GYRO_CTRL_REG_2_VAL);
    spi1_tx_buffer_write(GYRO_CTRL_REG_3_VAL);
    spi1_tx_buffer_write(GYRO_CTRL_REG_4_VAL);
    spi1_tx_buffer_write(GYRO_CTRL_REG_5_VAL);

    gyro_slave_select_toggle(ON);
    spi1_transfer_data(0);
    while(spi1_transfer_in_progress() == true) {}
    gyro_slave_select_toggle(OFF);
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief       Return if the gyroscope has new data.
 * @return bool True or false.
 */
bool io_gyroscope_data_ready(void)
{
    return (gyro_data_ready);
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief        Request and read data from the gyroscope.
 * @param gyro_x Rotation attraction in the x axis.
 * @param gyro_y Rotation attraction in the y axis.
 * @param gyro_z Rotation attraction in the z axis.
 */
void io_gyroscope_read(int32_t *gyro_x, int32_t *gyro_y, int32_t *gyro_z)
{
    int16_t data_x = 0;
    int16_t data_y = 0;
    int16_t data_z = 0;

    gyro_data_ready = false;
    while(spi1_transfer_in_progress() == true) {}

    /*
     * Read from all 6 output registers
     */
    uint8_t address;
    address = GYRO_OUT_REG_X_L_ADDR;
    address = GYRO_SET_INCREMENT_BIT(address);
    address = GYRO_SET_READ_BIT(address);
    spi1_tx_buffer_write(address);

    gyro_slave_select_toggle(ON);
    spi1_transfer_data(6);
    while(spi1_transfer_in_progress() == true) {}
    gyro_slave_select_toggle(OFF);

    /*
     * read the data. data is 16bit and spread over 2 8bit registers
     * preserve the MSb since the data's 2's compliment. first read
     * removes the 0 from the address transmision
     */
    spi1_rx_buffer_read();
    data_x = spi1_rx_buffer_read();
    data_x += (spi1_rx_buffer_read() << 8);

    data_y = spi1_rx_buffer_read();
    data_y += (spi1_rx_buffer_read() << 8);

    data_z = spi1_rx_buffer_read();
    data_z += (spi1_rx_buffer_read() << 8);

    *gyro_x = data_x;
    *gyro_y = data_y;
    *gyro_z = data_z;
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief Poll for the io module. Called by the systick interupt.
 */
void io_1ms_poll(void)
{
    if(led_timer >= 100)
    {
        led_timer = 0;

        port_clear(led_active);
        led_active++;
        if(led_active > LED_W)
        {
            led_active = LED_NW;
        }
        port_set(led_active);
    }
    else
    {
        led_timer++;
    }
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief ISR for the gyroscope data ready interrupt
 */
void external_interupt_1_isr(void)
{
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_1);
    gyro_data_ready = true;
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief ISR for the magnetometer data ready interrupt
 */
void external_interupt_2_isr(void)
{
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_2);
    mag_data_ready = true;
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief ISR for the accelerometer data ready interrupt
 */
void external_interupt_4_isr(void)
{
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_4);
    accel_data_ready = true;
}

/*------------------------------------------------------------------------------------------------*/
/*-static-functions-------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/**
 * @brief      Toggle the gyroscope slave select on or off.
 * @param mode ON or OFF.
 */
void gyro_slave_select_toggle(toggle_t mode)
{
    if(mode == ON)
    {
        port_clear(GYRO_CS);
    }
    else
    {
        port_set(GYRO_CS);
    }
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief Initialise any used GPIO pins.
 */
void initialise_pins(void)
{
    /*
     * GPIO
     */
    port_initialise_pin(LED_N, PORT_MODE_PUSH_PULL, 0);
    port_initialise_pin(LED_NE, PORT_MODE_PUSH_PULL, 0);
    port_initialise_pin(LED_E, PORT_MODE_PUSH_PULL, 0);
    port_initialise_pin(LED_SE, PORT_MODE_PUSH_PULL, 0);
    port_initialise_pin(LED_S, PORT_MODE_PUSH_PULL, 0);
    port_initialise_pin(LED_SW, PORT_MODE_PUSH_PULL, 0);
    port_initialise_pin(LED_W, PORT_MODE_PUSH_PULL, 0);
    port_initialise_pin(LED_NW, PORT_MODE_PUSH_PULL, 0);

    /*
     * Debug
     */
    port_initialise_pin(DEBUG_TX, PORT_MODE_ALT_OUTPUT, 7);
    port_initialise_pin(DEBUG_RX, PORT_MODE_INPUT_PULLUP, 7);

    /*
     * Accelerometer / Magnetometer
     */
    port_initialise_pin(ACCEL_CLOCK, PORT_MODE_ALT_OPEN_DRAIN, 4);
    port_initialise_pin(ACCEL_DATA, PORT_MODE_ALT_OPEN_DRAIN, 4);
    port_initialise_pin(ACCEL_DRDY, PORT_MODE_INPUT_PULLDOWN, 0);
    port_initialise_pin(ACCEL_INT1, PORT_MODE_INPUT_PULLDOWN, 0);
    port_initialise_pin(ACCEL_INT2, PORT_MODE_INPUT_PULLDOWN, 0);

    /*
     * Gyroscope
     */
    port_set(GYRO_CS);
    port_initialise_pin(GYRO_CLOCK, PORT_MODE_ALT_OUTPUT, 5);
    port_initialise_pin(GYRO_MISO, PORT_MODE_ALT_OUTPUT, 5);
    port_initialise_pin(GYRO_MOSI, PORT_MODE_ALT_OUTPUT, 5);
    port_initialise_pin(GYRO_CS, PORT_MODE_PUSH_PULL, 0);
    port_initialise_pin(GYRO_INT1, PORT_MODE_INPUT_PULLDOWN, 0);
    port_initialise_pin(GYRO_INT2, PORT_MODE_INPUT_PULLDOWN, 0);
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief Initialise any external interupts.
 */
void initialise_external_interupts(void)
{
    /*
     * Accelerometer data ready
     */
    LL_EXTI_InitTypeDef exti_initialisation_structure;
    exti_initialisation_structure.LineCommand = ENABLE;
    exti_initialisation_structure.Line_0_31   = LL_EXTI_LINE_4;
    exti_initialisation_structure.Line_32_63  = LL_EXTI_LINE_NONE;
    exti_initialisation_structure.Mode        = LL_EXTI_MODE_IT;
    exti_initialisation_structure.Trigger     = LL_EXTI_TRIGGER_RISING;
    LL_EXTI_Init(&exti_initialisation_structure);
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTE, LL_SYSCFG_EXTI_LINE4);
    NVIC_SetPriority(EXTI4_IRQn, 3);
    NVIC_EnableIRQ(EXTI4_IRQn);

    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_4);

    /*
     * Magnetometer data ready
     */
    exti_initialisation_structure.LineCommand = ENABLE;
    exti_initialisation_structure.Line_0_31   = LL_EXTI_LINE_2;
    exti_initialisation_structure.Line_32_63  = LL_EXTI_LINE_NONE;
    exti_initialisation_structure.Mode        = LL_EXTI_MODE_IT;
    exti_initialisation_structure.Trigger     = LL_EXTI_TRIGGER_RISING;
    LL_EXTI_Init(&exti_initialisation_structure);
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTE, LL_SYSCFG_EXTI_LINE2);
    NVIC_SetPriority(EXTI2_TSC_IRQn, 3);
    NVIC_EnableIRQ(EXTI2_TSC_IRQn);

    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_2);

    /*
     * Gyroscope data ready
     */
    exti_initialisation_structure.LineCommand = ENABLE;
    exti_initialisation_structure.Line_0_31   = LL_EXTI_LINE_1;
    exti_initialisation_structure.Line_32_63  = LL_EXTI_LINE_NONE;
    exti_initialisation_structure.Mode        = LL_EXTI_MODE_IT;
    exti_initialisation_structure.Trigger     = LL_EXTI_TRIGGER_RISING;
    LL_EXTI_Init(&exti_initialisation_structure);
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTE, LL_SYSCFG_EXTI_LINE1);
    NVIC_SetPriority(EXTI1_IRQn, 3);
    NVIC_EnableIRQ(EXTI1_IRQn);

    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_1);
}

/*------------------------------------------------------------------------------------------------*/
/*-end-of-module----------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/
