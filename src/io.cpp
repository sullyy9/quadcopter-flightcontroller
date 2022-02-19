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

#include "lsm303agr.hpp"
#include "i3g4250d.hpp"

#include "io.hpp"
#include "clocks.hpp"
#include "port.hpp"
#include "usart.hpp"
#include "i2c.hpp"
#include "spi.hpp"

#include "debug.hpp"

using namespace io;
/*------------------------------------------------------------------------------------------------*/
/*-constant-definitions---------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

#define LED_NW port::E8
#define LED_N  port::E9
#define LED_NE port::E10
#define LED_E  port::E11
#define LED_SE port::E12
#define LED_S  port::E13
#define LED_SW port::E14
#define LED_W  port::E15

#define DEBUG_TX port::C4
#define DEBUG_RX port::C5

#define ACCEL_CLOCK port::B6
#define ACCEL_DATA  port::B7
#define ACCEL_DRDY  port::E2
#define ACCEL_INT1  port::E4
#define ACCEL_INT2  port::E5

#define GYRO_CLOCK port::A5
#define GYRO_MISO  port::A6
#define GYRO_MOSI  port::A7
#define GYRO_CS    port::E3
#define GYRO_INT1  port::E0
#define GYRO_INT2  port::E1

/*------------------------------------------------------------------------------------------------*/
/*-exported-variables-----------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------------------*/
/*-static-variables-------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

static volatile uint32_t led_timer  = 0;

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
void io::initialise(void)
{
    clocks::initialise();

    initialise_pins();

    initialise_external_interupts();

    usart::initialise();
    i2c::initialise();
    spi::initialise();
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief Initialise the accelerometer
 */
void io::accelerometer_initialise(void)
{
    while(i2c::transfer_in_progress() == true) {}

    /*
     * first data byte is the address of the first control register. MSB of this
     * address enables increment mode so each successive data byte will be written
     * to the next control register along ( 6 in total )
     */
    i2c::tx_buffer_write(ACCEL_INC(ACCEL_CTRL_REG_1_ADDR));

    i2c::tx_buffer_write(ACCEL_CTRL_REG_1_VAL);
    i2c::tx_buffer_write(ACCEL_CTRL_REG_2_VAL);
    i2c::tx_buffer_write(ACCEL_CTRL_REG_3_VAL);
    i2c::tx_buffer_write(ACCEL_CTRL_REG_4_VAL);
    i2c::tx_buffer_write(ACCEL_CTRL_REG_5_VAL);
    i2c::tx_buffer_write(ACCEL_CTRL_REG_6_VAL);

    i2c::tx_data(i2c::ADDRESS_ACCEL, i2c::TRANSMIT_REQUEST);
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief       Return if the accelerometer has new data.
 * @return bool True or false.
 */
bool io::accelerometer_data_ready(void)
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
void io::accelerometer_read(int32_t *accel_x, int32_t *accel_y, int32_t *accel_z)
{
    int16_t data_x = 0;
    int16_t data_y = 0;
    int16_t data_z = 0;

    accel_data_ready = false;
    while(i2c::transfer_in_progress() == true) {}

    /*
     * Request transmission of all 6 data buffers.
     */
    i2c::tx_buffer_write(ACCEL_INC(ACCEL_OUT_REG_X_L_ADDR));
    i2c::tx_data(i2c::ADDRESS_ACCEL, i2c::TRANSMIT_NORMAL);
    while(i2c::transfer_in_progress() == true) {}

    /*
     * Receive the data, then read it from the buffer. The data is left-justified, 10bit and spread
     * over 2 8bit registers. The MSb must be preserved as we merge the 2 8bit values since the
     * data is 2's compliment.
     */
    i2c::rx_data(i2c::ADDRESS_ACCEL, 6);
    while(i2c::transfer_in_progress() == true) {}

    data_x = i2c::rx_buffer_read();
    data_x += (i2c::rx_buffer_read() << 8);
    data_x = (data_x >> (16 - ACCEL_RESOLUTION));

    data_y = i2c::rx_buffer_read();
    data_y += (i2c::rx_buffer_read() << 8);
    data_y = (data_y >> (16 - ACCEL_RESOLUTION));

    data_z = i2c::rx_buffer_read();
    data_z += (i2c::rx_buffer_read() << 8);
    data_z = (data_z >> (16 - ACCEL_RESOLUTION));

    *accel_x = data_x;
    *accel_y = data_y;
    *accel_z = data_z;
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief Initialise the Magnetometer.
 */
void io::magnetometer_initialise(void)
{
    while(i2c::transfer_in_progress() == true) {}

    /*
     * first data byte is the address of the first control register. MSB of this
     * address enables increment mode so each successive data byte will be written
     * to the next control register along ( 3 in total )
     */
    i2c::tx_buffer_write(ACCEL_INC(MAG_CONFIG_REG_A_ADDR));

    i2c::tx_buffer_write(MAG_CONFIG_REG_A_VAL);
    i2c::tx_buffer_write(MAG_CONFIG_REG_B_VAL);
    i2c::tx_buffer_write(MAG_CONFIG_REG_C_VAL);
    i2c::tx_buffer_write(MAG_INT_REG_VAL);

    i2c::tx_data(i2c::ADDRESS_MAG, i2c::TRANSMIT_REQUEST);
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief       Return if the magnetometer has new data.
 * @return bool True or false.
 */
bool io::magnetometer_data_ready(void)
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
void io::magnetometer_read(int32_t *mag_x, int32_t *mag_y, int32_t *mag_z)
{
    int16_t data_x = 0;
    int16_t data_y = 0;
    int16_t data_z = 0;

    mag_data_ready = false;
    while(i2c::transfer_in_progress() == true) {}

    /*
     * Incrementally read from all 6 output registers
     */
    i2c::tx_buffer_write(ACCEL_INC(MAG_OUT_REG_X_L_ADDR));
    i2c::tx_data(i2c::ADDRESS_MAG, i2c::TRANSMIT_NORMAL);
    while(i2c::transfer_in_progress() == true) {}

    /*
     * read the data. data is 16bit and spread over 2 8bit registers
     * preserve the MSb since the data's 2's compliment
     */
    i2c::rx_data(i2c::ADDRESS_MAG, 6);
    while(i2c::transfer_in_progress() == true) {}

    data_x = i2c::rx_buffer_read();
    data_x += (i2c::rx_buffer_read() << 8);

    data_y = i2c::rx_buffer_read();
    data_y += (i2c::rx_buffer_read() << 8);

    data_z = i2c::rx_buffer_read();
    data_z += (i2c::rx_buffer_read() << 8);

    *mag_x = data_x;
    *mag_y = data_y;
    *mag_z = data_z;
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief Initialise the gyroscope.
 */
void io::gyroscope_initialise(void)
{
    while(spi::transfer_in_progress() == true) {}

    uint8_t address;
    address = GYRO_CTRL_REG_1_ADDR;
    address = GYRO_SET_INCREMENT_BIT(address);
    address = GYRO_SET_WRITE_BIT(address);

    spi::tx_buffer_write(address);
    spi::tx_buffer_write(GYRO_CTRL_REG_1_VAL);
    spi::tx_buffer_write(GYRO_CTRL_REG_2_VAL);
    spi::tx_buffer_write(GYRO_CTRL_REG_3_VAL);
    spi::tx_buffer_write(GYRO_CTRL_REG_4_VAL);
    spi::tx_buffer_write(GYRO_CTRL_REG_5_VAL);

    gyro_slave_select_toggle(ON);
    spi::transfer_data(0);
    while(spi::transfer_in_progress() == true) {}
    gyro_slave_select_toggle(OFF);
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief       Return if the gyroscope has new data.
 * @return bool True or false.
 */
bool io::gyroscope_data_ready(void)
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
void io::gyroscope_read(int32_t *gyro_x, int32_t *gyro_y, int32_t *gyro_z)
{
    int16_t data_x = 0;
    int16_t data_y = 0;
    int16_t data_z = 0;

    gyro_data_ready = false;
    while(spi::transfer_in_progress() == true) {}

    /*
     * Read from all 6 output registers
     */
    uint8_t address;
    address = GYRO_OUT_REG_X_L_ADDR;
    address = GYRO_SET_INCREMENT_BIT(address);
    address = GYRO_SET_READ_BIT(address);
    spi::tx_buffer_write(address);

    gyro_slave_select_toggle(ON);
    spi::transfer_data(6);
    while(spi::transfer_in_progress() == true) {}
    gyro_slave_select_toggle(OFF);

    /*
     * read the data. data is 16bit and spread over 2 8bit registers
     * preserve the MSb since the data's 2's compliment. first read
     * removes the 0 from the address transmision
     */
    spi::rx_buffer_read();
    data_x = spi::rx_buffer_read();
    data_x += (spi::rx_buffer_read() << 8);

    data_y = spi::rx_buffer_read();
    data_y += (spi::rx_buffer_read() << 8);

    data_z = spi::rx_buffer_read();
    data_z += (spi::rx_buffer_read() << 8);

    *gyro_x = data_x;
    *gyro_y = data_y;
    *gyro_z = data_z;
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief Poll for the io module. Called by the systick interupt.
 */
void io::poll(void)
{
    const static port::pin_t led[] = {LED_NW, LED_N, LED_NE, LED_E, LED_SE, LED_S, LED_SW, LED_W};
    static uint8_t active = 0;

    if(led_timer >= 100)
    {
        led_timer = 0;

        port::clear(led[active]);
        
        if(led[active] == LED_W)
        {
            active = 0;
        }
        else
        {
            active++;
        }
        port::set(led[active]);
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
void io::external_interupt_1_isr(void)
{
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_1);
    gyro_data_ready = true;
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief ISR for the magnetometer data ready interrupt
 */
void io::external_interupt_2_isr(void)
{
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_2);
    mag_data_ready = true;
}

/*------------------------------------------------------------------------------------------------*/

/**
 * @brief ISR for the accelerometer data ready interrupt
 */
void io::external_interupt_4_isr(void)
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
        port::clear(GYRO_CS);
    }
    else
    {
        port::set(GYRO_CS);
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
    port::initialise_pin(LED_N, port::MODE_PUSH_PULL, 0);
    port::initialise_pin(LED_NE, port::MODE_PUSH_PULL, 0);
    port::initialise_pin(LED_E, port::MODE_PUSH_PULL, 0);
    port::initialise_pin(LED_SE, port::MODE_PUSH_PULL, 0);
    port::initialise_pin(LED_S, port::MODE_PUSH_PULL, 0);
    port::initialise_pin(LED_SW, port::MODE_PUSH_PULL, 0);
    port::initialise_pin(LED_W, port::MODE_PUSH_PULL, 0);
    port::initialise_pin(LED_NW, port::MODE_PUSH_PULL, 0);

    /*
     * Debug
     */
    port::initialise_pin(DEBUG_TX, port::MODE_ALT_OUTPUT, 7);
    port::initialise_pin(DEBUG_RX, port::MODE_INPUT_PULLUP, 7);

    /*
     * Accelerometer / Magnetometer
     */
    port::initialise_pin(ACCEL_CLOCK, port::MODE_ALT_OPEN_DRAIN, 4);
    port::initialise_pin(ACCEL_DATA, port::MODE_ALT_OPEN_DRAIN, 4);
    port::initialise_pin(ACCEL_DRDY, port::MODE_INPUT_PULLDOWN, 0);
    port::initialise_pin(ACCEL_INT1, port::MODE_INPUT_PULLDOWN, 0);
    port::initialise_pin(ACCEL_INT2, port::MODE_INPUT_PULLDOWN, 0);

    /*
     * Gyroscope
     */
    port::set(GYRO_CS);
    port::initialise_pin(GYRO_CLOCK, port::MODE_ALT_OUTPUT, 5);
    port::initialise_pin(GYRO_MISO, port::MODE_ALT_OUTPUT, 5);
    port::initialise_pin(GYRO_MOSI, port::MODE_ALT_OUTPUT, 5);
    port::initialise_pin(GYRO_CS, port::MODE_PUSH_PULL, 0);
    port::initialise_pin(GYRO_INT1, port::MODE_INPUT_PULLDOWN, 0);
    port::initialise_pin(GYRO_INT2, port::MODE_INPUT_PULLDOWN, 0);
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
