////////////////////////////////////////////////////////////////////////////////////////////////////
/// @author  Ryan Sullivan (ryansullivan@googlemail.com)
///
/// @file    io.cpp
/// @brief   Module for controlling GPIO and external devices.
////////////////////////////////////////////////////////////////////////////////////////////////////

#include <array>
#include <system_error>

#include "stm32f3xx_ll_exti.h"
#include "stm32f3xx_ll_system.h"

#include "gpio.hpp"
#include "gpio_stm32f303.hpp"

#include "io.hpp"
#include "i2c.hpp"
#include "spi.hpp"
#include "lsm303agr.hpp"
#include "i3g4250d.hpp"

using GPIO = gpio::GPIOSTM32F303;

/*------------------------------------------------------------------------------------------------*/
// Constants
/*------------------------------------------------------------------------------------------------*/

static constexpr auto LED_NW {gpio::Pin::E8};
static constexpr auto LED_N  {gpio::Pin::E9};
static constexpr auto LED_NE {gpio::Pin::E10};
static constexpr auto LED_E  {gpio::Pin::E11};
static constexpr auto LED_SE {gpio::Pin::E12};
static constexpr auto LED_S  {gpio::Pin::E13};
static constexpr auto LED_SW {gpio::Pin::E14};
static constexpr auto LED_W  {gpio::Pin::E15};

static constexpr auto  DEBUG_TX {gpio::Pin::C4};
static constexpr auto  DEBUG_RX {gpio::Pin::C5};

static constexpr auto  ACCEL_CLOCK {gpio::Pin::B6};
static constexpr auto  ACCEL_DATA  {gpio::Pin::B7};
static constexpr auto  ACCEL_DRDY  {gpio::Pin::E2};
static constexpr auto  ACCEL_INT1  {gpio::Pin::E4};
static constexpr auto  ACCEL_INT2  {gpio::Pin::E5};

static constexpr auto LED_CLOCK {gpio::Pin::B3}; // SPI3
static constexpr auto LED_DATA  {gpio::Pin::B5}; // SPI3

static constexpr auto  GYRO_CLOCK {gpio::Pin::A5};
static constexpr auto  GYRO_MISO  {gpio::Pin::A6};
static constexpr auto  GYRO_MOSI  {gpio::Pin::A7};
static constexpr auto  GYRO_CS    {gpio::Pin::E3};
static constexpr auto  GYRO_INT1  {gpio::Pin::E0};
static constexpr auto  GYRO_INT2  {gpio::Pin::E1};

static constexpr auto  THROTTLE1  {gpio::Pin::A0};
static constexpr auto  THROTTLE2  {gpio::Pin::A1};
static constexpr auto  THROTTLE3  {gpio::Pin::A2};
static constexpr auto  THROTTLE4  {gpio::Pin::A3};

struct GPIOConfig {
    gpio::Pin pin {};
    gpio::Config config {};
};
constexpr std::array GPIO_INITIAL_CONFIGURATION_TABLE {
    GPIOConfig{LED_N,  gpio::Output{.output_type = gpio::OutputType::PushPull}},
    GPIOConfig{LED_NE, gpio::Output{.output_type = gpio::OutputType::PushPull}},
    GPIOConfig{LED_E , gpio::Output{.output_type = gpio::OutputType::PushPull}},
    GPIOConfig{LED_SE, gpio::Output{.output_type = gpio::OutputType::PushPull}},
    GPIOConfig{LED_S , gpio::Output{.output_type = gpio::OutputType::PushPull}},
    GPIOConfig{LED_SW, gpio::Output{.output_type = gpio::OutputType::PushPull}},
    GPIOConfig{LED_W , gpio::Output{.output_type = gpio::OutputType::PushPull}},
    GPIOConfig{LED_NW, gpio::Output{.output_type = gpio::OutputType::PushPull}},

    GPIOConfig{DEBUG_TX, gpio::AltFunction{.output_type = gpio::OutputType::PushPull,  .pull = gpio::Pull::None, .function = 7}},
    GPIOConfig{DEBUG_RX, gpio::AltFunction{.output_type = gpio::OutputType::OpenDrain, .pull = gpio::Pull::Up,   .function = 7}},

    GPIOConfig{ACCEL_CLOCK, gpio::AltFunction{.output_type = gpio::OutputType::OpenDrain, .function = 4}},
    GPIOConfig{ACCEL_DATA , gpio::AltFunction{.output_type = gpio::OutputType::OpenDrain, .function = 4}},
    GPIOConfig{ACCEL_DRDY, gpio::Input{.pull = gpio::Pull::Down}},
    GPIOConfig{ACCEL_INT1, gpio::Input{.pull = gpio::Pull::Down}},
    GPIOConfig{ACCEL_INT2, gpio::Input{.pull = gpio::Pull::Down}},

    GPIOConfig{GYRO_CLOCK, gpio::AltFunction{.output_type = gpio::OutputType::PushPull, .function = 5}},
    GPIOConfig{GYRO_MISO , gpio::AltFunction{.output_type = gpio::OutputType::PushPull, .function = 5}},
    GPIOConfig{GYRO_MOSI , gpio::AltFunction{.output_type = gpio::OutputType::PushPull, .function = 5}},
    GPIOConfig{GYRO_CS, gpio::Output{.output_type = gpio::OutputType::PushPull}},
    GPIOConfig{GYRO_INT1, gpio::Input{.pull = gpio::Pull::Down}},
    GPIOConfig{GYRO_INT2, gpio::Input{.pull = gpio::Pull::Down}},

    GPIOConfig{THROTTLE1, gpio::AltFunction{.output_type = gpio::OutputType::PushPull, .function = 1}},
    GPIOConfig{THROTTLE2, gpio::AltFunction{.output_type = gpio::OutputType::PushPull, .function = 1}},
    GPIOConfig{THROTTLE3, gpio::AltFunction{.output_type = gpio::OutputType::PushPull, .function = 1}},
    GPIOConfig{THROTTLE4, gpio::AltFunction{.output_type = gpio::OutputType::PushPull, .function = 1}},

    GPIOConfig{LED_CLOCK, gpio::AltFunction{.output_type = gpio::OutputType::PushPull, .function = 6}},
    GPIOConfig{LED_DATA, gpio::AltFunction{.output_type = gpio::OutputType::PushPull, .function = 6}},
};

/*------------------------------------------------------------------------------------------------*/
// Module private variables
/*------------------------------------------------------------------------------------------------*/

static volatile uint32_t led_timer  = 0;

static volatile bool accel_data_ready = true;
static volatile bool mag_data_ready   = true;
static volatile bool gyro_data_ready  = true;

/*------------------------------------------------------------------------------------------------*/
// Forward declarations
/*------------------------------------------------------------------------------------------------*/

auto gyro_slave_select_toggle(const bool mode) -> void;
auto initialise_external_interupts() -> void;

/*------------------------------------------------------------------------------------------------*/
// Module public functions.
/*------------------------------------------------------------------------------------------------*/

/// @brief Initialise any IO.
/// 
auto io::initialise() -> void {
    // Initialise the GPIO pins.
    for(const auto& [pin, config]: GPIO_INITIAL_CONFIGURATION_TABLE) {
        if(GPIO::init(pin, config) != gpio::StatusCode::Ok) {
            // Should return error.
        }
    }

    initialise_external_interupts();

    i2c::initialise();
    spi::initialise();
}

/*------------------------------------------------------------------------------------------------*/

/// @brief Initialise the accelerometer
/// 
auto io::accelerometer_initialise() -> void {
    while(i2c::transfer_in_progress());

    // First data byte is the address of the first control register. MSB of this
    // address enables increment mode so each successive data byte will be written
    // to the next control register along ( 6 in total )
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

/// @brief Return if the accelerometer has new data.
///
/// @return bool True if data is ready, false otherwise.
/// 
auto io::accelerometer_data_ready() -> bool {
    return accel_data_ready;
}

/*------------------------------------------------------------------------------------------------*/

/// @brief         Request and read the data from the accelerometer.
///
/// @param accel_x Acceleration in the x axis.
/// @param accel_y Acceleration in the y axis.
/// @param accel_z Acceleration in the z axis.
/// 
auto io::accelerometer_read(int32_t *accel_x, int32_t *accel_y, int32_t *accel_z) -> void {
    int32_t data_x = 0;
    int32_t data_y = 0;
    int32_t data_z = 0;

    accel_data_ready = false;
    while(i2c::transfer_in_progress());

    // Request transmission of all 6 data buffers.
    i2c::tx_buffer_write(ACCEL_INC(ACCEL_OUT_REG_X_L_ADDR));
    i2c::tx_data(i2c::ADDRESS_ACCEL, i2c::TRANSMIT_NORMAL);
    while(i2c::transfer_in_progress());

    // Receive the data, then read it from the buffer. The data is left-justified, 10bit and spread
    // over 2 8bit registers. The MSb must be preserved as we merge the 2 8bit values since the
    // data is 2's compliment.
    i2c::rx_data(i2c::ADDRESS_ACCEL, 6);
    while(i2c::transfer_in_progress());

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

/// @brief Initialise the Magnetometer.
/// 
auto io::magnetometer_initialise() -> void {
    while(i2c::transfer_in_progress());

    // First data byte is the address of the first control register. MSB of this
    // address enables increment mode so each successive data byte will be written
    // to the next control register along ( 3 in total )
    i2c::tx_buffer_write(ACCEL_INC(MAG_CONFIG_REG_A_ADDR));

    i2c::tx_buffer_write(MAG_CONFIG_REG_A_VAL);
    i2c::tx_buffer_write(MAG_CONFIG_REG_B_VAL);
    i2c::tx_buffer_write(MAG_CONFIG_REG_C_VAL);
    i2c::tx_buffer_write(MAG_INT_REG_VAL);

    i2c::tx_data(i2c::ADDRESS_MAG, i2c::TRANSMIT_REQUEST);
}

/*------------------------------------------------------------------------------------------------*/

/// @brief       Return if the magnetometer has new data.
///
/// @return bool True or false.
/// 
auto io::magnetometer_data_ready() -> bool {
    return (mag_data_ready);
}

/*------------------------------------------------------------------------------------------------*/

/// @brief       Request and read data from the magnetometer.
///
/// @param mag_x Magnetic attraction in the x axis.
/// @param mag_y Magnetic attraction in the y axis.
/// @param mag_z Magnetic attraction in the z axis.
///
auto io::magnetometer_read(int32_t *mag_x, int32_t *mag_y, int32_t *mag_z) -> void {
    int32_t data_x = 0;
    int32_t data_y = 0;
    int32_t data_z = 0;

    mag_data_ready = false;
    while(i2c::transfer_in_progress());

    // Incrementally read from all 6 output registers
    i2c::tx_buffer_write(ACCEL_INC(MAG_OUT_REG_X_L_ADDR));
    i2c::tx_data(i2c::ADDRESS_MAG, i2c::TRANSMIT_NORMAL);
    while(i2c::transfer_in_progress());

    // Read the data. data is 16bit and spread over 2 8bit registers
    // preserve the MSb since the data's 2's compliment
    i2c::rx_data(i2c::ADDRESS_MAG, 6);
    while(i2c::transfer_in_progress());

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

/// @brief Initialise the gyroscope.
/// 
auto io::gyroscope_initialise() -> void {
    while(spi::transfer_in_progress());

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

    gyro_slave_select_toggle(true);
    spi::transfer_data(0);
    while(spi::transfer_in_progress());
    gyro_slave_select_toggle(false);
}

/*------------------------------------------------------------------------------------------------*/

/// @brief       Return if the gyroscope has new data.
///
/// @return bool True or false.
/// 
auto io::gyroscope_data_ready() -> bool {
    return (gyro_data_ready);
}

/*------------------------------------------------------------------------------------------------*/

/// @brief        Request and read data from the gyroscope.
///
/// @param gyro_x Rotation attraction in the x axis.
/// @param gyro_y Rotation attraction in the y axis.
/// @param gyro_z Rotation attraction in the z axis.
/// 
auto io::gyroscope_read(int32_t *gyro_x, int32_t *gyro_y, int32_t *gyro_z) -> void {
    int32_t data_x = 0;
    int32_t data_y = 0;
    int32_t data_z = 0;

    gyro_data_ready = false;
    while(spi::transfer_in_progress());

    
    // Read from all 6 output registers.
    uint8_t address;
    address = GYRO_OUT_REG_X_L_ADDR;
    address = GYRO_SET_INCREMENT_BIT(address);
    address = GYRO_SET_READ_BIT(address);
    spi::tx_buffer_write(address);

    gyro_slave_select_toggle(true);
    spi::transfer_data(6);
    while(spi::transfer_in_progress());
    gyro_slave_select_toggle(false);

    
    // Read the data. data is 16bit and spread over 2 8bit registers
    // preserve the MSb since the data's 2's compliment. first read
    // removes the 0 from the address transmision
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

/// @brief Poll for the io module. Called by the systick interupt.
/// 
auto io::poll() -> void {
    static constexpr std::array LEDS {LED_NW, LED_N, LED_NE, LED_E, LED_SE, LED_S, LED_SW, LED_W};
    static uint8_t active = 0;

    if(led_timer >= 100) {
        led_timer = 0;

        GPIO::reset(LEDS[active]);
        
        if(LEDS[active] == LED_W) {
            active = 0;
        }
        else {
            active++;
        }
        GPIO::set(LEDS[active]);
    }
    else {
        led_timer++;
    }
}

/*------------------------------------------------------------------------------------------------*/

/// @brief ISR for the gyroscope data ready interrupt
/// 
auto io::external_interupt_1_isr() -> void {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_1);
    gyro_data_ready = true;
}

/*------------------------------------------------------------------------------------------------*/

/// @brief ISR for the magnetometer data ready interrupt
/// 
auto io::external_interupt_2_isr() -> void {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_2);
    mag_data_ready = true;
}

/*------------------------------------------------------------------------------------------------*/

/// @brief ISR for the accelerometer data ready interrupt
/// 
auto io::external_interupt_4_isr() -> void {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_4);
    accel_data_ready = true;
}

/*------------------------------------------------------------------------------------------------*/
/*-static-functions-------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

/// @brief      Toggle the gyroscope slave select on or off.
///
/// @param mode ON or OFF.
/// 
auto gyro_slave_select_toggle(const bool mode) -> void {
    if(mode) {
        GPIO::reset(GYRO_CS);
    }
    else {
        GPIO::set(GYRO_CS);
    }
}

/*------------------------------------------------------------------------------------------------*/

/// @brief Initialise any external interupts.
/// 
auto initialise_external_interupts() -> void {

    // Accelerometer data ready.
    LL_EXTI_InitTypeDef accelerometer {
        .Line_0_31   = LL_EXTI_LINE_4,
        .Line_32_63  = LL_EXTI_LINE_NONE,
        .LineCommand = ENABLE,
        .Mode        = LL_EXTI_MODE_IT,
        .Trigger     = LL_EXTI_TRIGGER_RISING,
    };
    LL_EXTI_Init(&accelerometer);
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTE, LL_SYSCFG_EXTI_LINE4);
    NVIC_SetPriority(EXTI4_IRQn, 3);
    NVIC_EnableIRQ(EXTI4_IRQn);
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_4);

    // Magnetometer data ready.
    LL_EXTI_InitTypeDef magnetometer {
        .Line_0_31   = LL_EXTI_LINE_2,
        .Line_32_63  = LL_EXTI_LINE_NONE,
        .LineCommand = ENABLE,
        .Mode        = LL_EXTI_MODE_IT,
        .Trigger     = LL_EXTI_TRIGGER_RISING,
    };
    LL_EXTI_Init(&magnetometer);
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTE, LL_SYSCFG_EXTI_LINE2);
    NVIC_SetPriority(EXTI2_TSC_IRQn, 3);
    NVIC_EnableIRQ(EXTI2_TSC_IRQn);
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_2);


    // Gyroscope data ready.
    LL_EXTI_InitTypeDef gyroscope {
        .Line_0_31   = LL_EXTI_LINE_1,
        .Line_32_63  = LL_EXTI_LINE_NONE,
        .LineCommand = ENABLE,
        .Mode        = LL_EXTI_MODE_IT,
        .Trigger     = LL_EXTI_TRIGGER_RISING,
    };
    LL_EXTI_Init(&gyroscope);
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTE, LL_SYSCFG_EXTI_LINE1);
    NVIC_SetPriority(EXTI1_IRQn, 3);
    NVIC_EnableIRQ(EXTI1_IRQn);
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_1);
}

/*------------------------------------------------------------------------------------------------*/
/*-end-of-module----------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/
