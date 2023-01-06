////////////////////////////////////////////////////////////////////////////////////////////////////
/// @author  Ryan Sullivan (ryansullivan@googlemail.com)
///
/// @file    io.hpp
/// @brief   Module for controlling GPIO and external devices.
////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once

#include <cstdint>

/*------------------------------------------------------------------------------------------------*/
// Module public functions.
/*------------------------------------------------------------------------------------------------*/

namespace io {

auto initialise(void) -> void;

auto accelerometer_initialise(void) -> void;
auto accelerometer_data_ready(void) -> bool;
auto accelerometer_read(int32_t *accel_x, int32_t *accel_y, int32_t *accel_z) -> void;

auto magnetometer_initialise(void) -> void;
auto magnetometer_data_ready(void) -> bool;
auto magnetometer_read(int32_t *mag_x, int32_t *mag_y, int32_t *mag_z) -> void;

auto gyroscope_initialise(void) -> void;
auto gyroscope_data_ready(void) -> bool;
auto gyroscope_read(int32_t *gyro_x, int32_t *gyro_y, int32_t *gyro_z) -> void;

auto poll(void) -> void;
auto external_interupt_1_isr(void) -> void;
auto external_interupt_2_isr(void) -> void;
auto external_interupt_4_isr(void) -> void;

}

/*------------------------------------------------------------------------------------------------*/
