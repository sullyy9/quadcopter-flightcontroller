////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file    i2c_mock.hpp
/// @brief   I2C driver module mock for CppUTest.
////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once

#include "i2c.hpp"

/*------------------------------------------------------------------------------------------------*/
// types
/*------------------------------------------------------------------------------------------------*/

namespace i2c::mock::cpputest {

struct I2C {
    static auto transaction(Address address, std::span<Operation> operations) -> Result<void>;

    static auto read(Address address, std::span<std::byte> buffer) -> Result<void>;

    static auto write(Address address, std::span<const std::byte> buffer) -> Result<void>;

    static auto write_read(Address address,
                           std::span<const std::byte> write_buffer,
                           std::span<std::byte> read_buffer) -> Result<void>;
};

static_assert(I2CDriver<I2C>);

}

/*------------------------------------------------------------------------------------------------*/
