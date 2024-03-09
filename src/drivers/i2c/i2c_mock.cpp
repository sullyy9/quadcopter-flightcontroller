////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file    i2c_mock.cpp
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "CppUTestExt/MockSupport.h"

#include "i2c_mock.hpp"

using I2C = i2c::mock::cpputest::I2C;

/*------------------------------------------------------------------------------------------------*/
// mock method definitions
/*------------------------------------------------------------------------------------------------*/

auto I2C::transaction(Address address, std::span<Operation> operations) -> Result<void> {
    ::mock()
        .actualCall("transaction")
        .withParameter("address", address)
        .withParameterOfType("std::span<Operation>", "operations", &operations);

    auto* const return_value = ::mock().returnPointerValueOrDefault({});
    return *static_cast<Result<void>*>(return_value);
}

/*------------------------------------------------------------------------------------------------*/

auto I2C::read(Address address, std::span<std::byte> buffer) -> Result<void> {
    ::mock()
        .actualCall("transaction")
        .withParameter("address", address)
        .withOutputParameterOfType("std::span<std::byte>", "buffer", &buffer);

    auto* const return_value = ::mock().returnPointerValueOrDefault({});
    return *static_cast<Result<void>*>(return_value);
}

/*------------------------------------------------------------------------------------------------*/

auto I2C::write(Address address, std::span<const std::byte> buffer) -> Result<void> {
    ::mock()
        .actualCall("transaction")
        .withParameter("address", address)
        .withParameterOfType("std::span<const std::byte>", "buffer", &buffer);

    auto* const return_value = ::mock().returnPointerValueOrDefault({});
    return *static_cast<Result<void>*>(return_value);
}

/*------------------------------------------------------------------------------------------------*/

auto I2C::write_read(Address address,
                     std::span<const std::byte> write_buffer,
                     std::span<std::byte> read_buffer) -> Result<void> {
    ::mock()
        .actualCall("transaction")
        .withParameter("address", address)
        .withParameterOfType("std::span<const std::byte>", "write_buffer", &write_buffer)
        .withOutputParameterOfType("std::span<std::byte>", "read_buffer", &read_buffer);

    auto* const return_value = ::mock().returnPointerValueOrDefault({});
    return *static_cast<Result<void>*>(return_value);
}

/*------------------------------------------------------------------------------------------------*/
