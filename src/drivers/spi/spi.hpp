////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file  i2c.hpp
/// @brief Common interface for I2C drivers.
////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once

#include <expected>
#include <span>
#include <system_error>

/*------------------------------------------------------------------------------------------------*/
// error handling
/*------------------------------------------------------------------------------------------------*/

namespace spi {

template<typename T>
using Result = std::expected<T, std::error_code>;

}

/*------------------------------------------------------------------------------------------------*/
// driver concepts
/*------------------------------------------------------------------------------------------------*/

namespace spi {

/// @brief Interface for SPI drivers.
template<typename T>
concept SPIDriver = requires(std::span<const std::byte> write, std::span<std::byte> read) {
    { T::transfer(write, read) } -> std::same_as<Result<void>>;
    { T::read(read) } -> std::same_as<Result<void>>;
    { T::write(write) } -> std::same_as<Result<void>>;
};

}

/*------------------------------------------------------------------------------------------------*/
