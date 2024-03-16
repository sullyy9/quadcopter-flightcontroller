////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file    spi_mock.hpp
/// @brief   SPI driver module mock for CppUTest.
////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once

#include "spi.hpp"

/*------------------------------------------------------------------------------------------------*/
// types
/*------------------------------------------------------------------------------------------------*/

namespace spi::mock::cpputest {

struct SPI {
    static auto transfer(std::span<const std::byte> write, std::span<std::byte> read) noexcept
        -> Result<void>;

    static auto read(std::span<std::byte> buf) noexcept -> Result<void>;

    static auto write(std::span<const std::byte> buf) noexcept -> Result<void>;
};

static_assert(SPIDriver<SPI>);

}

/*------------------------------------------------------------------------------------------------*/
