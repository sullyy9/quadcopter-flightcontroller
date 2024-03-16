////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file  spi_stm32f303.hpp
/// @brief SPI driver implementation for the stm32f303.
////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once

#include "spi.hpp"

/*------------------------------------------------------------------------------------------------*/
// class declarations.
/*------------------------------------------------------------------------------------------------*/

namespace spi::stm32f303 {

struct SPI {
    static auto init() noexcept -> Result<void>;

    static auto transfer(std::span<const std::byte> write, std::span<std::byte> read) noexcept
        -> Result<void>;

    static auto read(std::span<std::byte> buf) noexcept -> Result<void>;

    static auto write(std::span<const std::byte> buf) noexcept -> Result<void>;
};

static_assert(SPIDriver<SPI>);

}

/*------------------------------------------------------------------------------------------------*/
// interupts.
/*------------------------------------------------------------------------------------------------*/

namespace spi::stm32f303 {

auto dma1_channel2_isr() noexcept -> void;
auto dma1_channel3_isr() noexcept -> void;
auto error_isr() noexcept -> void;

}

/*------------------------------------------------------------------------------------------------*/
