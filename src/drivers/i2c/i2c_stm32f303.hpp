////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file  i2c_stm32f303.hpp
/// @brief I2C driver implementation for the stm32f303.
////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once

#include "i2c.hpp"

/*------------------------------------------------------------------------------------------------*/
// class declarations.
/*------------------------------------------------------------------------------------------------*/

namespace i2c::stm32f303 {

struct I2C {
    static auto init() noexcept -> Result<void>;

    static auto transaction(Address address, std::span<Operation> operations) noexcept -> Result<void>;

    static auto read(Address address, std::span<std::byte> buffer) noexcept -> Result<void>;

    static auto write(Address address, std::span<const std::byte> buffer) noexcept -> Result<void>;

    static auto write_read(Address address,
                           std::span<const std::byte> write_buffer,
                           std::span<std::byte> read_buffer) noexcept -> Result<void>;
};

static_assert(I2CDriver<I2C>);

}

/*------------------------------------------------------------------------------------------------*/
// interupts.
/*------------------------------------------------------------------------------------------------*/

namespace i2c::stm32f303 {

auto dma1_channel6_isr() noexcept -> void;
auto dma1_channel7_isr() noexcept -> void;
auto ev_isr() noexcept -> void;
auto er_isr() noexcept -> void;

}

/*------------------------------------------------------------------------------------------------*/
