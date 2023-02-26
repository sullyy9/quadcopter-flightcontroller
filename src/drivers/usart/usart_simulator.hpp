////////////////////////////////////////////////////////////////////////////////////////////////////
/// @author  Ryan Sullivan (ryansullivan@googlemail.com)
///
/// @file    usart_simulator.hpp
/// @brief   USART driver module for simulation.
////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once

#include <optional>
#include <vector>
#include <span>

#include "usart.hpp"

/*------------------------------------------------------------------------------------------------*/
// Class declarations.
/*------------------------------------------------------------------------------------------------*/

namespace usart::sim {

struct USART {

    [[nodiscard]] static auto init(const USARTConfig& config) -> std::error_code;

    [[nodiscard]] static auto tx_free() -> uint32_t;
    static auto tx_byte(std::byte byte) -> void;
    static auto tx_flush() -> void;

};

static_assert(USARTDriver<USART>);

}

/*------------------------------------------------------------------------------------------------*/
// Simulator controls.
/*------------------------------------------------------------------------------------------------*/

namespace usart::sim::ctrl {

[[maybe_unused]] static auto reset() -> void;

[[maybe_unused]] static auto set_tx_buffer_size(size_t number_bytes) -> void;

[[maybe_unused]] static auto get_transmitted_data() -> std::span<std::byte>;
[[maybe_unused]] static auto take_transmitted_data() -> std::vector<std::byte>;
[[maybe_unused]] static auto clear_transmitted_data() -> void;

}

/*------------------------------------------------------------------------------------------------*/
// Module public functions.
/*------------------------------------------------------------------------------------------------*/

namespace usart {

auto dma1_channel4_isr() -> void;

}

/*------------------------------------------------------------------------------------------------*/
