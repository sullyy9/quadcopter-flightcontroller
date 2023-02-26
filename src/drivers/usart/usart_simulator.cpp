////////////////////////////////////////////////////////////////////////////////////////////////////
/// @author  Ryan Sullivan (ryansullivan@googlemail.com)
///
/// @file    usart_simulator.cpp
/// @brief   USART driver module for simulation.
////////////////////////////////////////////////////////////////////////////////////////////////////

#include <array>
#include <vector>
#include <utility>

#include "usart.hpp"
#include "usart_simulator.hpp"

/*------------------------------------------------------------------------------------------------*/
// Module private objects.
/*------------------------------------------------------------------------------------------------*/

// Tx buffer for USART.
static constinit std::vector<std::byte> transmitted_data {};
static constinit std::vector<std::byte> tx_buffer {};

static constexpr size_t TX_BUFFER_SIZE_DEFAULT = 128;
static constinit size_t tx_buffer_size {TX_BUFFER_SIZE_DEFAULT};

/*------------------------------------------------------------------------------------------------*/
// Class method definitions.
/*------------------------------------------------------------------------------------------------*/

auto usart::sim::USART::init([[maybe_unused]] const USARTConfig& config) -> std::error_code {
    return StatusCode::Ok;
}

/*------------------------------------------------------------------------------------------------*/

auto usart::sim::USART::tx_free() -> uint32_t {
    return tx_buffer.size() - tx_buffer_size;
}

/*------------------------------------------------------------------------------------------------*/

auto usart::sim::USART::tx_byte(const std::byte byte) -> void {
    tx_buffer.emplace_back(byte);
}

/*------------------------------------------------------------------------------------------------*/

auto usart::sim::USART::tx_flush() -> void {
    transmitted_data.insert(transmitted_data.cend(), tx_buffer.cbegin(), tx_buffer.cend());
    tx_buffer.clear();
}

/*------------------------------------------------------------------------------------------------*/
// Simulator controls.
/*------------------------------------------------------------------------------------------------*/

static auto usart::sim::ctrl::reset() -> void {
    transmitted_data.clear();
    tx_buffer.clear();
    tx_buffer_size = TX_BUFFER_SIZE_DEFAULT;
}

/*------------------------------------------------------------------------------------------------*/

static auto usart::sim::ctrl::set_tx_buffer_size(size_t number_bytes) -> void {
    tx_buffer_size = number_bytes;
}

/*------------------------------------------------------------------------------------------------*/

static auto usart::sim::ctrl::get_transmitted_data() -> std::span<std::byte> {
    return std::span{transmitted_data};
}

/*------------------------------------------------------------------------------------------------*/

static auto usart::sim::ctrl::take_transmitted_data() -> std::vector<std::byte> {
    return std::exchange(transmitted_data, decltype(transmitted_data){});
}

/*------------------------------------------------------------------------------------------------*/

static auto usart::sim::ctrl::clear_transmitted_data() -> void {
    transmitted_data.clear();
}

/*------------------------------------------------------------------------------------------------*/
