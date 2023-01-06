////////////////////////////////////////////////////////////////////////////////////////////////////
/// @author  Ryan Sullivan (ryansullivan@googlemail.com)
/// 
/// @file    usart.hpp
/// @brief   Module for controlling the USART peripheral
////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once

#include <span>
#include <tuple>
#include <array>
#include <cstdint>
#include <cstddef>
#include <optional>
#include <system_error>

/*------------------------------------------------------------------------------------------------*/
// Error handling.
/*------------------------------------------------------------------------------------------------*/
namespace usart {

enum class StatusCode: uint32_t {
    Ok,
    Unimplemented,
    InvalidBaudRate,
    InvalidDataBits,
    InvalidStopBits,
    InvalidTxRxConf,
};

std::error_code make_error_code(StatusCode);

}

namespace std {

template <>
struct is_error_code_enum<usart::StatusCode> : true_type {};

}

template<typename T>
concept SerialComms = requires(T serial, std::byte byte) {
    {serial.tx_free()} -> std::unsigned_integral;
    {serial.tx_byte(byte)};
    {serial.tx_flush()};
};

/*------------------------------------------------------------------------------------------------*/
// Class declarations.
/*------------------------------------------------------------------------------------------------*/

namespace usart {

enum class Parity {
    None,
    Even,
    Odd,
};

enum class FlowControl {
    None,
    RTS,
    CTS,
    RTS_CTS,
};

struct USARTConfig {
    uint32_t    baud_rate    {9600};
    uint32_t    data_bits    {8};
    uint32_t    stop_bits    {1};
    Parity      parity       {Parity::None};
    FlowControl flow_control {FlowControl::None};
    bool        enable_tx    {true};
    bool        enable_rx    {true};
    bool        enable_dma   {true};
};

/// @brief Handler to a USART peripheral.
///
struct USART {
    USART(USART&) = delete;
    USART(USART&&) = default;
    
    ~USART() = default;

    auto operator=(const USART&) -> USART& = delete;
    auto operator=(USART&&) -> USART& = default;

    static auto init(const USARTConfig& config) -> std::tuple<std::optional<USART>, std::error_code>;

    auto tx_free(void) -> uint32_t;
    auto tx_byte(const std::byte byte) -> void;
    auto tx_flush() -> void;

private:
    explicit USART() = default;
};

static_assert(SerialComms<USART>);

}

/*------------------------------------------------------------------------------------------------*/
// Module public functions.
/*------------------------------------------------------------------------------------------------*/

namespace usart {

auto dma1_channel4_isr() -> void;

}

/*------------------------------------------------------------------------------------------------*/
