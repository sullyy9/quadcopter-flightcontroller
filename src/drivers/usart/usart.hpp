////////////////////////////////////////////////////////////////////////////////////////////////////
/// @author  Ryan Sullivan (ryansullivan@googlemail.com)
/// 
/// @file    usart.hpp
/// @brief   USART driver module.
////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once

#include <cstdint>
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

auto make_error_code(StatusCode) -> std::error_code;

}

namespace std {

template <>
struct is_error_code_enum<usart::StatusCode> : true_type {};

}

/*------------------------------------------------------------------------------------------------*/
// Driver configuration.
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

}

/*------------------------------------------------------------------------------------------------*/
// Driver Concepts.
/*------------------------------------------------------------------------------------------------*/

namespace usart {

template<typename T>
concept USARTDriver = requires(const USARTConfig& config, std::byte byte) {
    {T::init(config)} -> std::same_as<std::error_code>;

    {T::tx_free()}     -> std::unsigned_integral;
    {T::tx_byte(byte)} -> std::same_as<void>;
    {T::tx_flush()}    -> std::same_as<void>;
};

}

/*------------------------------------------------------------------------------------------------*/
