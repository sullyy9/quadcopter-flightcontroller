////////////////////////////////////////////////////////////////////////////////////////////////////
/// @author  Ryan Sullivan (ryansullivan@googlemail.com)
/// 
/// @file    usart_stm32f303.hpp
/// @brief   USART driver module for the STM32F303.
////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once

#include <cstdint>
#include <optional>

#include "usart.hpp"

/*------------------------------------------------------------------------------------------------*/
// Class declarations.
/*------------------------------------------------------------------------------------------------*/

namespace usart::stm32f303 {

struct USART {

    [[nodiscard]] static auto init(const USARTConfig& config) noexcept -> std::error_code;

    [[nodiscard]] static auto tx_free() noexcept -> uint32_t;
    static auto tx_byte(std::byte byte) noexcept -> void;
    static auto tx_flush() noexcept -> void;

};

static_assert(USARTDriver<USART>);

}

/*------------------------------------------------------------------------------------------------*/
// Module public functions.
/*------------------------------------------------------------------------------------------------*/

namespace usart::stm32f303 {

auto dma1_channel4_isr() -> void;

}

/*------------------------------------------------------------------------------------------------*/
