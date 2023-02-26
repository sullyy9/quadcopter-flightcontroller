////////////////////////////////////////////////////////////////////////////////////////////////////
/// @author  Ryan Sullivan (ryansullivan@googlemail.com)
///
/// @file    usart_mock.hpp
/// @brief   USART driver mock for CppUTest.
////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once

#include "usart.hpp"

/*------------------------------------------------------------------------------------------------*/

namespace usart::mock::cpputest {

struct USART {
    static auto init(usart::USARTConfig config) -> std::error_code;
    
    static auto tx_free() -> uint32_t;
    static auto tx_byte(std::byte byte) -> void;
    static auto tx_flush() -> void;
};

static_assert(usart::USARTDriver<USART>);

}

/*------------------------------------------------------------------------------------------------*/
