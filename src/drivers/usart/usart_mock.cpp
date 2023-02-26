////////////////////////////////////////////////////////////////////////////////////////////////////
/// @author  Ryan Sullivan (ryansullivan@googlemail.com)
///
/// @file    usart_mock.cpp
/// @brief   USART driver mock for CppUTest.
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "CppUTestExt/MockSupport.h"

#include "usart.hpp"
#include "usart_mock.hpp"

/*------------------------------------------------------------------------------------------------*/
// USART method definitions.
/*------------------------------------------------------------------------------------------------*/

auto usart::mock::cpputest::USART::init([[maybe_unused]] usart::USARTConfig config) -> std::error_code {
    return usart::StatusCode::Ok;
}

/*------------------------------------------------------------------------------------------------*/

auto usart::mock::cpputest::USART::tx_free() -> uint32_t {
    ::mock().actualCall("tx_free");
    return 0;
}

/*------------------------------------------------------------------------------------------------*/

auto usart::mock::cpputest::USART::tx_byte(const std::byte byte) -> void {
    ::mock().actualCall("tx_byte").withParameter("byte", static_cast<int>(byte));
}

/*------------------------------------------------------------------------------------------------*/

auto usart::mock::cpputest::USART::tx_flush() -> void {
    ::mock().actualCall("tx_flush");
}

/*------------------------------------------------------------------------------------------------*/
