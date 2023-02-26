////////////////////////////////////////////////////////////////////////////////////////////////////
/// @author  Ryan Sullivan (ryansullivan@googlemail.com)
/// 
/// @file    usart.cpp
/// @brief   USART driver module.
////////////////////////////////////////////////////////////////////////////////////////////////////

#include <system_error>

#include "usart.hpp"

/*------------------------------------------------------------------------------------------------*/
// Error handling.
/*------------------------------------------------------------------------------------------------*/

struct UsartStatusCategory : std::error_category {
    [[nodiscard]] auto name() const noexcept -> const char* override;
    [[nodiscard]] auto message(int status) const -> std::string override;
};
const UsartStatusCategory USART_STATUS_CATEGORY {};

/*------------------------------------------------------------------------------------------------*/
 
auto UsartStatusCategory::name() const noexcept -> const char* {
    return "USART";
}

/*------------------------------------------------------------------------------------------------*/

auto UsartStatusCategory::message(const int status) const -> std::string {
    using enum usart::StatusCode;
    switch (static_cast<usart::StatusCode>(status)) {
        case Ok:              return "Ok";
        case Unimplemented:   return "Use of unimplemented feature";
        case InvalidBaudRate: return "Invalid baud rate";
        case InvalidDataBits: return "Invalid data bits";
        case InvalidStopBits: return "Invalid stop bits";
        case InvalidTxRxConf: return "Invalid Tx/Rx configuration";
        default:              return "Unknown";
    }
}

/*------------------------------------------------------------------------------------------------*/

auto usart::make_error_code(const usart::StatusCode status) -> std::error_code {
    return {static_cast<int>(status), USART_STATUS_CATEGORY};
}

/*------------------------------------------------------------------------------------------------*/
