////////////////////////////////////////////////////////////////////////////////////////////////////
/// @author  Ryan Sullivan (ryansullivan@googlemail.com)
///
/// @file    gpio.cpp
/// @brief   GPIO driver module.
////////////////////////////////////////////////////////////////////////////////////////////////////

#include <cstdint>
#include <system_error>

#include "gpio.hpp"

/*------------------------------------------------------------------------------------------------*/
// Error handling.
/*------------------------------------------------------------------------------------------------*/

struct GPIOStatusCategory : std::error_category {
    [[nodiscard]] auto name() const noexcept -> const char* override;
    [[nodiscard]] auto message(int status) const noexcept -> std::string override;
};
const GPIOStatusCategory GPIO_STATUS_CATEGORY {};

/*------------------------------------------------------------------------------------------------*/
 
auto GPIOStatusCategory::name() const noexcept -> const char* {
    return "GPIO";
}

/*------------------------------------------------------------------------------------------------*/

auto GPIOStatusCategory::message(const int status) const noexcept -> std::string {
    using enum gpio::StatusCode;
    switch (static_cast<gpio::StatusCode>(status)) {
        case Ok:                    return "Ok";
        case Unimplemented:         return "Use of unimplemented feature";
        case UnderlyingDriverError: return "Underlying driver error";
        case InvalidAltFunction:    return "Invalid alt function";
        default:                    return "Unknown";
    }
}

/*------------------------------------------------------------------------------------------------*/

auto gpio::make_error_code(const StatusCode status) -> std::error_code {
    return {static_cast<int>(status), GPIO_STATUS_CATEGORY};
}

/*------------------------------------------------------------------------------------------------*/
