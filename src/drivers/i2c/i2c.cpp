////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file    i2c.cpp
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "i2c.hpp"

/*------------------------------------------------------------------------------------------------*/
// Error handling.
/*------------------------------------------------------------------------------------------------*/

struct I2CStatusCategory: std::error_category {
    [[nodiscard]] auto name() const noexcept -> const char* override;
    [[nodiscard]] auto message(int status) const -> std::string override;
};
const I2CStatusCategory I2C_STATUS_CATEGORY{};

/*------------------------------------------------------------------------------------------------*/

auto I2CStatusCategory::name() const noexcept -> const char* {
    return "I2C";
}

/*------------------------------------------------------------------------------------------------*/

auto I2CStatusCategory::message(const int status) const -> std::string {
    using enum i2c::Error;
    switch (static_cast<i2c::Error>(status)) {
        case Unimplemented: return "Use of unimplemented feature";
        case InsufficientCapacity: return "Insufficient buffer capacity";
        case InsufficientData: return "Insufficient data to fill buffer";
        default: return "Unknown";
    }
}

/*------------------------------------------------------------------------------------------------*/

auto i2c::make_error_code(const i2c::Error status) -> std::error_code {
    return {static_cast<int>(status), I2C_STATUS_CATEGORY};
}

/*------------------------------------------------------------------------------------------------*/
