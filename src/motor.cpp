////////////////////////////////////////////////////////////////////////////////////////////////////
/// @author  Ryan Sullivan (ryansullivan@googlemail.com)
///
/// @file    motor.cpp
/// @brief   Module for controlling a motor via a Hobbywing X-rotor ECS.
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "motor.hpp"

/*------------------------------------------------------------------------------------------------*/
// Error handling.
/*------------------------------------------------------------------------------------------------*/

struct MotorStatusCategory : std::error_category {
    [[nodiscard]] auto name() const noexcept -> const char* override;
    [[nodiscard]] auto message(int status) const -> std::string override;
};
const MotorStatusCategory MOTOR_STATUS_CATEGORY {};

/*------------------------------------------------------------------------------------------------*/
 
auto MotorStatusCategory::name() const noexcept -> const char* {
    return "Motor";
}

/*------------------------------------------------------------------------------------------------*/

auto MotorStatusCategory::message(const int status) const -> std::string {
    using enum motor::StatusCode;
    switch (static_cast<motor::StatusCode>(status)) {
        case Ok:                  return "Ok";
        case Unimplemented:       return "Use of unimplemented feature";
        case InvalidPWMFrequency: return "Invalid PWM frequency";
        case InvalidThrottle:     return "Invalid throttle";
        default:                  return "Unknown";
    }
}

/*------------------------------------------------------------------------------------------------*/

auto motor::make_error_code(const StatusCode status) -> std::error_code {
    return {static_cast<int>(status), MOTOR_STATUS_CATEGORY};
}

/*------------------------------------------------------------------------------------------------*/
