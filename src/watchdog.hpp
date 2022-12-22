/**
 * -------------------------------------------------------------------------------------------------
 * @author  Ryan Sullivan (ryansullivan@googlemail.com)
 * 
 * @file    watchdog.hpp
 * @brief   header
 * 
 * @date    2022-02-26
 * -------------------------------------------------------------------------------------------------
 */
#pragma once

#include <optional>
#include <system_error>
#include <tuple>

#include "system_info.hpp"


namespace iwdg {

/// @brief Status codes for the watchdog module.
///
enum class StatusCode: uint32_t {
    Ok,
    InvalidTimeout,
};

std::error_code make_error_code(StatusCode);

}

namespace std {

template <>
struct is_error_code_enum<iwdg::StatusCode> : true_type {};

}

namespace iwdg {

/// @brief Class representing an instance of the indipendant watchdog peripheral.
///
struct Watchdog {
    explicit Watchdog() = delete;
    explicit Watchdog(Watchdog&) = delete;
    explicit Watchdog(Watchdog&&) = default;
    
    ~Watchdog() = default;

    auto operator=(const Watchdog&) -> Watchdog& = delete;
    auto operator=(Watchdog&&) -> Watchdog& = default;

    static auto with_timeout(const sys::Microseconds timeout) -> std::tuple<std::optional<Watchdog>, std::error_code>;

    auto update() -> void;

private:
    explicit Watchdog(const uint32_t prescaler_value, const uint32_t reload_value);
};

}
