////////////////////////////////////////////////////////////////////////////////////////////////////
/// @author  Ryan Sullivan (ryansullivan@googlemail.com)
///
/// @file    ringbuf.cpp
/// @brief   Ring buffer module.
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "ringbuf.hpp"

#include <system_error>

/*------------------------------------------------------------------------------------------------*/
// Error handling.
/*------------------------------------------------------------------------------------------------*/

struct RingBufferErrorCategory: std::error_category {
    [[nodiscard]] auto name() const noexcept -> const char* override;
    [[nodiscard]] auto message(int status) const -> std::string override;
};
const RingBufferErrorCategory RING_BUFFER_ERROR_CATEGORY{};

/*------------------------------------------------------------------------------------------------*/

auto RingBufferErrorCategory::name() const noexcept -> const char* {
    return "RingBuffer";
}

/*------------------------------------------------------------------------------------------------*/

auto RingBufferErrorCategory::message(const int status) const -> std::string {
    using enum buffer::Error;

    switch(static_cast<buffer::Error>(status)) {
        case Full: return "Full";
        case Empty: return "Empty";

        default: return "Unknown";
    }
}

/*------------------------------------------------------------------------------------------------*/

auto buffer::make_error_code(const buffer::Error status) -> std::error_code {
    return {static_cast<int>(status), RING_BUFFER_ERROR_CATEGORY};
}

/*------------------------------------------------------------------------------------------------*/
