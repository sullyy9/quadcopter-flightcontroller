////////////////////////////////////////////////////////////////////////////////////////////////////
/// @author  Ryan Sullivan (ryansullivan@googlemail.com)
///
/// @file    esc.hpp
/// @brief   Concept declaration for ESC's.
////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once

#include <chrono>
#include <cstdint>

/*------------------------------------------------------------------------------------------------*/

namespace esc {

template<typename T>
concept ESCDefinition = requires(T) {
    {T::MIN_PULSE_WIDTH} -> std::convertible_to<std::chrono::microseconds>;
    {T::MAX_PULSE_WIDTH} -> std::convertible_to<std::chrono::microseconds>;
    {T::MAX_FREQUENCY_HZ} -> std::convertible_to<uint32_t>;
};

}

/*------------------------------------------------------------------------------------------------*/
// Class declarations.
/*------------------------------------------------------------------------------------------------*/

namespace esc {

struct HobbywingXRotor {
    static constexpr std::chrono::microseconds MIN_PULSE_WIDTH {1100};
    static constexpr std::chrono::microseconds MAX_PULSE_WIDTH {1940};
    static constexpr uint32_t MAX_FREQUENCY_HZ {500};
};

static_assert(ESCDefinition<HobbywingXRotor>);

}

/*------------------------------------------------------------------------------------------------*/
