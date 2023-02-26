////////////////////////////////////////////////////////////////////////////////////////////////////
/// @author  Ryan Sullivan (ryansullivan@googlemail.com)
///
/// @file    debug.cpp
/// @brief   Module containing debugging utilities.
////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once

#include <limits>
#include <cstdint>
#include <string_view>
#include <optional>
#include <array>
#include <system_error>

#include "usart.hpp"

/*------------------------------------------------------------------------------------------------*/
// Class declarations.
/*------------------------------------------------------------------------------------------------*/

namespace debug {

template<usart::USARTDriver Comms>
struct Serial {

    static auto init(const usart::USARTConfig& driver_config) -> std::error_code {
        return Comms::init(driver_config);
    }

    /// @brief Print a formatted string to the serial debug interface.
    ///
    /// @param format String specifying the format for the args.
    /// @param args   Args.
    ///
    static constexpr auto print(const std::string_view& format, const auto&... args) -> void {
        print_format(format, args...);
    }

    static constexpr auto flush() -> void {
        Comms::tx_flush();
    }

private:
    static constexpr auto print_format(const std::string_view& format) -> void {
        for(const auto c: format) {
            print_type(c);
        }
    }

    static constexpr auto print_format(const std::string_view& format, const auto& arg, const auto&... args) -> void {
        uint32_t i = 1;
        
        for(const auto c: format) {
            
            if (c != '%') {
                print_type(c);
            }
            else {
                print_type(arg);
                print_format(format.substr(i), args...);
                break;
            }
            i++;
        }
    }

    /// @brief Print a single character to the debug interface.
    ///
    /// @param character Character to print.
    ///
    static auto print_type(const char character) -> void {
        Comms::tx_byte(std::byte(character));
    }

    /// @brief Print an integer to the debug interface.
    ///
    /// @param val Integer to print.
    ///
    static constexpr auto print_type(const std::integral auto val) -> void {   
        std::array<char, max_digits<decltype(val)>()> buffer {};
        uint8_t buffer_ptr = 0;

        if(val == 0) {
            print_type('0');
            return;
        }

        if constexpr(std::is_signed_v<decltype(val)>) {
            if(val < 0) {
                for(auto num = val * -1; num > 0; num /= 10) {
                    buffer[buffer_ptr++] = ('0' + static_cast<uint8_t>(num % 10));
                }
                buffer[buffer_ptr++] = '-';
            }
            else {
                for(auto num = val; num > 0; num /= 10) {
                    buffer[buffer_ptr++] = ('0' + static_cast<uint8_t>(num % 10));
                }                
            }
        }
        else {
            for(auto num = val; num > 0; num /= 10) {
                buffer[buffer_ptr++] = ('0' + static_cast<uint8_t>(num % 10));
            }
        }
        
        while(buffer_ptr > 0) {
            print_type(buffer[--buffer_ptr]);
        }
    }

    /// @brief Print a string to the debug interface.
    ///
    /// @param val String to print.
    ///
    static constexpr auto print_type(const std::string_view& val) -> void {
        for(const auto c: val) {
            print_type(c);
        }
    }

    template<std::integral T>
    static consteval auto max_digits() -> int {
        auto count = 0;
        for(auto i = std::numeric_limits<T>::max(); i > 0; i /= 10) count++;
        if(std::is_signed_v<T>) count++;
        return count;
    }

    static_assert(max_digits<uint8_t>() == 3);
    static_assert(max_digits<int8_t>() == 4);
    static_assert(max_digits<int16_t>() == 6);
    static_assert(max_digits<uint32_t>() == 10);
};

}

/*------------------------------------------------------------------------------------------------*/
// Module public functions.
/*------------------------------------------------------------------------------------------------*/

namespace debug {

void     stopwatch_initialise(void);
void     stopwatch_start(void);
uint32_t stopwatch_stop(void);

}

/*------------------------------------------------------------------------------------------------*/
