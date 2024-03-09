////////////////////////////////////////////////////////////////////////////////////////////////////
/// @file  i2c.hpp
/// @brief Common interface for I2C drivers.
////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once

#include <cstdint>
#include <expected>
#include <span>
#include <system_error>
#include <variant>

/*------------------------------------------------------------------------------------------------*/
// error handling
/*------------------------------------------------------------------------------------------------*/

namespace i2c {

enum class Error : uint32_t {
    Unimplemented,

    InsufficientCapacity,
    InsufficientData,
};

auto make_error_code(Error) -> std::error_code;

template<typename T>
using Result = std::expected<T, std::error_code>;

}

/*------------------------------------------------------------------------------------------------*/

namespace std {

template<>
struct is_error_code_enum<i2c::Error>: true_type {};

}

/*------------------------------------------------------------------------------------------------*/
// types
/*------------------------------------------------------------------------------------------------*/

namespace i2c {

using Address = uint32_t;

namespace operation {

    struct Write {
        std::span<const std::byte> source{};
    };

    struct Read {
        std::span<std::byte> destination{};
    };

}

using Operation = std::variant<operation::Write, operation::Read>;

}

/*------------------------------------------------------------------------------------------------*/
// driver concepts
/*------------------------------------------------------------------------------------------------*/

namespace i2c {

/// @brief Base for I2C drivers.
///
/// Implementors of this can then use I2CDriverDefaults to inherit the default implementations of
/// methods required to be a full I2CDriver.
///
template<typename T>
concept I2CDriverBase = requires(Address address, std::span<Operation> operations) {
    { T::transaction(address, operations) } -> std::same_as<Result<void>>;
};

/*------------------------------------------------------------------------------------------------*/

template<typename T>
concept I2CDriverReadWrite =
    requires(T self, Address address, std::span<std::byte> read, std::span<const std::byte> write) {
        { T::read(address, read) } -> std::same_as<Result<void>>;
        { T::write(address, write) } -> std::same_as<Result<void>>;

        { T::write_read(address, write, read) } -> std::same_as<Result<void>>;
    };

/*------------------------------------------------------------------------------------------------*/

template<typename T>
concept I2CDriver = I2CDriverBase<T> && I2CDriverReadWrite<T>;

}

/*------------------------------------------------------------------------------------------------*/
// default implementations
/*------------------------------------------------------------------------------------------------*/

namespace i2c::default_impl {

template<I2CDriverBase T>
auto read(const Address address, const std::span<std::byte> buffer) -> Result<void> {
    auto operations = std::array{Operation{operation::Read{buffer}}};
    return T::transaction(address, operations);
}

/*------------------------------------------------------------------------------------------------*/

template<I2CDriverBase T>
auto write(const Address address, const std::span<const std::byte> buffer) -> Result<void> {
    auto operations = std::array{Operation{operation::Write{buffer}}};
    return T::transaction(address, operations);
}

/*------------------------------------------------------------------------------------------------*/

template<I2CDriverBase T>
auto write_read(const Address address,
                const std::span<const std::byte> write_buffer,
                const std::span<std::byte> read_buffer) -> Result<void> {
    auto operations = std::array{Operation{operation::Write{write_buffer}},
                                 Operation{operation::Read{read_buffer}}};
    return T::transaction(address, operations);
}

}

/*------------------------------------------------------------------------------------------------*/
