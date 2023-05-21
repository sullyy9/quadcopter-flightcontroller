////////////////////////////////////////////////////////////////////////////////////////////////////
/// @author  Ryan Sullivan (ryansullivan@googlemail.com)
///
/// @file    ringbuf.hpp
/// @brief   Ring buffer module.
////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once

#include <array>
#include <cstddef>
#include <expected>
#include <system_error>

/*------------------------------------------------------------------------------------------------*/
// Error handling.
/*------------------------------------------------------------------------------------------------*/

namespace buffer {

enum class Error : size_t {
    Full,
    Empty,

};

auto make_error_code(Error) -> std::error_code;

}

namespace std {

template<>
struct is_error_code_enum<buffer::Error>: true_type {};

}

/*------------------------------------------------------------------------------------------------*/
// Error handling.
/*------------------------------------------------------------------------------------------------*/

namespace buffer {

template<typename T, size_t MaxSize>
struct RingBuffer {
    explicit RingBuffer() noexcept = default;
    explicit RingBuffer(const RingBuffer&) noexcept = default;
    explicit RingBuffer(RingBuffer&&) noexcept = default;

    auto operator=(const RingBuffer&) noexcept -> RingBuffer& = default;
    auto operator=(RingBuffer&&) noexcept -> RingBuffer& = default;

    ~RingBuffer() = default;

    [[nodiscard]] auto push(T value) noexcept -> std::expected<void, std::error_code>;
    auto push_unchecked(T value) noexcept -> void;

    [[nodiscard]] auto pop() noexcept -> std::expected<T, std::error_code>;
    [[nodiscard]] auto pop_unchecked() noexcept -> T;

    auto clear() noexcept -> void;

    [[nodiscard]] auto empty() const noexcept -> bool;
    [[nodiscard]] auto full() const noexcept -> bool;

    [[nodiscard]] auto size() const noexcept -> size_t;
    [[nodiscard]] auto capacity() const noexcept -> size_t;

private:
    std::array<T, MaxSize> buffer{};
    size_t write_ptr{};
    size_t read_ptr{};
    bool is_full{};
};

/*------------------------------------------------------------------------------------------------*/
// Class method definitions.
/*------------------------------------------------------------------------------------------------*/

template<typename T, size_t MaxSize>
auto RingBuffer<T, MaxSize>::push(const T value) noexcept -> std::expected<void, std::error_code> {

    if(this->is_full) {
        return std::unexpected{Error::Full};
    }

    this->buffer[this->write_ptr++] = value;
    this->write_ptr = this->write_ptr % MaxSize;

    if(this->write_ptr == this->read_ptr) {
        this->is_full = true;
    }

    return {};
}

/*------------------------------------------------------------------------------------------------*/

template<typename T, size_t MaxSize>
auto RingBuffer<T, MaxSize>::push_unchecked(const T value) noexcept -> void {

    this->buffer[this->write_ptr++] = value;
    this->write_ptr = this->write_ptr % MaxSize;

    if(this->write_ptr == this->read_ptr) {
        this->is_full = true;
    }
}

/*------------------------------------------------------------------------------------------------*/

template<typename T, size_t MaxSize>
auto RingBuffer<T, MaxSize>::pop() noexcept -> std::expected<T, std::error_code> {

    if(this->empty()) {
        return std::unexpected{Error::Empty};
    }

    const auto value = this->buffer[this->read_ptr++];
    this->read_ptr = this->read_ptr % MaxSize;
    this->is_full = false;

    return value;
}

/*------------------------------------------------------------------------------------------------*/

template<typename T, size_t MaxSize>
auto RingBuffer<T, MaxSize>::pop_unchecked() noexcept -> T {

    const auto value = this->buffer[this->read_ptr++];
    this->read_ptr = this->read_ptr % MaxSize;
    this->is_full = false;

    return value;
}

/*------------------------------------------------------------------------------------------------*/

template<typename T, size_t MaxSize>
auto RingBuffer<T, MaxSize>::clear() noexcept -> void {
    this->write_ptr = 0;
    this->read_ptr = 0;
    this->is_full = false;
}

/*------------------------------------------------------------------------------------------------*/

template<typename T, size_t MaxSize>
auto RingBuffer<T, MaxSize>::empty() const noexcept -> bool {
    return this->write_ptr == this->read_ptr && !this->is_full;
}

/*------------------------------------------------------------------------------------------------*/

template<typename T, size_t MaxSize>
auto RingBuffer<T, MaxSize>::full() const noexcept -> bool {
    return this->is_full;
}

/*------------------------------------------------------------------------------------------------*/

template<typename T, size_t MaxSize>
auto RingBuffer<T, MaxSize>::size() const noexcept -> size_t {
    if(this->is_full) {
        return MaxSize;
    }

    if(this->write_ptr >= this->read_ptr) {
        return this->write_ptr - this->read_ptr;
    }

    return this->write_ptr + (MaxSize - this->read_ptr);
}

/*------------------------------------------------------------------------------------------------*/

template<typename T, size_t MaxSize>
auto RingBuffer<T, MaxSize>::capacity() const noexcept -> size_t {
    return MaxSize;
}

}

/*------------------------------------------------------------------------------------------------*/
