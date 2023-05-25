////////////////////////////////////////////////////////////////////////////////////////////////////
/// @author  Ryan Sullivan (ryansullivan@googlemail.com)
///
/// @file    ringbuf.hpp
/// @brief   Ring buffer module.
////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <span>
#include <system_error>

/*------------------------------------------------------------------------------------------------*/
// Error handling.
/*------------------------------------------------------------------------------------------------*/

namespace buffer {

enum class Error : uint32_t {
    Full,
    Empty,

    NotEnoughSpace,
    NotEnoughItems

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

template<typename T, uint32_t MaxSize>
struct RingBuffer {
    explicit RingBuffer() noexcept = default;
    explicit RingBuffer(const RingBuffer&) noexcept = default;
    explicit RingBuffer(RingBuffer&&) noexcept = default;

    auto operator=(const RingBuffer&) noexcept -> RingBuffer& = default;
    auto operator=(RingBuffer&&) noexcept -> RingBuffer& = default;

    ~RingBuffer() = default;

    [[nodiscard]] auto push(T value) noexcept -> std::expected<void, std::error_code>;
    auto push_unchecked(T value) noexcept -> void;

    [[nodiscard]] auto push_buffer(std::span<const T> buffer) noexcept
        -> std::expected<void, std::error_code>;

    [[nodiscard]] auto pop() noexcept -> std::expected<T, std::error_code>;
    [[nodiscard]] auto pop_unchecked() noexcept -> T;

    [[nodiscard]] auto pop_buffer(std::span<T> buffer) noexcept
        -> std::expected<void, std::error_code>;

    auto clear() noexcept -> void;

    [[nodiscard]] auto empty() const noexcept -> bool;
    [[nodiscard]] auto full() const noexcept -> bool;

    [[nodiscard]] auto size() const noexcept -> uint32_t;
    [[nodiscard]] auto free() const noexcept -> uint32_t;
    [[nodiscard]] auto capacity() const noexcept -> uint32_t;

private:
    std::array<T, MaxSize> _buffer{};
    uint32_t _write_ptr{};
    uint32_t _read_ptr{};
    bool _is_full{};
};

/*------------------------------------------------------------------------------------------------*/
// Class method definitions.
/*------------------------------------------------------------------------------------------------*/

template<typename T, uint32_t MaxSize>
auto RingBuffer<T, MaxSize>::push(const T value) noexcept -> std::expected<void, std::error_code> {

    if(this->_is_full) {
        return std::unexpected{Error::Full};
    }

    this->_buffer[this->_write_ptr++] = value;
    this->_write_ptr = this->_write_ptr % MaxSize;

    if(this->_write_ptr == this->_read_ptr) {
        this->_is_full = true;
    }

    return {};
}

/*------------------------------------------------------------------------------------------------*/

template<typename T, uint32_t MaxSize>
auto RingBuffer<T, MaxSize>::push_unchecked(const T value) noexcept -> void {

    this->_buffer[this->_write_ptr++] = value;
    this->_write_ptr = this->_write_ptr % MaxSize;

    if(this->_write_ptr == this->_read_ptr) {
        this->_is_full = true;
    }
}

/*------------------------------------------------------------------------------------------------*/

template<typename T, uint32_t MaxSize>
auto RingBuffer<T, MaxSize>::push_buffer(const std::span<const T> buffer) noexcept
    -> std::expected<void, std::error_code> {

    if(buffer.size() > this->free()) {
        if(this->_is_full) {
            return std::unexpected{Error::Full};
        }
        return std::unexpected{Error::NotEnoughSpace};
    }

    const auto space_until_wrap = MaxSize - this->_write_ptr;

    if(buffer.size() > space_until_wrap) {
        const auto chunk1 = buffer.first(space_until_wrap);
        const auto chunk2 = buffer.last(buffer.size() - space_until_wrap);

        std::copy(chunk1.begin(), chunk1.end(), std::next(this->_buffer.begin(), this->_write_ptr));
        std::copy(chunk2.begin(), chunk2.end(), this->_buffer.begin());

    } else {
        std::copy(buffer.begin(), buffer.end(), std::next(this->_buffer.begin(), this->_write_ptr));
    }

    this->_write_ptr = (this->_write_ptr + buffer.size()) % MaxSize;

    if(this->_write_ptr == this->_read_ptr) {
        this->_is_full = true;
    }

    return {};
}

/*------------------------------------------------------------------------------------------------*/

template<typename T, uint32_t MaxSize>
auto RingBuffer<T, MaxSize>::pop() noexcept -> std::expected<T, std::error_code> {

    if(this->empty()) {
        return std::unexpected{Error::Empty};
    }

    const auto value = this->_buffer[this->_read_ptr++];
    this->_read_ptr = this->_read_ptr % MaxSize;
    this->_is_full = false;

    return value;
}

/*------------------------------------------------------------------------------------------------*/

template<typename T, uint32_t MaxSize>
auto RingBuffer<T, MaxSize>::pop_unchecked() noexcept -> T {

    const auto value = this->_buffer[this->_read_ptr++];
    this->_read_ptr = this->_read_ptr % MaxSize;
    this->_is_full = false;

    return value;
}

/*------------------------------------------------------------------------------------------------*/

template<typename T, uint32_t MaxSize>
auto RingBuffer<T, MaxSize>::pop_buffer(const std::span<T> buffer) noexcept
    -> std::expected<void, std::error_code> {

    if(buffer.size() > this->size()) {
        if(this->empty()) {
            return std::unexpected{Error::Empty};
        }
        return std::unexpected{Error::NotEnoughItems};
    }

    const auto items_until_wrap = MaxSize - this->_read_ptr;

    if(buffer.size() > items_until_wrap) {
        const auto chunk1 = std::span(this->_buffer).last(items_until_wrap);
        const auto chunk2 = std::span(this->_buffer).first(buffer.size() - items_until_wrap);

        std::copy(chunk1.begin(), chunk1.end(), buffer.begin());
        std::copy(chunk2.begin(), chunk2.end(), std::next(buffer.begin(), items_until_wrap));

    } else {
        const auto begin = std::next(this->_buffer.begin(), this->_read_ptr);
        const auto end = std::next(begin, items_until_wrap);

        std::copy(begin, end, buffer.begin());
    }

    this->_read_ptr = (this->_read_ptr + buffer.size()) % MaxSize;
    this->_is_full = false;

    return {};
}

/*------------------------------------------------------------------------------------------------*/

template<typename T, uint32_t MaxSize>
auto RingBuffer<T, MaxSize>::clear() noexcept -> void {
    this->_write_ptr = 0;
    this->_read_ptr = 0;
    this->_is_full = false;
}

/*------------------------------------------------------------------------------------------------*/

template<typename T, uint32_t MaxSize>
auto RingBuffer<T, MaxSize>::empty() const noexcept -> bool {
    return this->_write_ptr == this->_read_ptr && !this->_is_full;
}

/*------------------------------------------------------------------------------------------------*/

template<typename T, uint32_t MaxSize>
auto RingBuffer<T, MaxSize>::full() const noexcept -> bool {
    return this->_is_full;
}

/*------------------------------------------------------------------------------------------------*/

template<typename T, uint32_t MaxSize>
auto RingBuffer<T, MaxSize>::size() const noexcept -> uint32_t {
    if(this->_is_full) {
        return MaxSize;
    }

    if(this->_write_ptr >= this->_read_ptr) {
        return this->_write_ptr - this->_read_ptr;
    }

    return this->_write_ptr + (MaxSize - this->_read_ptr);
}

/*------------------------------------------------------------------------------------------------*/

template<typename T, uint32_t MaxSize>
[[nodiscard]] auto RingBuffer<T, MaxSize>::free() const noexcept -> uint32_t {
    if(this->_is_full) {
        return 0;
    }

    if(this->_write_ptr >= this->_read_ptr) {
        return (MaxSize - this->_write_ptr) + this->_read_ptr;
    }

    return this->_read_ptr - this->_write_ptr;
}

/*------------------------------------------------------------------------------------------------*/

template<typename T, uint32_t MaxSize>
auto RingBuffer<T, MaxSize>::capacity() const noexcept -> uint32_t {
    return MaxSize;
}

}

/*------------------------------------------------------------------------------------------------*/
