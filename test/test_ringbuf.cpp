////////////////////////////////////////////////////////////////////////////////////////////////////
/// @author  Ryan Sullivan (ryansullivan@googlemail.com)
///
/// @file    test_ringbuf.cpp
/// @brief   Tests for RingBuffer.
////////////////////////////////////////////////////////////////////////////////////////////////////

#include <cstddef>
#include <string_view>
#include <system_error>

#include "CppUTest/TestHarness.h"

#include "ringbuf.hpp"

using namespace std::literals;

// clang-format off
TEST_GROUP(TestRingBuffer) {
    TEST_SETUP() {

    }

    TEST_TEARDOWN() {

    }
};
// clang-format on

static constexpr std::string_view TEST_STRING = {"test string"sv};

TEST(TestRingBuffer, TestReadWrite) {
    const size_t buffer_capacity = 16;
    buffer::RingBuffer<char, buffer_capacity> buffer{};

    CHECK(buffer.empty());
    CHECK_FALSE(buffer.full());
    CHECK_EQUAL(0, buffer.size());

    size_t buffer_count = 0;

    for(uint32_t i = 0; i < 10; i++) {
        // Write to the buffer.
        for(const auto character : TEST_STRING) {
            buffer.push_unchecked(character);
            buffer_count++;

            CHECK_EQUAL(buffer.size(), buffer_count);

            if(buffer_count == buffer_capacity) {
                CHECK(buffer.full());
            } else {
                CHECK_FALSE(buffer.full());
            }

            if(buffer_count == 0) {
                CHECK(buffer.empty());
            } else {
                CHECK_FALSE(buffer.empty());
            }
        }

        // Read from the buffer.
        for(const auto expected_character : TEST_STRING) {
            const auto actual_character = buffer.pop_unchecked();
            buffer_count--;

            CHECK_EQUAL(buffer.size(), buffer_count);

            if(buffer_count == TEST_STRING.length()) {
                CHECK(buffer.full());
            } else {
                CHECK_FALSE(buffer.full());
            }

            if(buffer_count == 0) {
                CHECK(buffer.empty());
            } else {
                CHECK_FALSE(buffer.empty());
            }

            CHECK_EQUAL(expected_character, actual_character);
        }
    }
}

TEST(TestRingBuffer, TestOverflow) {
    const size_t buffer_capacity = 4;
    buffer::RingBuffer<char, buffer_capacity> buffer{};
    size_t buffer_count = 0;

    for(const auto character : TEST_STRING) {
        const auto result = buffer.push(character);
        buffer_count++;

        if(buffer_count >= buffer_capacity) {
            CHECK(buffer.full());
        }

        if(buffer_count > buffer_capacity) {
            CHECK_FALSE(result.has_value());
            CHECK(result.error() == buffer::Error::Full);
        } else {
            CHECK(result.has_value());
        }
    }

    // Read and check the characters that were written.
    for(size_t i = 0; i < buffer_capacity; i++) {
        const auto result = buffer.pop();

        CHECK(result.has_value());
        CHECK_EQUAL(TEST_STRING[i], result.value());
    }

    CHECK(buffer.empty());
}

TEST(TestRingBuffer, TestUnderflow) {
    buffer::RingBuffer<char, 4> buffer{};
    CHECK(buffer.empty());

    const auto result = buffer.pop();

    CHECK_FALSE(result.has_value());
    CHECK(result.error() == buffer::Error::Empty);
}

TEST(TestRingBuffer, TestPushBuffer) {
    buffer::RingBuffer<char, 8> buffer{};

    // Check a push that would result in an overflow returns an error.
    auto result = buffer.push_buffer(TEST_STRING);
    CHECK(!result.has_value());
    CHECK(result.error() == buffer::Error::NotEnoughSpace);

    for(uint32_t i = 0; i < 10; i++) {
        result = buffer.push_buffer(TEST_STRING.substr(0, 6));
        CHECK(result.has_value());

        for(const auto expected_char : TEST_STRING.substr(0, 6)) {
            CHECK_EQUAL(expected_char, buffer.pop_unchecked());
        }
    }
}

TEST(TestRingBuffer, TestPopBuffer) {
    buffer::RingBuffer<char, 8> buffer{};

    for(uint32_t i = 0; i < 10; i++) {
        for(const auto character : TEST_STRING.substr(0, 6)) {
            CHECK(buffer.push(character).has_value());
        }

        std::array<char, 6> read_buffer = {};
        const auto result = buffer.pop_buffer(read_buffer);
        CHECK(result.has_value());
        STRNCMP_EQUAL(TEST_STRING.substr(0, 6).data(), read_buffer.data(), 6);
    }
}
