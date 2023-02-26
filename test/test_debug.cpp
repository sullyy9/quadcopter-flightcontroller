#include <system_error>

#include "CppUTest/UtestMacros.h"
#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

#include "usart_mock.hpp"

#include "debug.hpp"
#include "utils.hpp"
#include "usart.hpp"

using SerialDebug = debug::Serial<usart::mock::cpputest::USART>;

TEST_GROUP(DebugPrint)
{
    TEST_SETUP() {
        SerialDebug::init({
            .baud_rate = 115'200,
            .enable_rx = false,
            .enable_dma = true,
        });
    }

    TEST_TEARDOWN() {
        mock().checkExpectations();
        mock().clear();
    }
};

TEST(DebugPrint, TestString) {
    mock().strictOrder();
    mock().expectOneCall("tx_byte").withParameter("byte", 't');
    mock().expectOneCall("tx_byte").withParameter("byte", 'e');
    mock().expectOneCall("tx_byte").withParameter("byte", 's');
    mock().expectOneCall("tx_byte").withParameter("byte", 't');
    mock().expectOneCall("tx_flush");

    SerialDebug::print("test");
    SerialDebug::flush();
}

TEST(DebugPrint, TestStringFormat) {
    mock().strictOrder();
    mock().expectOneCall("tx_byte").withParameter("byte", 't');
    mock().expectOneCall("tx_byte").withParameter("byte", 'e');
    mock().expectOneCall("tx_byte").withParameter("byte", 's');
    mock().expectOneCall("tx_byte").withParameter("byte", 't');
    mock().expectOneCall("tx_byte").withParameter("byte", ' ');
    mock().expectOneCall("tx_byte").withParameter("byte", 't');
    mock().expectOneCall("tx_byte").withParameter("byte", 'e');
    mock().expectOneCall("tx_byte").withParameter("byte", 's');
    mock().expectOneCall("tx_byte").withParameter("byte", 't');
    mock().expectOneCall("tx_flush");

    SerialDebug::print("test %", "test");
    SerialDebug::flush();
}

TEST(DebugPrint, TestCharFormat) {
    mock().strictOrder();
    mock().expectOneCall("tx_byte").withParameter("byte", 't');
    mock().expectOneCall("tx_byte").withParameter("byte", 'e');
    mock().expectOneCall("tx_byte").withParameter("byte", 's');
    mock().expectOneCall("tx_byte").withParameter("byte", 't');
    mock().expectOneCall("tx_byte").withParameter("byte", ' ');
    mock().expectOneCall("tx_byte").withParameter("byte", 'f');
    mock().expectOneCall("tx_flush");

    SerialDebug::print("test %", 'f');
    SerialDebug::flush();
}

TEST(DebugPrint, TestIntFormat) {
    mock().strictOrder();
    mock().expectOneCall("tx_byte").withParameter("byte", 'n');
    mock().expectOneCall("tx_byte").withParameter("byte", 'u');
    mock().expectOneCall("tx_byte").withParameter("byte", 'm');
    mock().expectOneCall("tx_byte").withParameter("byte", ' ');
    mock().expectOneCall("tx_byte").withParameter("byte", '5');
    mock().expectOneCall("tx_byte").withParameter("byte", '6');
    mock().expectOneCall("tx_byte").withParameter("byte", ' ');
    mock().expectOneCall("tx_byte").withParameter("byte", '0');
    mock().expectOneCall("tx_byte").withParameter("byte", ' ');
    mock().expectOneCall("tx_byte").withParameter("byte", '-');
    mock().expectOneCall("tx_byte").withParameter("byte", '9');
    mock().expectOneCall("tx_byte").withParameter("byte", '8');
    mock().expectOneCall("tx_byte").withParameter("byte", '7');
    mock().expectOneCall("tx_byte").withParameter("byte", '6');
    mock().expectOneCall("tx_flush");

    SerialDebug::print("num % % %", uint32_t{56}, 0, -9876);
    SerialDebug::flush();
}