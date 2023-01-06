#include "CppUTest/UtestMacros.h"
#include "debug.hpp"
#include "utils.hpp"

#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

struct SerialMock {
    auto tx_free(void) -> uint32_t {
        mock().actualCall("tx_free");
        return 0;
    }

    auto tx_byte(const std::byte byte) -> void {
        mock().actualCall("tx_byte").withParameter("byte", static_cast<int>(byte));
    }

    auto tx_flush() -> void {
        mock().actualCall("tx_flush");
    }
};

static_assert(SerialComms<SerialMock>);

using SerialDebug = debug::Serial<SerialMock>;

TEST_GROUP(DebugPrint)
{
    TEST_SETUP() {
        SerialDebug::set_interface(SerialMock());
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

TEST(DebugPrint, TestClearedInterface) {
    mock().expectNoCall("tx_byte");
    mock().expectNoCall("tx_flush");

    SerialDebug::clear_interface();
    SerialDebug::print("num % % %", uint32_t{56}, 0, -9876);
    SerialDebug::flush();
}