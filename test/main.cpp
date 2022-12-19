#include <cstdint>

#include "CppUTest/CommandLineTestRunner.h"
#include "CppUTest/TestHarness.h"

#include "clocks.hpp"
#include "utils.hpp"
#include "port.hpp"
#include "usart.hpp"
#include "main.hpp"

IMPORT_TEST_GROUP(SecondTestGroup);

#define DEBUG_TX port::C4
#define DEBUG_RX port::C5

int main(void)
{

    if(auto status = clk::initialise(); status != clk::OK)
    {
        while(1) {}
    }
    port::initialise_pin(DEBUG_TX, port::MODE_ALT_OUTPUT, 7);
    port::initialise_pin(DEBUG_RX, port::MODE_INPUT_PULLUP, 7);
    usart::initialise();
    clk::clear_reset_flags();

    utils::wait_ms(1000);

    const char * dummy_args[] = {(char*) 0,};
    int ac = sizeof(dummy_args) / sizeof(char*);
    return CommandLineTestRunner::RunAllTests(ac, dummy_args);
}

void main_1ms_timer_isr()
{
    utils::poll();
}