#include <cstdint>

#include "CppUTest/CommandLineTestRunner.h"
#include "CppUTest/TestHarness.h"

#include "clocks.hpp"
#include "io.hpp"
#include "utils.hpp"
#include "debug.hpp"

#include "main.hpp"

IMPORT_TEST_GROUP(SecondTestGroup);

int main(void)
{

    if(auto status = clk::initialise(); status != clk::OK)
    {
        while(1) {}
    }

    io::initialise();
    clk::clear_reset_flags();

    utils::wait_ms(1000);

    const char * dummy_args[] = {(char*) 0,};
    int ac = sizeof(dummy_args) / sizeof(char*);
    return CommandLineTestRunner::RunAllTests(ac, dummy_args);
}

void main_1ms_timer_isr()
{
    utils::poll();
    io::poll();
}