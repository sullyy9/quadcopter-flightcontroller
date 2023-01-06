#include <cstdint>
#include <utility>

#include "CppUTest/CommandLineTestRunner.h"
#include "CppUTest/TestHarness.h"

#include "clocks.hpp"
#include "utils.hpp"
#include "port.hpp"
#include "usart.hpp"
#include "main.hpp"

IMPORT_TEST_GROUP(SecondTestGroup);

#define DEBUG_TX port::Pin::C4
#define DEBUG_RX port::Pin::C5

auto set_stdout_interface(usart::USART&& out_interface) -> void;

int main(void) {

    if(auto status = clk::initialise(); status != clk::OK) return -1;

    port::initialise_pin(DEBUG_TX, port::Mode::ALT_OUTPUT, 7);
    port::initialise_pin(DEBUG_RX, port::Mode::INPUT_PULLUP, 7);
    clk::clear_reset_flags();

    utils::wait_ms(100);

    auto [usart_result, usart_status] = usart::USART::init({
        .baud_rate = 115'200,
        .enable_rx = false,
        .enable_dma = true,
    });
    if(!usart_result.has_value()) return -1;
    
    set_stdout_interface(std::exchange(usart_result, std::nullopt).value());

    std::printf("\r\n");
    std::printf("\r\n");
    std::printf("\r\n");
    std::printf("Unit Tests\r\n");
    std::printf("----------------------------------------\r\n");
    utils::wait_ms(100);

    const char * dummy_args[] = {(char*) 0,};
    int ac = sizeof(dummy_args) / sizeof(char*);
    return CommandLineTestRunner::RunAllTests(ac, dummy_args);
}

void main_1ms_timer_isr() {
    utils::poll();
}