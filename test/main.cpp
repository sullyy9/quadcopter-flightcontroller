#include <cstdint>
#include <system_error>
#include <utility>

#include "CppUTest/CommandLineTestRunner.h"
#include "CppUTest/TestHarness.h"

#include "clocks.hpp"
#include "utils.hpp"
#include "usart.hpp"
#include "main.hpp"

#include "gpio.hpp"
#include "gpio_stm32f303.hpp"

using GPIO = gpio::GPIOSTM32F303;

IMPORT_TEST_GROUP(SecondTestGroup);

static constexpr auto DEBUG_TX {gpio::Pin::C4};
static constexpr auto DEBUG_RX {gpio::Pin::C5};

struct GPIOConfig {
    gpio::Pin pin {};
    gpio::Config config {};
};
constexpr std::array GPIO_INITIAL_CONFIGURATION_TABLE {
    GPIOConfig{DEBUG_TX, gpio::AltFunction{.output_type = gpio::OutputType::PushPull,  .pull = gpio::Pull::None, .function = 7}},
    GPIOConfig{DEBUG_RX, gpio::AltFunction{.output_type = gpio::OutputType::OpenDrain, .pull = gpio::Pull::Up,   .function = 7}},
};

auto set_stdout_interface(usart::USART&& out_interface) -> void;

int main(void) {

    if(auto status = clk::initialise(); status != clk::OK) return -1;

    // Initialise the GPIO pins.
    for(const auto& [pin, config]: GPIO_INITIAL_CONFIGURATION_TABLE) {
        if(GPIO::init(pin, config) != gpio::StatusCode::Ok) {
            return -1;
        }
    }

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