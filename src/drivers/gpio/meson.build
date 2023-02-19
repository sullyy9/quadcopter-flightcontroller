# Source files

driver_gpio_simulator_src = files(
    './gpio.cpp',
    './gpio_simulator.cpp'
)

driver_gpio_stm32f303_src = files(
    './gpio.cpp',
    './gpio_stm32f303.cpp'
)

# Libraries

driver_gpio_simulator_lib = static_library(
    'driver-gpio-simulator',
    sources: driver_gpio_simulator_src,
    include_directories: '.',
)

driver_gpio_stm32f303_lib = static_library(
    'driver-gpio-stm32f303',
    sources: driver_gpio_stm32f303_src,
    include_directories: '.',
    dependencies: stm32f3_ll_driver_dep
)

# Dependancies

driver_gpio_simulator_dep = declare_dependency(
    include_directories: '.',
    link_with: driver_gpio_simulator_lib,
)

driver_gpio_stm32f303_dep = declare_dependency(
    include_directories: '.',
    link_with: driver_gpio_stm32f303_lib,
)
