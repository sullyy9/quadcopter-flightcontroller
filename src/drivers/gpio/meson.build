# Source files

driver_gpio_simulator_src = files(
    './gpio.cpp',
    './gpio_simulator.cpp'
)

driver_gpio_stm32f303_src = files(
    './gpio.cpp',
    './gpio_stm32f303.cpp'
)

# Dependancies

driver_gpio_simulator_dep = declare_dependency(
    include_directories: '.',
    sources: driver_gpio_simulator_src,
)

driver_gpio_stm32f303_dep = declare_dependency(
    include_directories: '.',
    sources: driver_gpio_stm32f303_src,
)
