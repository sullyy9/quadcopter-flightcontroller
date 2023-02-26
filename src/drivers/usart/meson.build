# Source files

driver_usart_simulator_src = files(
    './usart.cpp',
    './usart_simulator.cpp'
)

driver_usart_mock_src = files(
    './usart.cpp',
    './usart_mock.cpp'
)

driver_usart_stm32f303_src = files(
    './usart.cpp',
    './usart_stm32f303.cpp'
)

# Libraries

driver_usart_simulator_lib = static_library(
    'driver-usart-simulator',
    sources: driver_usart_simulator_src,
    include_directories: '.',
)

driver_usart_mock_lib = static_library(
    'driver-usart-mock',
    sources: driver_usart_mock_src,
    include_directories: '.',
    dependencies: cpputest_dep
)

driver_usart_stm32f303_lib = static_library(
    'driver-usart-stm32f303',
    sources: driver_usart_stm32f303_src,
    include_directories: '.',
    dependencies: stm32f3_ll_driver_dep
)

# Dependancies

driver_usart_simulator_dep = declare_dependency(
    include_directories: '.',
    link_with: driver_usart_simulator_lib,
)

driver_usart_mock_dep = declare_dependency(
    include_directories: '.',
    link_with: driver_usart_mock_lib,
)


driver_usart_stm32f303_dep = declare_dependency(
    include_directories: '.',
    link_with: driver_usart_stm32f303_lib,
)
