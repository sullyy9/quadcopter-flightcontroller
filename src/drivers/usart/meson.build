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

# Dependancies

driver_usart_simulator_dep = declare_dependency(
    include_directories: '.',
    sources: driver_usart_simulator_src,
)

driver_usart_mock_dep = declare_dependency(
    include_directories: '.',
    sources: driver_usart_mock_src,
)


driver_usart_stm32f303_dep = declare_dependency(
    include_directories: '.',
    sources: driver_usart_stm32f303_src,
)
