subdir('gpio')
subdir('usart')
subdir('i2c')
subdir('spi')

# Dependancies
driver_simulator_dep = declare_dependency(
    dependencies: [
        driver_gpio_simulator_dep,
        driver_usart_simulator_dep,
    ],
)

driver_mock_dep = declare_dependency(
    dependencies: [
        driver_usart_mock_dep,
        driver_i2c_mock_dep,
        driver_spi_mock_dep,
    ],
)

driver_stm32f303_dep = declare_dependency(
    dependencies: [
        driver_gpio_stm32f303_dep,
        driver_usart_stm32f303_dep,
        driver_i2c_stm32f303_dep,
        driver_spi_stm32f303_dep,
    ],
)
