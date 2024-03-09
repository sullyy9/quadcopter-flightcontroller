# Source files
driver_i2c_mock_src = files('./i2c.cpp', './i2c_mock.cpp')
driver_i2c_stm32f303_src = files('./i2c.cpp', './i2c_stm32f303.cpp')

# Dependancies
driver_i2c_mock_dep = declare_dependency(
    include_directories: '.',
    sources: driver_i2c_mock_src,
)

driver_i2c_stm32f303_dep = declare_dependency(
    include_directories: '.',
    sources: driver_i2c_stm32f303_src,
)
