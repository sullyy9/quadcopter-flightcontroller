subdir('drivers')

# Source files

project_src = files(
    'clocks.cpp',
    'debug.cpp',
    'i2c.cpp',
    'io.cpp',
    'main.cpp',
    'spi.cpp',
    'startup.cpp',
    'utils.cpp',
    'watchdog.cpp',
    'pwm.cpp',
    'motor.cpp',
)

project_inc = include_directories(
    '.'
)

# Libraries

project_simulator_lib = static_library(
    'project-simulator',
    sources: project_src,
    include_directories: '.',
    dependencies: [driver_simulator_dep, stm32f3_ll_driver_dep]
)

project_stm32f303_lib = static_library(
    'project-stm32f303',
    sources: project_src,
    include_directories: '.',
    dependencies: [driver_stm32f303_dep, stm32f3_ll_driver_dep]
)

# Dependancies

project_simulator_dep = declare_dependency(
    include_directories: '.',
    link_with: project_simulator_lib,
    dependencies: [driver_simulator_dep, stm32f3_ll_driver_dep]
)

project_stm32f303_dep = declare_dependency(
    include_directories: '.',
    link_with: project_stm32f303_lib,
    dependencies: [driver_stm32f303_dep, stm32f3_ll_driver_dep]
)
