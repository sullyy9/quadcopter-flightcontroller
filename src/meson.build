subdir('utils')
subdir('drivers')

# Source files

project_src = files(
    'clocks.cpp',
    'debug.cpp',
    'i2c.cpp',
    'io.cpp',
    'spi.cpp',
    'utils.cpp',
    'watchdog.cpp',
    'pwm.cpp',
    'motor.cpp',
)

project_executable_src = files(
    'main.cpp',
    'startup.cpp'
)

# Dependancies

project_testing_dep = declare_dependency(
    include_directories: '.',
    sources: project_src,
    dependencies: [utils_dep, stm32f3_ll_driver_dep],
)

project_native_testing_dep = declare_dependency(
    include_directories: '.',
    sources: [],
    dependencies: [utils_dep],
)

project_executable_dep = declare_dependency(
    include_directories: '.',
    sources: project_src + project_executable_src,
    dependencies: [utils_dep, stm32f3_ll_driver_dep],
)
