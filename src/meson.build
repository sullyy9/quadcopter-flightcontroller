subdir('utils')
subdir('drivers')

# Source files
project_src = files(
    'clocks.cpp',
    'debug.cpp',
    'io.cpp',
    'motor.cpp',
    'pwm.cpp',
    'utils.cpp',
    'watchdog.cpp',
)

project_executable_src = files('main.cpp', 'startup.cpp')

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
