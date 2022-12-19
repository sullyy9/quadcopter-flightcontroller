project(
    'quadcopter-flightcontroller',
    'cpp',
    version: '1.0',
    default_options: ['cpp_std=c++20', 'warning_level=3', 'b_staticpic=false'],
)

add_project_arguments(
    [
        '-fdata-sections',
        '-ffunction-sections',
        '-fno-common',
        '-fno-unwind-tables',

        '-DRESULT_DISABLE_EXCEPTIONS',

        '-Wundef',
        '-Wshadow',
        '-Wconversion',
        '-Wno-volatile',
        '-Wsign-conversion',
        '-Wdouble-promotion',
    ],
    language: ['cpp'],
)

linkscript = files('link_script.ld')

add_project_link_arguments(
    [
        '-Wl,-T,@0@/@1@'.format(meson.current_source_dir(), linkscript[0]),
        '-Wl,--gc-sections',
        '-Wl,--print-memory-usage',
    ],
    language: ['cpp'],
)

####################################################################################################
# Setup dependencies

cmake = import('cmake')
opt_var = cmake.subproject_options()

stm32f3_hal_driver_proj = subproject('stm32f3_hal_driver')
stm32f3_ll_driver_dep = stm32f3_hal_driver_proj.get_variable(
    'stm32f3_ll_driver_dep',
)

result_proj = cmake.subproject('result')
result_dep = result_proj.dependency('Result', include_type: 'system')

opt_var.add_cmake_defines(
    {
        'STD_CPP': false,
        'TESTS': false,
        'HAVE_PTHREAD_MUTEX_LOCK': false,
        'HAVE_FORK': false,
        'VERBOSE_CONFIG': false,
    },
)
opt_var.set_override_option('warning_level', '0')
opt_var.append_compile_args('cpp', '-Wno-conversion', '-Wno-dev')
cpputest_proj = cmake.subproject('cpputest', options: opt_var)

cpputest_dep = cpputest_proj.dependency('CppUTest', include_type: 'system')
cpputestext_dep = cpputest_proj.dependency('CppUTestExt', include_type: 'system')

####################################################################################################

subdir('src')

executable(
    'quadcopter-flightcontroller',
    name_suffix: 'elf',
    sources: project_src,
    include_directories: include_directories('src'),
    link_args: [
        '-nostartfiles',
        '--specs=nano.specs',
        '--specs=nosys.specs',
    ],
    dependencies: [stm32f3_ll_driver_dep, result_dep],
)

if get_option('enable_test')
    subdir('test')

    flightcontroller_lib = static_library(
        'quadcopter-flightcontroller',
        sources: project_src,
        include_directories: include_directories('src'),
        dependencies: [stm32f3_ll_driver_dep, result_dep],
    )

    flightcontroller_dep = declare_dependency(
        include_directories: include_directories('src'),
        link_with: flightcontroller_lib,
        dependencies: [stm32f3_ll_driver_dep, result_dep],
    )

    executable(
        'unit_test',
        name_suffix: 'elf',
        sources: [test_src],
        include_directories: [include_directories(['src', 'test'])],
        dependencies: [cpputest_dep, cpputestext_dep, flightcontroller_dep],
        link_args: [
            '-nostartfiles',
            '--specs=nano.specs',

            '-Wl,-Map=test.map',
        ],
        build_by_default: true,
    )
endif
