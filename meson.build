project(
    'quadcopter-flightcontroller',
    'cpp',
    version: '1.0',
    default_options: ['warning_level=3', 'b_staticpic=false'],
)

add_project_arguments(
    [
        '-std=c++23',
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

if(meson.is_cross_build())
    add_project_link_arguments(
        [
            '-Wl,-T,@0@/@1@'.format(meson.current_source_dir(), linkscript[0]),
            '-Wl,--gc-sections',
            '-Wl,--print-memory-usage',
            '-Wl,--no-warn-rwx-segment',
        ],
        language: ['cpp'],
    )
endif

####################################################################################################
# Setup dependencies

cmake = import('cmake')
opt_var = cmake.subproject_options()

stm32f3_hal_driver_proj = subproject('stm32f3_hal_driver')
stm32f3_ll_driver_dep = stm32f3_hal_driver_proj.get_variable(
    'stm32f3_ll_driver_dep',
)

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

if(meson.is_cross_build())
    executable(
        'quadcopter-flightcontroller',
        name_suffix: 'elf',
        link_args: [
            '-nostartfiles',
            '--specs=nano.specs',
            '--specs=nosys.specs',
        ],
        dependencies: [project_executable_dep, driver_stm32f303_dep],
    )
endif

if get_option('enable_test')
    subdir('test')

    if(meson.is_cross_build())
        executable(
            'unit_test',
            name_suffix: 'elf',
            sources: stm32f303_test_src,
            include_directories: './test',
            link_args: [
                '-nostartfiles',
                '--specs=nano.specs',

                '-Wl,-Map=test.map',
            ],
            dependencies: [project_testing_dep, driver_mock_dep, driver_stm32f303_dep, cpputest_dep, cpputestext_dep],
            build_by_default: true,
        )
    
    else
        executable(
            'unit_test_native',
            sources: native_test_src,
            include_directories: './test',
            dependencies: [project_native_testing_dep, driver_mock_dep, cpputest_dep, cpputestext_dep],
            build_by_default: true,
        )
    endif
endif
