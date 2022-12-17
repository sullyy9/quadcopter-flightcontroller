project(
    'stm32f3_hal_driver',
    'c',
    version: '1.0',
    default_options: ['c_std=c99', 'b_staticpic=false'],
)

add_project_arguments(
    [
        '-fdata-sections',
        '-ffunction-sections',
        '-fno-common',

        '-w',

        '-DSTM32F303xC',
        '-DUSE_FULL_LL_DRIVER',
        '-DUSE_WWDG',
    ],
    language: ['c'],
)

cmsis_src = files(
    'Drivers/CMSIS/Device/ST/STM32F3xx/Source/Templates/system_stm32f3xx.c',
)
cmsis_inc = include_directories(
    [
        'Drivers/CMSIS/Device/ST/STM32F3xx/Include',
        'Drivers/CMSIS/Include',
    ],
    is_system: true,
)

lldriver_src = files(
    'Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_adc.c',
    'Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_comp.c',
    'Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_crc.c',
    'Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_dac.c',
    'Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_dma.c',
    'Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_exti.c',
    'Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_gpio.c',
    'Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_hrtim.c',
    'Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_i2c.c',
    'Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_opamp.c',
    'Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_pwr.c',
    'Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_rcc.c',
    'Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_rtc.c',
    'Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_spi.c',
    'Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_tim.c',
    'Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_usart.c',
    'Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_utils.c',
)

lldriver_inc = include_directories(
    [
        'Drivers/STM32F3xx_HAL_Driver/Inc',
    ],
    is_system: true,
)

stm32f3_cmsis_lib = static_library(
    'stm32f3_cmsis',
    sources: cmsis_src,
    include_directories: cmsis_inc,
)

stm32f3_cmsis_dep = declare_dependency(
    include_directories: cmsis_inc,
    link_with: stm32f3_cmsis_lib,
)

stm32f3_ll_driver_lib = static_library(
    'stm32f3_ll_driver',
    sources: lldriver_src,
    include_directories: [lldriver_inc, cmsis_inc],
    dependencies: stm32f3_cmsis_dep,
)

stm32f3_ll_driver_dep = declare_dependency(
    include_directories: [lldriver_inc, cmsis_inc],
    link_with: stm32f3_ll_driver_lib,
    compile_args: [
        '-DSTM32F303xC',
        '-DUSE_FULL_LL_DRIVER',
        '-DUSE_WWDG',
    ],
)
