cmake_minimum_required(VERSION 3.20)
project(device-drivers
    VERSION 1.0
    LANGUAGES C
    )

# Set input files.
####################################################################################################
file(GLOB_RECURSE CMSIS_SOURCES "cmsis/*.c")
file(GLOB_RECURSE HAL_SOURCES   "ll-drivers/*.c")

# Set flags.
####################################################################################################
set(FLAGS_DEVICE            -std=c99 -mthumb -mcpu=cortex-m4)
set(FLAGS_OPTIMISATIONS     -fdata-sections -ffunction-sections -fno-common -Os)
set(FLAGS_WARNINGS          -w)
set(FLAGS_DEFINES           STM32F303xC USE_FULL_LL_DRIVER USE_WWDG)

# Configure the target.
####################################################################################################
add_library(${PROJECT_NAME} STATIC
    ${CMSIS_SOURCES}
    ${HAL_SOURCES}
    )   

target_compile_options(${PROJECT_NAME}
    PRIVATE ${FLAGS_DEVICE}
    PRIVATE ${FLAGS_OPTIMISATION}
    PRIVATE ${FLAGS_WARNINGS}
    )

target_compile_definitions(${PROJECT_NAME}
    PUBLIC ${FLAGS_DEFINES}
    )

target_include_directories(${PROJECT_NAME} SYSTEM
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/cmsis"
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/ll-drivers"
    )