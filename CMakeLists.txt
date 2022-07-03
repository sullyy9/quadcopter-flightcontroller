# Set project.
####################################################################################################
cmake_minimum_required(VERSION 3.20)
project(quadcopter-flightcontroller
    VERSION 1.0
    LANGUAGES CXX
    )

set(CMAKE_VERBOSE_MAKEFILE OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/release")
set(ASSEMBLY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/assembly")

add_subdirectory(lib)

# Configure the target.
####################################################################################################
add_executable(${PROJECT_NAME})

set_target_properties(${PROJECT_NAME}
    PROPERTIES 
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED YES
        CXX_EXTENSIONS OFF
        SUFFIX .elf
    )

target_sources(${PROJECT_NAME}
    PRIVATE
        src/clocks.cpp
        src/debug.cpp
        src/i2c.cpp
        src/io.cpp
        src/main.cpp
        src/port.cpp
        src/spi.cpp
        src/startup.cpp
        src/usart.cpp
        src/utils.cpp
        src/watchdog.cpp
    )

target_include_directories(${PROJECT_NAME}
    PRIVATE
        src/
        )

target_compile_options(${PROJECT_NAME}
    PRIVATE
        -mthumb
        -mcpu=cortex-m4
        -nostartfiles
        -specs=nano.specs
        -specs=nosys.specs
        -fdata-sections
        -ffunction-sections
        -fno-common
        -fno-unwind-tables
        -Os
        -Wall
        -Wextra
        -Wundef
        -Wshadow
        -Wconversion
        -Wno-volatile
        -Wsign-conversion
        -Wdouble-promotion
    )

target_link_options(${PROJECT_NAME}
    PRIVATE
        -mthumb
        -mcpu=cortex-m4
        -nostartfiles
        -specs=nano.specs
        -specs=nosys.specs
        -T${CMAKE_SOURCE_DIR}/src/application.ld
        LINKER:--gc-sections
        LINKER:--print-memory-usage
        LINKER:-Map=${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${PROJECT_NAME}.map
    )

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        device-drivers
        m
    )


# Generate assembly for each source file.
####################################################################################################
# set(COMPILE_FLAGS -std=c++20 -mthumb -mcpu=cortex-m4 -fdata-sections -ffunction-sections -fno-common -Os -g -w)
# set(DEFINE_FLAGS -DSTM32F303xC -DUSE_FULL_LL_DRIVER -DUSE_WWDG)
# set(INCLUDE_FLAGS -I${CMAKE_SOURCE_DIR}/lib/cmsis -I${CMAKE_SOURCE_DIR}/lib/ll-drivers)

# set(ASM_FLAGS
#     ${COMPILE_FLAGS} 
#     ${DEFINE_FLAGS}
#     ${INCLUDE_FLAGS}
#     )

# add_custom_command(TARGET ${PROJECT_NAME}
#     COMMENT "Building asm targets"
#     COMMAND ${CMAKE_COMMAND} -E make_directory ${ASSEMBLY_OUTPUT_DIRECTORY}
#     )

# foreach(SOURCE_FILE IN LISTS USER_SOURCES)
#     get_filename_component(SOURCE_FILE_NAME ${SOURCE_FILE} NAME_WLE)
#     set(ASM_FILE "${SOURCE_FILE_NAME}.s")

#     add_custom_target(${ASM_FILE} ALL DEPENDS ${PROJECT_NAME}.elf)

#     add_custom_command(TARGET ${ASM_FILE}
#         WORKING_DIRECTORY ${ASSEMBLY_OUTPUT_DIRECTORY}
#         COMMAND ${CMAKE_CXX_COMPILER} ${ASM_FLAGS} -S ${SOURCE_FILE} -o ${SOURCE_FILE_NAME}-raw.s
#         COMMAND ${CMAKE_CXX_FILT} < ${SOURCE_FILE_NAME}-raw.s > ${SOURCE_FILE_NAME}.s
#         COMMAND ${CMAKE_COMMAND} -E remove ${SOURCE_FILE_NAME}-raw.s
#         )

# endforeach()

# Add bin and hex files.
####################################################################################################
add_custom_target(${PROJECT_NAME}.hex ALL DEPENDS ${PROJECT_NAME})
add_custom_target(${PROJECT_NAME}.bin ALL DEPENDS ${PROJECT_NAME})

add_custom_command(TARGET ${PROJECT_NAME}.hex
        COMMENT "Building hex file"
        WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
        COMMAND ${CMAKE_OBJCOPY} -Oihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
        )

add_custom_command(TARGET ${PROJECT_NAME}.bin
        COMMENT "Building bin file"
        WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
        COMMAND ${CMAKE_OBJCOPY} -Obinary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
        )
