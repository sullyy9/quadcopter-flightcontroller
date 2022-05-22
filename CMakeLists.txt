# Set project.
####################################################################################################
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/toolchain.cmake")
cmake_minimum_required(VERSION 3.00)
project(quadcopter-flightcontroller
    VERSION 1.0
    LANGUAGES CXX
    )

set(CMAKE_VERBOSE_MAKEFILE OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set input files.
####################################################################################################
file(GLOB_RECURSE USER_SOURCES  "src/*.cpp")
set(LINKER_SCRIPT               "${CMAKE_SOURCE_DIR}/src/application.ld")

# Set output file names.
####################################################################################################
set(DEVICE_DRIVERS  "device-drivers")
set(PROJECT_ELF     "${PROJECT_NAME}.elf")
set(PROJECT_HEX     "${PROJECT_NAME}.hex")
set(PROJECT_BIN     "${PROJECT_NAME}.bin")
set(PROJECT_MAP     "${PROJECT_NAME}.map")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/release")
set(ASSEMBLY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/assembly")

# Set flags.
####################################################################################################
set(FLAGS_DEVICE            -std=c++20 -mthumb -mcpu=cortex-m4)
set(FLAGS_OPTIMISATIONS     -fdata-sections -ffunction-sections -fno-common -fno-unwind-tables -Os)
set(FLAGS_MISC              -nostartfiles -specs=nano.specs -specs=nosys.specs )
set(FLAGS_WARNINGS          -Wall -Wextra -Wshadow -Wdouble-promotion -Wundef -Wno-volatile)

# Configure the target.
####################################################################################################
add_executable(${PROJECT_ELF} ${USER_SOURCES})

add_subdirectory(lib)

target_compile_options(${PROJECT_ELF}
    PRIVATE ${FLAGS_DEVICE}
    PRIVATE ${FLAGS_OPTIMISATIONS}
    PRIVATE ${FLAGS_MISC}
    PRIVATE ${FLAGS_WARNINGS}
    )

target_link_options(${PROJECT_ELF}
    PRIVATE ${FLAGS_DEVICE}
    PRIVATE ${FLAGS_OPTIMISATIONS}
    PRIVATE ${FLAGS_MISC}
    PRIVATE "LINKER:--gc-sections,-T${LINKER_SCRIPT},-Map=${PROJECT_MAP}"
    )

target_link_libraries(${PROJECT_ELF} PRIVATE device-drivers)
target_link_libraries(${PROJECT_ELF} PRIVATE m)

# Generate assembly for each source file.
####################################################################################################
set(COMPILE_FLAGS -std=c++20 -mthumb -mcpu=cortex-m4 -fdata-sections -ffunction-sections -fno-common -Os -g -w)
set(DEFINE_FLAGS -DSTM32F303xC -DUSE_FULL_LL_DRIVER -DUSE_WWDG)
set(INCLUDE_FLAGS -I${CMAKE_SOURCE_DIR}/lib/cmsis -I${CMAKE_SOURCE_DIR}/lib/ll-drivers)

set(ASM_FLAGS
    ${COMPILE_FLAGS} 
    ${DEFINE_FLAGS}
    ${INCLUDE_FLAGS}
    )

add_custom_command(TARGET ${PROJECT_ELF}
    COMMENT "Building asm targets"
    COMMAND ${CMAKE_COMMAND} -E make_directory ${ASSEMBLY_OUTPUT_DIRECTORY}
    )

foreach(SOURCE_FILE IN LISTS USER_SOURCES)
    get_filename_component(SOURCE_FILE_NAME ${SOURCE_FILE} NAME_WLE)
    set(ASM_FILE "${SOURCE_FILE_NAME}.s")

    add_custom_target(${ASM_FILE} ALL DEPENDS ${PROJECT_ELF})

    add_custom_command(TARGET ${ASM_FILE}
        WORKING_DIRECTORY ${ASSEMBLY_OUTPUT_DIRECTORY}
        COMMAND ${CMAKE_CXX_COMPILER} ${ASM_FLAGS} -S ${SOURCE_FILE} -o ${SOURCE_FILE_NAME}-raw.s
        COMMAND ${CMAKE_CXX_FILT} < ${SOURCE_FILE_NAME}-raw.s > ${SOURCE_FILE_NAME}.s
        COMMAND ${CMAKE_COMMAND} -E remove ${SOURCE_FILE_NAME}-raw.s
        )

endforeach()

# Add bin and hex files.
####################################################################################################
add_custom_target(${PROJECT_HEX} ALL DEPENDS ${PROJECT_ELF})
add_custom_target(${PROJECT_BIN} ALL DEPENDS ${PROJECT_ELF})

add_custom_command(TARGET ${PROJECT_HEX}
        COMMENT "Building hex file"
        WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
        COMMAND ${CMAKE_OBJCOPY} -Oihex ${PROJECT_ELF} ${PROJECT_HEX}
        )

add_custom_command(TARGET ${PROJECT_BIN}
        COMMENT "Building bin file"
        WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
        COMMAND ${CMAKE_OBJCOPY} -Obinary ${PROJECT_ELF} ${PROJECT_BIN}
        )
