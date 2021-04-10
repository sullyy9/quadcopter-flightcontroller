cmake_minimum_required(VERSION 3.00)
project(ryansullivan-quadcopter-flightcontroller
    VERSION 1.0
    LANGUAGES C
    )

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type selected, default to Release")
    set(CMAKE_BUILD_TYPE Release)
endif()

# Clear default compile flags.
####################################################################################################
set(CMAKE_C_FLAGS                "")
set(CMAKE_C_FLAGS_DEBUG          "")
set(CMAKE_C_FLAGS_RELEASE        "")
set(CMAKE_C_FLAGS_MINSIZEREL     "")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "")

# Set input files.
####################################################################################################
file(GLOB_RECURSE USER_SOURCES  "src/*.c")
file(GLOB_RECURSE CMSIS_SOURCES "lib/src/cmsis/*.c")
file(GLOB_RECURSE HAL_SOURCES   "lib/src/stm32f3-hal/*.c")
set(LINKER_SCRIPT               "${CMAKE_SOURCE_DIR}/src/linker.ld")

# Set output file names.
####################################################################################################
set(ST_LIB      "stm32f3_ll")
set(PROJECT_ELF "${PROJECT_NAME}.elf")
set(PROJECT_HEX "${PROJECT_NAME}.hex")
set(PROJECT_BIN "${PROJECT_NAME}.bin")

# Configure the peripheral library.
####################################################################################################
add_library(${ST_LIB} STATIC
    ${CMSIS_SOURCES}
    ${HAL_SOURCES}
    )   

target_compile_options(${ST_LIB}
    PRIVATE -mthumb -mcpu=cortex-m4 -fdata-sections -ffunction-sections -fno-common -Os
    PRIVATE -W
    )

target_compile_definitions(${ST_LIB}
    PUBLIC STM32F303xC USE_FULL_LL_DRIVER USE_WWDG
    )

target_include_directories(${ST_LIB}
    PUBLIC "lib/include/cmsis"
    PUBLIC "lib/include/stm32f3-hal"
    )

# Configure the elf file.
####################################################################################################
add_executable(${PROJECT_ELF}
    ${USER_SOURCES}
    )

target_compile_options(${PROJECT_ELF}
    PUBLIC -mthumb -mcpu=cortex-m4 -fdata-sections -ffunction-sections -fno-common -Os
    PRIVATE -Wall -Wextra -Wshadow -Wdouble-promotion -Wundef
    )

target_link_options(${PROJECT_ELF}
    PRIVATE -mthumb -mcpu=cortex-m4 -fdata-sections -ffunction-sections -fno-common -Os
    PRIVATE -nostartfiles
    PRIVATE "LINKER:--gc-sections,-T${LINKER_SCRIPT}"
    )

target_link_libraries(${PROJECT_ELF} ${ST_LIB})
target_link_libraries(${PROJECT_ELF} m)

# Add bin and hex files.
####################################################################################################
add_custom_target(${PROJECT_HEX} ALL DEPENDS ${PROJECT_ELF})
add_custom_target(${PROJECT_BIN} ALL DEPENDS ${PROJECT_ELF})

add_custom_command(TARGET ${PROJECT_HEX}
        COMMENT "Building hex file"
        COMMAND ${CMAKE_OBJCOPY} -Oihex ${PROJECT_ELF} ${PROJECT_HEX}
        )

add_custom_command(TARGET ${PROJECT_BIN}
        COMMENT "Building bin file"
        COMMAND ${CMAKE_OBJCOPY} -Obinary ${PROJECT_ELF} ${PROJECT_BIN}
        )
