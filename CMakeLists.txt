# Set the toolchain.
####################################################################################################
set(CMAKE_SYSTEM_NAME           "Generic")
set(CMAKE_SYSTEM_PROCESSOR      "arm")
set(CMAKE_CROSSCOMPILING 1)

set(CMAKE_AR           "arm-none-eabi-ar")
set(CMAKE_ASM_COMPILER "arm-none-eabi-gcc")
set(CMAKE_C_COMPILER   "arm-none-eabi-gcc")
set(CMAKE_CXX_COMPILER "arm-none-eabi-g++")
set(CMAKE_LINKER       "arm-none-eabi-ld")
set(CMAKE_OBJCOPY      "arm-none-eabi-objcopy")
set(CMAKE_RANLIB       "arm-none-eabi-ranlib")
set(CMAKE_STRIP        "arm-none-eabi-strip")
set(CMAKE_SIZE         "arm-none-eabi-size")
set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")

# Set project.
####################################################################################################
cmake_minimum_required(VERSION 3.00)
project(quadcopter-flightcontroller
    VERSION 1.0
    LANGUAGES C CXX
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

set(CMAKE_CXX_FLAGS                "")
set(CMAKE_CXX_FLAGS_DEBUG          "")
set(CMAKE_CXX_FLAGS_RELEASE        "")
set(CMAKE_CXX_FLAGS_MINSIZEREL     "")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "")

# Set input files.
####################################################################################################
file(GLOB_RECURSE USER_SOURCES  "src/*.cpp")
file(GLOB_RECURSE CMSIS_SOURCES "lib/cmsis/*.c")
file(GLOB_RECURSE HAL_SOURCES   "lib/ll-drivers/*.c")
set(LINKER_SCRIPT               "${CMAKE_SOURCE_DIR}/src/application.ld")

# Set output file names.
####################################################################################################
set(DEVICE_DRIVERS  "device-drivers")
set(PROJECT_ELF     "${PROJECT_NAME}.elf")
set(PROJECT_HEX     "${PROJECT_NAME}.hex")
set(PROJECT_BIN     "${PROJECT_NAME}.bin")
set(PROJECT_MAP     "${PROJECT_NAME}.map")

# Configure the peripheral library.
####################################################################################################
add_library(${DEVICE_DRIVERS} STATIC
    ${CMSIS_SOURCES}
    ${HAL_SOURCES}
    )   

target_compile_options(${DEVICE_DRIVERS}
    PRIVATE -std=c99 -mthumb -mcpu=cortex-m4 -fdata-sections -ffunction-sections -fno-common -Os
    PRIVATE -W
    )

target_compile_definitions(${DEVICE_DRIVERS}
    PUBLIC STM32F303xC USE_FULL_LL_DRIVER USE_WWDG
    )

target_include_directories(${DEVICE_DRIVERS}
    PUBLIC "lib/cmsis"
    PUBLIC "lib/ll-drivers"
    )

# Configure the elf file.
####################################################################################################
add_executable(${PROJECT_ELF}
    ${USER_SOURCES}
    )

target_compile_options(${PROJECT_ELF}
    PUBLIC -std=c++20 -mthumb -mcpu=cortex-m4 -fdata-sections -ffunction-sections -fno-common -Os
    PRIVATE -Wall -Wextra -Wshadow -Wdouble-promotion -Wundef -Wno-volatile
    )
target_link_options(${PROJECT_ELF}
    PRIVATE -std=c++20 -mthumb -mcpu=cortex-m4 -fdata-sections -ffunction-sections -fno-common -Os
    PRIVATE -nostartfiles -specs=nosys.specs -fno-unwind-tables
    PRIVATE "LINKER:--gc-sections,-T${LINKER_SCRIPT},-Map=${PROJECT_MAP}"
    )

target_link_libraries(${PROJECT_ELF} PRIVATE ${DEVICE_DRIVERS})
target_link_libraries(${PROJECT_ELF} PRIVATE m)

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
