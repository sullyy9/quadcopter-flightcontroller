cmake_minimum_required(VERSION 3.17)
set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")
project(ryansullivan_quadcopter_flightcontroller C)

set(CMAKE_C_STANDARD 99)

add_compile_definitions(STM32F303xC)
add_compile_definitions(USE_FULL_LL_DRIVER)
add_compile_definitions(USE_WWDG)

if(${CMAKE_BUILD_TYPE} STREQUAL "Release")
    add_compile_definitions(NDEBUG)
    set(CMAKE_C_FLAGS
            "-mcpu=cortex-m4 \
            -mthumb \
            -Os \
            -fmessage-length=0 \
            -fsigned-char \
            -ffunction-sections \
            -fdata-sections \
            -ffreestanding \
            -Wall \
            -Wextra")

    set(LINKER_FLAGS "-nostartfiles -Wl,-Map=${CMAKE_SOURCE_DIR}/build/release/${PROJECT_NAME}.map --specs=nano.specs --specs=nosys.specs")

elseif(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
    add_compile_definitions(DEBUG)
    add_compile_definitions(TRACE)
    add_compile_definitions(OS_USE_TRACE_SEMIHOSTING_DEBUG)
    set(CMAKE_C_FLAGS
            "-mcpu=cortex-m4 \
            -mthumb \
            -Og \
            -g3 \
            -fmessage-length=0 \
            -fsigned-char \
            -ffunction-sections \
            -fdata-sections \
            -ffreestanding \
            -Wall \
            -Wextra")

    set(LINKER_FLAGS "-Xlinker --gc-sections -Wl,-Map=${CMAKE_SOURCE_DIR}/build/release/${PROJECT_NAME}.map")
endif()

file(GLOB_RECURSE USER_SOURCES "src/*.c")
file(GLOB_RECURSE CMSIS_SOURCES "lib/src/cmsis/*.c")
file(GLOB_RECURSE HAL_SOURCES "lib/src/stm32f3-hal/*.c")
set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/src/linker.ld")

set(CMAKE_EXE_LINKER_FLAGS "${LINKER_FLAGS} -T${LINKER_SCRIPT}")

include_directories(lib/include/stm32f3-hal lib/include/cmsis)

add_executable(${PROJECT_NAME}.elf
        ${USER_SOURCES}
        ${CMSIS_SOURCES}
        ${HAL_SOURCES}
        ${LINKER_SCRIPT}
        )

target_link_libraries(${PROJECT_NAME}.elf m)

set(BIN_FILE ${PROJECT_SOURCE_DIR}/build/release/${PROJECT_NAME}.bin)
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Obinary ${PROJECT_SOURCE_DIR}/build/release/${PROJECT_NAME}.elf ${BIN_FILE}
        COMMENT "Building ${BIN_FILE}"
        )
