set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

# Without that flag CMake is not able to pass test compilation check
set(CMAKE_TRY_COMPILE_TARGET_TYPE   STATIC_LIBRARY)

set(TOOLCHAIN_PATH "C:/Program Files (x86)/GNU Arm Embedded Toolchain/10 2020-q4-major/bin/")
set(CMAKE_AR                        ${TOOLCHAIN_PATH}arm-none-eabi-ar.exe)
set(CMAKE_ASM_COMPILER              ${TOOLCHAIN_PATH}arm-none-eabi-gcc.exe)
set(CMAKE_C_COMPILER                ${TOOLCHAIN_PATH}arm-none-eabi-gcc.exe)
set(CMAKE_CXX_COMPILER              ${TOOLCHAIN_PATH}arm-none-eabi-g++.exe)
set(CMAKE_LINKER                    ${TOOLCHAIN_PATH}arm-none-eabi-ld.exe)
set(CMAKE_OBJCOPY                   ${TOOLCHAIN_PATH}arm-none-eabi-objcopy.exe CACHE INTERNAL "")
set(CMAKE_RANLIB                    ${TOOLCHAIN_PATH}arm-none-eabi-ranlib.exe CACHE INTERNAL "")
set(CMAKE_SIZE                      ${TOOLCHAIN_PATH}arm-none-eabi-size.exe CACHE INTERNAL "")
set(CMAKE_STRIP                     ${TOOLCHAIN_PATH}arm-none-eabi-strip.exe CACHE INTERNAL "")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)