set(CMAKE_SYSTEM_NAME           "Generic")
set(CMAKE_SYSTEM_PROCESSOR      "arm")
set(CMAKE_CROSSCOMPILING 1)

# Setup search paths
set(CMAKE_FIND_ROOT_PATH "C:/Users/ryans/Tools/GNU-Arm-Embedded-Toolchain/10-2020-q4-major")
set(CMAKE_SYSTEM_PROGRAM_PATH /bin)
set(CMAKE_SYSTEM_LIBRARY_PATH /lib)
set(CMAKE_SYSTEM_INCLUDE_PATH /arm-none-eabi/include)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

set(CMAKE_AR                "arm-none-eabi-ar.exe")
set(CMAKE_ASM_COMPILER      "arm-none-eabi-gcc.exe")
set(CMAKE_C_COMPILER        "arm-none-eabi-gcc.exe")
set(CMAKE_CXX_COMPILER      "arm-none-eabi-g++.exe")
set(CMAKE_LINKER            "arm-none-eabi-ld.exe")
set(CMAKE_OBJCOPY           "arm-none-eabi-objcopy.exe")
set(CMAKE_RANLIB            "arm-none-eabi-ranlib.exe")
set(CMAKE_STRIP             "arm-none-eabi-strip.exe")
set(CMAKE_SIZE              "arm-none-eabi-size.exe")

set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")