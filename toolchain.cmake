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
set(CMAKE_CXX_FILT     "arm-none-eabi-c++filt")
set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")