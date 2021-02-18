################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/commonio.c \
../src/io.c \
../src/main.c \
../src/startup.c 

OBJS += \
./src/commonio.o \
./src/io.o \
./src/main.o \
./src/startup.o 

C_DEPS += \
./src/commonio.d \
./src/io.d \
./src/main.d \
./src/startup.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU Arm Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -Wall -Wextra  -g -DNDEBUG -DSTM32F303xC -DUSE_FULL_LL_DRIVER -DHSE_VALUE=8000000 -I"../STlib/include/stm32f3-hal" -I"../STlib/include" -I"../STlib/include/cortexm" -I"../STlib/include/cmsis" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


