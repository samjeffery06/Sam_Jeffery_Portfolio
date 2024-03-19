################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/shared_lib/CAN.c 

OBJS += \
./Core/shared_lib/CAN.o 

C_DEPS += \
./Core/shared_lib/CAN.d 


# Each subdirectory must supply rules for building sources it contributes
Core/shared_lib/%.o Core/shared_lib/%.su: ../Core/shared_lib/%.c Core/shared_lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F412Cx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-shared_lib

clean-Core-2f-shared_lib:
	-$(RM) ./Core/shared_lib/CAN.d ./Core/shared_lib/CAN.o ./Core/shared_lib/CAN.su

.PHONY: clean-Core-2f-shared_lib

