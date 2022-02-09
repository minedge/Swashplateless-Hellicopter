################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Controller/CyclicPitchControl.c \
../Core/Src/Controller/PID.c 

OBJS += \
./Core/Src/Controller/CyclicPitchControl.o \
./Core/Src/Controller/PID.o 

C_DEPS += \
./Core/Src/Controller/CyclicPitchControl.d \
./Core/Src/Controller/PID.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Controller/%.o: ../Core/Src/Controller/%.c Core/Src/Controller/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Controller

clean-Core-2f-Src-2f-Controller:
	-$(RM) ./Core/Src/Controller/CyclicPitchControl.d ./Core/Src/Controller/CyclicPitchControl.o ./Core/Src/Controller/PID.d ./Core/Src/Controller/PID.o

.PHONY: clean-Core-2f-Src-2f-Controller

