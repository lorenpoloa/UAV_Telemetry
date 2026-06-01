################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/sensors/gps.c \
../Core/Src/sensors/sensor10DoF.c 

OBJS += \
./Core/Src/sensors/gps.o \
./Core/Src/sensors/sensor10DoF.o 

C_DEPS += \
./Core/Src/sensors/gps.d \
./Core/Src/sensors/sensor10DoF.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/sensors/%.o Core/Src/sensors/%.su Core/Src/sensors/%.cyclo: ../Core/Src/sensors/%.c Core/Src/sensors/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-sensors

clean-Core-2f-Src-2f-sensors:
	-$(RM) ./Core/Src/sensors/gps.cyclo ./Core/Src/sensors/gps.d ./Core/Src/sensors/gps.o ./Core/Src/sensors/gps.su ./Core/Src/sensors/sensor10DoF.cyclo ./Core/Src/sensors/sensor10DoF.d ./Core/Src/sensors/sensor10DoF.o ./Core/Src/sensors/sensor10DoF.su

.PHONY: clean-Core-2f-Src-2f-sensors

