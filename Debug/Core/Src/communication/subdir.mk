################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/communication/wifi.c 

OBJS += \
./Core/Src/communication/wifi.o 

C_DEPS += \
./Core/Src/communication/wifi.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/communication/%.o Core/Src/communication/%.su Core/Src/communication/%.cyclo: ../Core/Src/communication/%.c Core/Src/communication/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-communication

clean-Core-2f-Src-2f-communication:
	-$(RM) ./Core/Src/communication/wifi.cyclo ./Core/Src/communication/wifi.d ./Core/Src/communication/wifi.o ./Core/Src/communication/wifi.su

.PHONY: clean-Core-2f-Src-2f-communication

