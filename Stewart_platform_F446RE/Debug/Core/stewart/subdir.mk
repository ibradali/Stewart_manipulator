################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/stewart/stewart.c 

OBJS += \
./Core/stewart/stewart.o 

C_DEPS += \
./Core/stewart/stewart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/stewart/%.o Core/stewart/%.su Core/stewart/%.cyclo: ../Core/stewart/%.c Core/stewart/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-stewart

clean-Core-2f-stewart:
	-$(RM) ./Core/stewart/stewart.cyclo ./Core/stewart/stewart.d ./Core/stewart/stewart.o ./Core/stewart/stewart.su

.PHONY: clean-Core-2f-stewart

