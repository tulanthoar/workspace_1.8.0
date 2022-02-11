################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/is42s32800g/is42s32800g.c 

OBJS += \
./Drivers/BSP/Components/is42s32800g/is42s32800g.o 

C_DEPS += \
./Drivers/BSP/Components/is42s32800g/is42s32800g.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/is42s32800g/%.o: ../Drivers/BSP/Components/is42s32800g/%.c Drivers/BSP/Components/is42s32800g/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"../Drivers/BSP/STM32H7xx_Nucleo" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-is42s32800g

clean-Drivers-2f-BSP-2f-Components-2f-is42s32800g:
	-$(RM) ./Drivers/BSP/Components/is42s32800g/is42s32800g.d ./Drivers/BSP/Components/is42s32800g/is42s32800g.o

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-is42s32800g

