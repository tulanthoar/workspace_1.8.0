################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/exc7200/exc7200.c \
../Drivers/BSP/Components/exc7200/exc7200_reg.c 

OBJS += \
./Drivers/BSP/Components/exc7200/exc7200.o \
./Drivers/BSP/Components/exc7200/exc7200_reg.o 

C_DEPS += \
./Drivers/BSP/Components/exc7200/exc7200.d \
./Drivers/BSP/Components/exc7200/exc7200_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/exc7200/%.o: ../Drivers/BSP/Components/exc7200/%.c Drivers/BSP/Components/exc7200/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"../Drivers/BSP/STM32H7xx_Nucleo" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-exc7200

clean-Drivers-2f-BSP-2f-Components-2f-exc7200:
	-$(RM) ./Drivers/BSP/Components/exc7200/exc7200.d ./Drivers/BSP/Components/exc7200/exc7200.o ./Drivers/BSP/Components/exc7200/exc7200_reg.d ./Drivers/BSP/Components/exc7200/exc7200_reg.o

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-exc7200

