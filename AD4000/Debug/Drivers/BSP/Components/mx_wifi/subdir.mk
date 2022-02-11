################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/mx_wifi/CheckSumUtils.c \
../Drivers/BSP/Components/mx_wifi/mx_wifi.c \
../Drivers/BSP/Components/mx_wifi/mx_wifi_hci.c \
../Drivers/BSP/Components/mx_wifi/mx_wifi_ipc.c \
../Drivers/BSP/Components/mx_wifi/mx_wifi_slip.c 

OBJS += \
./Drivers/BSP/Components/mx_wifi/CheckSumUtils.o \
./Drivers/BSP/Components/mx_wifi/mx_wifi.o \
./Drivers/BSP/Components/mx_wifi/mx_wifi_hci.o \
./Drivers/BSP/Components/mx_wifi/mx_wifi_ipc.o \
./Drivers/BSP/Components/mx_wifi/mx_wifi_slip.o 

C_DEPS += \
./Drivers/BSP/Components/mx_wifi/CheckSumUtils.d \
./Drivers/BSP/Components/mx_wifi/mx_wifi.d \
./Drivers/BSP/Components/mx_wifi/mx_wifi_hci.d \
./Drivers/BSP/Components/mx_wifi/mx_wifi_ipc.d \
./Drivers/BSP/Components/mx_wifi/mx_wifi_slip.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/mx_wifi/%.o: ../Drivers/BSP/Components/mx_wifi/%.c Drivers/BSP/Components/mx_wifi/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"../Drivers/BSP/STM32H7xx_Nucleo" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-mx_wifi

clean-Drivers-2f-BSP-2f-Components-2f-mx_wifi:
	-$(RM) ./Drivers/BSP/Components/mx_wifi/CheckSumUtils.d ./Drivers/BSP/Components/mx_wifi/CheckSumUtils.o ./Drivers/BSP/Components/mx_wifi/mx_wifi.d ./Drivers/BSP/Components/mx_wifi/mx_wifi.o ./Drivers/BSP/Components/mx_wifi/mx_wifi_hci.d ./Drivers/BSP/Components/mx_wifi/mx_wifi_hci.o ./Drivers/BSP/Components/mx_wifi/mx_wifi_ipc.d ./Drivers/BSP/Components/mx_wifi/mx_wifi_ipc.o ./Drivers/BSP/Components/mx_wifi/mx_wifi_slip.d ./Drivers/BSP/Components/mx_wifi/mx_wifi_slip.o

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-mx_wifi

