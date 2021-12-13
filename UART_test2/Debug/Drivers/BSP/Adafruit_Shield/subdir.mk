################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Adafruit_Shield/adafruit_802.c \
../Drivers/BSP/Adafruit_Shield/adafruit_802_lcd.c \
../Drivers/BSP/Adafruit_Shield/adafruit_802_sd.c 

OBJS += \
./Drivers/BSP/Adafruit_Shield/adafruit_802.o \
./Drivers/BSP/Adafruit_Shield/adafruit_802_lcd.o \
./Drivers/BSP/Adafruit_Shield/adafruit_802_sd.o 

C_DEPS += \
./Drivers/BSP/Adafruit_Shield/adafruit_802.d \
./Drivers/BSP/Adafruit_Shield/adafruit_802_lcd.d \
./Drivers/BSP/Adafruit_Shield/adafruit_802_sd.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Adafruit_Shield/%.o: ../Drivers/BSP/Adafruit_Shield/%.c Drivers/BSP/Adafruit_Shield/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"../Drivers/BSP/STM32H7xx_Nucleo" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Adafruit_Shield

clean-Drivers-2f-BSP-2f-Adafruit_Shield:
	-$(RM) ./Drivers/BSP/Adafruit_Shield/adafruit_802.d ./Drivers/BSP/Adafruit_Shield/adafruit_802.o ./Drivers/BSP/Adafruit_Shield/adafruit_802_lcd.d ./Drivers/BSP/Adafruit_Shield/adafruit_802_lcd.o ./Drivers/BSP/Adafruit_Shield/adafruit_802_sd.d ./Drivers/BSP/Adafruit_Shield/adafruit_802_sd.o

.PHONY: clean-Drivers-2f-BSP-2f-Adafruit_Shield

