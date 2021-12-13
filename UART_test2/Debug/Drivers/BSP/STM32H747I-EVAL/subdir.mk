################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval.c \
../Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_audio.c \
../Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_bus.c \
../Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_eeprom.c \
../Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_io.c \
../Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_lcd.c \
../Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_nor.c \
../Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_qspi.c \
../Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_sd.c \
../Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_sdram.c \
../Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_sram.c \
../Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_ts.c 

OBJS += \
./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval.o \
./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_audio.o \
./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_bus.o \
./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_eeprom.o \
./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_io.o \
./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_lcd.o \
./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_nor.o \
./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_qspi.o \
./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_sd.o \
./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_sdram.o \
./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_sram.o \
./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_ts.o 

C_DEPS += \
./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval.d \
./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_audio.d \
./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_bus.d \
./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_eeprom.d \
./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_io.d \
./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_lcd.d \
./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_nor.d \
./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_qspi.d \
./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_sd.d \
./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_sdram.d \
./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_sram.d \
./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_ts.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/STM32H747I-EVAL/%.o: ../Drivers/BSP/STM32H747I-EVAL/%.c Drivers/BSP/STM32H747I-EVAL/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"../Drivers/BSP/STM32H7xx_Nucleo" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-STM32H747I-2d-EVAL

clean-Drivers-2f-BSP-2f-STM32H747I-2d-EVAL:
	-$(RM) ./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval.d ./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval.o ./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_audio.d ./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_audio.o ./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_bus.d ./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_bus.o ./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_eeprom.d ./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_eeprom.o ./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_io.d ./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_io.o ./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_lcd.d ./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_lcd.o ./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_nor.d ./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_nor.o ./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_qspi.d ./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_qspi.o ./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_sd.d ./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_sd.o ./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_sdram.d ./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_sdram.o ./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_sram.d ./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_sram.o ./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_ts.d ./Drivers/BSP/STM32H747I-EVAL/stm32h747i_eval_ts.o

.PHONY: clean-Drivers-2f-BSP-2f-STM32H747I-2d-EVAL

