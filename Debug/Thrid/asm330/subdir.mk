################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Thrid/asm330/asm330lhh_reg.c 

OBJS += \
./Thrid/asm330/asm330lhh_reg.o 

C_DEPS += \
./Thrid/asm330/asm330lhh_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Thrid/asm330/%.o Thrid/asm330/%.su Thrid/asm330/%.cyclo: ../Thrid/asm330/%.c Thrid/asm330/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../FATFS/App -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FatFs/src -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/fefe/Desktop/vscodepro/UWB/APP/Bsp" -I"C:/Users/fefe/Desktop/vscodepro/UWB/APP/common" -I"C:/Users/fefe/Desktop/vscodepro/UWB/APP/task" -I"C:/Users/fefe/Desktop/vscodepro/UWB/Thrid/asm330" -I../Core/Inc -I../FATFS/Target -I"C:/Users/fefe/Desktop/vscodepro/UWB/Thrid/decadriver" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Thrid-2f-asm330

clean-Thrid-2f-asm330:
	-$(RM) ./Thrid/asm330/asm330lhh_reg.cyclo ./Thrid/asm330/asm330lhh_reg.d ./Thrid/asm330/asm330lhh_reg.o ./Thrid/asm330/asm330lhh_reg.su

.PHONY: clean-Thrid-2f-asm330

