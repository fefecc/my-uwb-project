################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../APP/Bsp/EXITcallback.c \
../APP/Bsp/bsp_asm330.c 

OBJS += \
./APP/Bsp/EXITcallback.o \
./APP/Bsp/bsp_asm330.o 

C_DEPS += \
./APP/Bsp/EXITcallback.d \
./APP/Bsp/bsp_asm330.d 


# Each subdirectory must supply rules for building sources it contributes
APP/Bsp/%.o APP/Bsp/%.su APP/Bsp/%.cyclo: ../APP/Bsp/%.c APP/Bsp/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../FATFS/App -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FatFs/src -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/fefe/Desktop/vscodepro/UWB/APP/Bsp" -I"C:/Users/fefe/Desktop/vscodepro/UWB/APP/common" -I"C:/Users/fefe/Desktop/vscodepro/UWB/APP/task" -I"C:/Users/fefe/Desktop/vscodepro/UWB/Thrid/asm330" -I../Core/Inc -I../FATFS/Target -I"C:/Users/fefe/Desktop/vscodepro/UWB/Thrid/decadriver" -I"C:/Users/fefe/Desktop/vscodepro/UWB/APP/UWB" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-APP-2f-Bsp

clean-APP-2f-Bsp:
	-$(RM) ./APP/Bsp/EXITcallback.cyclo ./APP/Bsp/EXITcallback.d ./APP/Bsp/EXITcallback.o ./APP/Bsp/EXITcallback.su ./APP/Bsp/bsp_asm330.cyclo ./APP/Bsp/bsp_asm330.d ./APP/Bsp/bsp_asm330.o ./APP/Bsp/bsp_asm330.su

.PHONY: clean-APP-2f-Bsp

