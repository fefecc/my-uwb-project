################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../APP/common/DoubleRingBuffer.c \
../APP/common/gnss_parser.c 

OBJS += \
./APP/common/DoubleRingBuffer.o \
./APP/common/gnss_parser.o 

C_DEPS += \
./APP/common/DoubleRingBuffer.d \
./APP/common/gnss_parser.d 


# Each subdirectory must supply rules for building sources it contributes
APP/common/%.o APP/common/%.su APP/common/%.cyclo: ../APP/common/%.c APP/common/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../FATFS/App -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FatFs/src -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/fefe/Desktop/vscodepro/UWB/APP/Bsp" -I"C:/Users/fefe/Desktop/vscodepro/UWB/APP/common" -I"C:/Users/fefe/Desktop/vscodepro/UWB/APP/task" -I"C:/Users/fefe/Desktop/vscodepro/UWB/Thrid/asm330" -I../Core/Inc -I../FATFS/Target -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-APP-2f-common

clean-APP-2f-common:
	-$(RM) ./APP/common/DoubleRingBuffer.cyclo ./APP/common/DoubleRingBuffer.d ./APP/common/DoubleRingBuffer.o ./APP/common/DoubleRingBuffer.su ./APP/common/gnss_parser.cyclo ./APP/common/gnss_parser.d ./APP/common/gnss_parser.o ./APP/common/gnss_parser.su

.PHONY: clean-APP-2f-common

