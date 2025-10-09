################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../APP/task/DW1000samplingtask.c \
../APP/task/SDcardtask.c \
../APP/task/UM960samplingtask.c \
../APP/task/imudatadealtask.c \
../APP/task/imusamplingtask.c \
../APP/task/timestamptask.c 

OBJS += \
./APP/task/DW1000samplingtask.o \
./APP/task/SDcardtask.o \
./APP/task/UM960samplingtask.o \
./APP/task/imudatadealtask.o \
./APP/task/imusamplingtask.o \
./APP/task/timestamptask.o 

C_DEPS += \
./APP/task/DW1000samplingtask.d \
./APP/task/SDcardtask.d \
./APP/task/UM960samplingtask.d \
./APP/task/imudatadealtask.d \
./APP/task/imusamplingtask.d \
./APP/task/timestamptask.d 


# Each subdirectory must supply rules for building sources it contributes
APP/task/%.o APP/task/%.su APP/task/%.cyclo: ../APP/task/%.c APP/task/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../FATFS/App -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FatFs/src -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/fefe/Desktop/vscodepro/UWB/APP/Bsp" -I"C:/Users/fefe/Desktop/vscodepro/UWB/APP/common" -I"C:/Users/fefe/Desktop/vscodepro/UWB/APP/task" -I"C:/Users/fefe/Desktop/vscodepro/UWB/Thrid/asm330" -I../Core/Inc -I../FATFS/Target -I"C:/Users/fefe/Desktop/vscodepro/UWB/Thrid/decadriver" -I"C:/Users/fefe/Desktop/vscodepro/UWB/APP/UWB" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-APP-2f-task

clean-APP-2f-task:
	-$(RM) ./APP/task/DW1000samplingtask.cyclo ./APP/task/DW1000samplingtask.d ./APP/task/DW1000samplingtask.o ./APP/task/DW1000samplingtask.su ./APP/task/SDcardtask.cyclo ./APP/task/SDcardtask.d ./APP/task/SDcardtask.o ./APP/task/SDcardtask.su ./APP/task/UM960samplingtask.cyclo ./APP/task/UM960samplingtask.d ./APP/task/UM960samplingtask.o ./APP/task/UM960samplingtask.su ./APP/task/imudatadealtask.cyclo ./APP/task/imudatadealtask.d ./APP/task/imudatadealtask.o ./APP/task/imudatadealtask.su ./APP/task/imusamplingtask.cyclo ./APP/task/imusamplingtask.d ./APP/task/imusamplingtask.o ./APP/task/imusamplingtask.su ./APP/task/timestamptask.cyclo ./APP/task/timestamptask.d ./APP/task/timestamptask.o ./APP/task/timestamptask.su

.PHONY: clean-APP-2f-task

