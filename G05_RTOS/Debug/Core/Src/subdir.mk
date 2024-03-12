################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/BME280_STM32.c \
../Core/Src/GPS.c \
../Core/Src/as5600.c \
../Core/Src/bno055.c \
../Core/Src/freertos.c \
../Core/Src/kom.c \
../Core/Src/main.c \
../Core/Src/pid.c \
../Core/Src/servo.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_hal_timebase_tim.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/tm_stm32f4_bkpsram.c \
../Core/Src/user_task.c 

OBJS += \
./Core/Src/BME280_STM32.o \
./Core/Src/GPS.o \
./Core/Src/as5600.o \
./Core/Src/bno055.o \
./Core/Src/freertos.o \
./Core/Src/kom.o \
./Core/Src/main.o \
./Core/Src/pid.o \
./Core/Src/servo.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_hal_timebase_tim.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/tm_stm32f4_bkpsram.o \
./Core/Src/user_task.o 

C_DEPS += \
./Core/Src/BME280_STM32.d \
./Core/Src/GPS.d \
./Core/Src/as5600.d \
./Core/Src/bno055.d \
./Core/Src/freertos.d \
./Core/Src/kom.d \
./Core/Src/main.d \
./Core/Src/pid.d \
./Core/Src/servo.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_hal_timebase_tim.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/tm_stm32f4_bkpsram.d \
./Core/Src/user_task.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FatFs/src -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/BME280_STM32.cyclo ./Core/Src/BME280_STM32.d ./Core/Src/BME280_STM32.o ./Core/Src/BME280_STM32.su ./Core/Src/GPS.cyclo ./Core/Src/GPS.d ./Core/Src/GPS.o ./Core/Src/GPS.su ./Core/Src/as5600.cyclo ./Core/Src/as5600.d ./Core/Src/as5600.o ./Core/Src/as5600.su ./Core/Src/bno055.cyclo ./Core/Src/bno055.d ./Core/Src/bno055.o ./Core/Src/bno055.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/kom.cyclo ./Core/Src/kom.d ./Core/Src/kom.o ./Core/Src/kom.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/pid.cyclo ./Core/Src/pid.d ./Core/Src/pid.o ./Core/Src/pid.su ./Core/Src/servo.cyclo ./Core/Src/servo.d ./Core/Src/servo.o ./Core/Src/servo.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_hal_timebase_tim.cyclo ./Core/Src/stm32f4xx_hal_timebase_tim.d ./Core/Src/stm32f4xx_hal_timebase_tim.o ./Core/Src/stm32f4xx_hal_timebase_tim.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/tm_stm32f4_bkpsram.cyclo ./Core/Src/tm_stm32f4_bkpsram.d ./Core/Src/tm_stm32f4_bkpsram.o ./Core/Src/tm_stm32f4_bkpsram.su ./Core/Src/user_task.cyclo ./Core/Src/user_task.d ./Core/Src/user_task.o ./Core/Src/user_task.su

.PHONY: clean-Core-2f-Src

