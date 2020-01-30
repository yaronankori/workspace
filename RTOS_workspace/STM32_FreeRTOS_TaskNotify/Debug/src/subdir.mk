################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/main.c \
../src/syscalls.c \
../src/system_stm32f4xx.c 

OBJS += \
./src/main.o \
./src/syscalls.o \
./src/system_stm32f4xx.o 

C_DEPS += \
./src/main.d \
./src/syscalls.d \
./src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -DDEBUG -DSTM32F446xx -DUSE_STDPERIPH_DRIVER -I"C:/Users/User/workspace/RTOS_workspace/STM32_FreeRTOS_TaskNotify/StdPeriph_Driver/inc" -I"C:/Users/User/workspace/RTOS_workspace/STM32_FreeRTOS_TaskNotify/Third-Party/SEGGER/Config" -I"C:/Users/User/workspace/RTOS_workspace/STM32_FreeRTOS_TaskNotify/Third-Party/SEGGER/OS" -I"C:/Users/User/workspace/RTOS_workspace/STM32_FreeRTOS_TaskNotify/Third-Party/SEGGER/SEGGER" -I"C:/Users/User/workspace/RTOS_workspace/STM32_FreeRTOS_TaskNotify/Config" -I"C:/Users/User/workspace/RTOS_workspace/STM32_FreeRTOS_TaskNotify/Third-Party/FreeRTOS/org/Source/include" -I"C:/Users/User/workspace/RTOS_workspace/STM32_FreeRTOS_TaskNotify/Third-Party/FreeRTOS/org/Source/portable/GCC/ARM_CM4F" -I"C:/Users/User/workspace/RTOS_workspace/STM32_FreeRTOS_TaskNotify/inc" -I"C:/Users/User/workspace/RTOS_workspace/STM32_FreeRTOS_TaskNotify/CMSIS/device" -I"C:/Users/User/workspace/RTOS_workspace/STM32_FreeRTOS_TaskNotify/CMSIS/core" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


