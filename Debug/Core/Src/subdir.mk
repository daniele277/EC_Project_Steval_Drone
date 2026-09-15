################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/custom_bus.c \
../Core/Src/lps22hh.c \
../Core/Src/lps22hh_reg.c \
../Core/Src/lsm6dsr.c \
../Core/Src/lsm6dsr_reg.c \
../Core/Src/main.c \
../Core/Src/pid.c \
../Core/Src/sensor_fusion.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

OBJS += \
./Core/Src/custom_bus.o \
./Core/Src/lps22hh.o \
./Core/Src/lps22hh_reg.o \
./Core/Src/lsm6dsr.o \
./Core/Src/lsm6dsr_reg.o \
./Core/Src/main.o \
./Core/Src/pid.o \
./Core/Src/sensor_fusion.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 

C_DEPS += \
./Core/Src/custom_bus.d \
./Core/Src/lps22hh.d \
./Core/Src/lps22hh_reg.d \
./Core/Src/lsm6dsr.d \
./Core/Src/lsm6dsr_reg.d \
./Core/Src/main.d \
./Core/Src/pid.d \
./Core/Src/sensor_fusion.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../BlueNRG_2/Target -I../Middlewares/ST/BlueNRG-2/hci/hci_tl_patterns/Basic -I../Middlewares/ST/BlueNRG-2/utils -I../Middlewares/ST/BlueNRG-2/includes -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/custom_bus.cyclo ./Core/Src/custom_bus.d ./Core/Src/custom_bus.o ./Core/Src/custom_bus.su ./Core/Src/lps22hh.cyclo ./Core/Src/lps22hh.d ./Core/Src/lps22hh.o ./Core/Src/lps22hh.su ./Core/Src/lps22hh_reg.cyclo ./Core/Src/lps22hh_reg.d ./Core/Src/lps22hh_reg.o ./Core/Src/lps22hh_reg.su ./Core/Src/lsm6dsr.cyclo ./Core/Src/lsm6dsr.d ./Core/Src/lsm6dsr.o ./Core/Src/lsm6dsr.su ./Core/Src/lsm6dsr_reg.cyclo ./Core/Src/lsm6dsr_reg.d ./Core/Src/lsm6dsr_reg.o ./Core/Src/lsm6dsr_reg.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/pid.cyclo ./Core/Src/pid.d ./Core/Src/pid.o ./Core/Src/pid.su ./Core/Src/sensor_fusion.cyclo ./Core/Src/sensor_fusion.d ./Core/Src/sensor_fusion.o ./Core/Src/sensor_fusion.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su

.PHONY: clean-Core-2f-Src

