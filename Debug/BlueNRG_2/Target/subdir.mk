################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BlueNRG_2/Target/hci_tl_interface.c 

OBJS += \
./BlueNRG_2/Target/hci_tl_interface.o 

C_DEPS += \
./BlueNRG_2/Target/hci_tl_interface.d 


# Each subdirectory must supply rules for building sources it contributes
BlueNRG_2/Target/%.o BlueNRG_2/Target/%.su BlueNRG_2/Target/%.cyclo: ../BlueNRG_2/Target/%.c BlueNRG_2/Target/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../BlueNRG_2/Target -I../Middlewares/ST/BlueNRG-2/hci/hci_tl_patterns/Basic -I../Middlewares/ST/BlueNRG-2/utils -I../Middlewares/ST/BlueNRG-2/includes -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-BlueNRG_2-2f-Target

clean-BlueNRG_2-2f-Target:
	-$(RM) ./BlueNRG_2/Target/hci_tl_interface.cyclo ./BlueNRG_2/Target/hci_tl_interface.d ./BlueNRG_2/Target/hci_tl_interface.o ./BlueNRG_2/Target/hci_tl_interface.su

.PHONY: clean-BlueNRG_2-2f-Target

