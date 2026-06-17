################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/W25Q_STM32_HAL_Driver/Src/w25q_ospi.c \
../Drivers/W25Q_STM32_HAL_Driver/Src/w25q_qspi.c 

OBJS += \
./Drivers/W25Q_STM32_HAL_Driver/Src/w25q_ospi.o \
./Drivers/W25Q_STM32_HAL_Driver/Src/w25q_qspi.o 

C_DEPS += \
./Drivers/W25Q_STM32_HAL_Driver/Src/w25q_ospi.d \
./Drivers/W25Q_STM32_HAL_Driver/Src/w25q_qspi.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/W25Q_STM32_HAL_Driver/Src/%.o Drivers/W25Q_STM32_HAL_Driver/Src/%.su Drivers/W25Q_STM32_HAL_Driver/Src/%.cyclo: ../Drivers/W25Q_STM32_HAL_Driver/Src/%.c Drivers/W25Q_STM32_HAL_Driver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM7 -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H730xx -c -I../Core/Inc -I../Drivers/W25Q_STM32_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-W25Q_STM32_HAL_Driver-2f-Src

clean-Drivers-2f-W25Q_STM32_HAL_Driver-2f-Src:
	-$(RM) ./Drivers/W25Q_STM32_HAL_Driver/Src/w25q_ospi.cyclo ./Drivers/W25Q_STM32_HAL_Driver/Src/w25q_ospi.d ./Drivers/W25Q_STM32_HAL_Driver/Src/w25q_ospi.o ./Drivers/W25Q_STM32_HAL_Driver/Src/w25q_ospi.su ./Drivers/W25Q_STM32_HAL_Driver/Src/w25q_qspi.cyclo ./Drivers/W25Q_STM32_HAL_Driver/Src/w25q_qspi.d ./Drivers/W25Q_STM32_HAL_Driver/Src/w25q_qspi.o ./Drivers/W25Q_STM32_HAL_Driver/Src/w25q_qspi.su

.PHONY: clean-Drivers-2f-W25Q_STM32_HAL_Driver-2f-Src

