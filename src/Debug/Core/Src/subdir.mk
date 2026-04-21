################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/MadgwickAHRS.c \
../Core/Src/flash.c \
../Core/Src/iis2mdc_reg.c \
../Core/Src/lps22df_reg.c \
../Core/Src/lsm6dsv80x_reg.c \
../Core/Src/main.c \
../Core/Src/sensors.c \
../Core/Src/state_estimation.c \
../Core/Src/stm32h7xx_hal_msp.c \
../Core/Src/stm32h7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32h7xx.c 

OBJS += \
./Core/Src/MadgwickAHRS.o \
./Core/Src/flash.o \
./Core/Src/iis2mdc_reg.o \
./Core/Src/lps22df_reg.o \
./Core/Src/lsm6dsv80x_reg.o \
./Core/Src/main.o \
./Core/Src/sensors.o \
./Core/Src/state_estimation.o \
./Core/Src/stm32h7xx_hal_msp.o \
./Core/Src/stm32h7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32h7xx.o 

C_DEPS += \
./Core/Src/MadgwickAHRS.d \
./Core/Src/flash.d \
./Core/Src/iis2mdc_reg.d \
./Core/Src/lps22df_reg.d \
./Core/Src/lsm6dsv80x_reg.d \
./Core/Src/main.d \
./Core/Src/sensors.d \
./Core/Src/state_estimation.d \
./Core/Src/stm32h7xx_hal_msp.d \
./Core/Src/stm32h7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32h7xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM7 -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H730xx -c -I../Core/Inc -I../Drivers/CMSIS/DSP/Include -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/MadgwickAHRS.cyclo ./Core/Src/MadgwickAHRS.d ./Core/Src/MadgwickAHRS.o ./Core/Src/MadgwickAHRS.su ./Core/Src/flash.cyclo ./Core/Src/flash.d ./Core/Src/flash.o ./Core/Src/flash.su ./Core/Src/iis2mdc_reg.cyclo ./Core/Src/iis2mdc_reg.d ./Core/Src/iis2mdc_reg.o ./Core/Src/iis2mdc_reg.su ./Core/Src/lps22df_reg.cyclo ./Core/Src/lps22df_reg.d ./Core/Src/lps22df_reg.o ./Core/Src/lps22df_reg.su ./Core/Src/lsm6dsv80x_reg.cyclo ./Core/Src/lsm6dsv80x_reg.d ./Core/Src/lsm6dsv80x_reg.o ./Core/Src/lsm6dsv80x_reg.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/sensors.cyclo ./Core/Src/sensors.d ./Core/Src/sensors.o ./Core/Src/sensors.su ./Core/Src/state_estimation.cyclo ./Core/Src/state_estimation.d ./Core/Src/state_estimation.o ./Core/Src/state_estimation.su ./Core/Src/stm32h7xx_hal_msp.cyclo ./Core/Src/stm32h7xx_hal_msp.d ./Core/Src/stm32h7xx_hal_msp.o ./Core/Src/stm32h7xx_hal_msp.su ./Core/Src/stm32h7xx_it.cyclo ./Core/Src/stm32h7xx_it.d ./Core/Src/stm32h7xx_it.o ./Core/Src/stm32h7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32h7xx.cyclo ./Core/Src/system_stm32h7xx.d ./Core/Src/system_stm32h7xx.o ./Core/Src/system_stm32h7xx.su

.PHONY: clean-Core-2f-Src

