################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/MCP4725.c \
../Core/Src/PIDrestrictions.c \
../Core/Src/SwitchSpeed.c \
../Core/Src/ThrottleADC.c \
../Core/Src/acs758.c \
../Core/Src/adc.c \
../Core/Src/battery.c \
../Core/Src/buttons.c \
../Core/Src/ds18b20.c \
../Core/Src/gpio.c \
../Core/Src/gps_neo6.c \
../Core/Src/hall.c \
../Core/Src/i2c.c \
../Core/Src/iwdg.c \
../Core/Src/lm35.c \
../Core/Src/main.c \
../Core/Src/ntc10k.c \
../Core/Src/ntc10kS.c \
../Core/Src/onewire.c \
../Core/Src/page.c \
../Core/Src/pid.c \
../Core/Src/pidspeed.c \
../Core/Src/pwm.c \
../Core/Src/ssd1306.c \
../Core/Src/ssd1306_fonts.c \
../Core/Src/ssd1306_tests.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/tim.c \
../Core/Src/usart.c 

OBJS += \
./Core/Src/MCP4725.o \
./Core/Src/PIDrestrictions.o \
./Core/Src/SwitchSpeed.o \
./Core/Src/ThrottleADC.o \
./Core/Src/acs758.o \
./Core/Src/adc.o \
./Core/Src/battery.o \
./Core/Src/buttons.o \
./Core/Src/ds18b20.o \
./Core/Src/gpio.o \
./Core/Src/gps_neo6.o \
./Core/Src/hall.o \
./Core/Src/i2c.o \
./Core/Src/iwdg.o \
./Core/Src/lm35.o \
./Core/Src/main.o \
./Core/Src/ntc10k.o \
./Core/Src/ntc10kS.o \
./Core/Src/onewire.o \
./Core/Src/page.o \
./Core/Src/pid.o \
./Core/Src/pidspeed.o \
./Core/Src/pwm.o \
./Core/Src/ssd1306.o \
./Core/Src/ssd1306_fonts.o \
./Core/Src/ssd1306_tests.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/tim.o \
./Core/Src/usart.o 

C_DEPS += \
./Core/Src/MCP4725.d \
./Core/Src/PIDrestrictions.d \
./Core/Src/SwitchSpeed.d \
./Core/Src/ThrottleADC.d \
./Core/Src/acs758.d \
./Core/Src/adc.d \
./Core/Src/battery.d \
./Core/Src/buttons.d \
./Core/Src/ds18b20.d \
./Core/Src/gpio.d \
./Core/Src/gps_neo6.d \
./Core/Src/hall.d \
./Core/Src/i2c.d \
./Core/Src/iwdg.d \
./Core/Src/lm35.d \
./Core/Src/main.d \
./Core/Src/ntc10k.d \
./Core/Src/ntc10kS.d \
./Core/Src/onewire.d \
./Core/Src/page.d \
./Core/Src/pid.d \
./Core/Src/pidspeed.d \
./Core/Src/pwm.d \
./Core/Src/ssd1306.d \
./Core/Src/ssd1306_fonts.d \
./Core/Src/ssd1306_tests.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/tim.d \
./Core/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/MCP4725.cyclo ./Core/Src/MCP4725.d ./Core/Src/MCP4725.o ./Core/Src/MCP4725.su ./Core/Src/PIDrestrictions.cyclo ./Core/Src/PIDrestrictions.d ./Core/Src/PIDrestrictions.o ./Core/Src/PIDrestrictions.su ./Core/Src/SwitchSpeed.cyclo ./Core/Src/SwitchSpeed.d ./Core/Src/SwitchSpeed.o ./Core/Src/SwitchSpeed.su ./Core/Src/ThrottleADC.cyclo ./Core/Src/ThrottleADC.d ./Core/Src/ThrottleADC.o ./Core/Src/ThrottleADC.su ./Core/Src/acs758.cyclo ./Core/Src/acs758.d ./Core/Src/acs758.o ./Core/Src/acs758.su ./Core/Src/adc.cyclo ./Core/Src/adc.d ./Core/Src/adc.o ./Core/Src/adc.su ./Core/Src/battery.cyclo ./Core/Src/battery.d ./Core/Src/battery.o ./Core/Src/battery.su ./Core/Src/buttons.cyclo ./Core/Src/buttons.d ./Core/Src/buttons.o ./Core/Src/buttons.su ./Core/Src/ds18b20.cyclo ./Core/Src/ds18b20.d ./Core/Src/ds18b20.o ./Core/Src/ds18b20.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/gps_neo6.cyclo ./Core/Src/gps_neo6.d ./Core/Src/gps_neo6.o ./Core/Src/gps_neo6.su ./Core/Src/hall.cyclo ./Core/Src/hall.d ./Core/Src/hall.o ./Core/Src/hall.su ./Core/Src/i2c.cyclo ./Core/Src/i2c.d ./Core/Src/i2c.o ./Core/Src/i2c.su ./Core/Src/iwdg.cyclo ./Core/Src/iwdg.d ./Core/Src/iwdg.o ./Core/Src/iwdg.su ./Core/Src/lm35.cyclo ./Core/Src/lm35.d ./Core/Src/lm35.o ./Core/Src/lm35.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/ntc10k.cyclo ./Core/Src/ntc10k.d ./Core/Src/ntc10k.o ./Core/Src/ntc10k.su ./Core/Src/ntc10kS.cyclo ./Core/Src/ntc10kS.d ./Core/Src/ntc10kS.o ./Core/Src/ntc10kS.su ./Core/Src/onewire.cyclo ./Core/Src/onewire.d ./Core/Src/onewire.o ./Core/Src/onewire.su ./Core/Src/page.cyclo ./Core/Src/page.d ./Core/Src/page.o ./Core/Src/page.su ./Core/Src/pid.cyclo ./Core/Src/pid.d ./Core/Src/pid.o ./Core/Src/pid.su ./Core/Src/pidspeed.cyclo ./Core/Src/pidspeed.d ./Core/Src/pidspeed.o ./Core/Src/pidspeed.su ./Core/Src/pwm.cyclo ./Core/Src/pwm.d ./Core/Src/pwm.o ./Core/Src/pwm.su ./Core/Src/ssd1306.cyclo ./Core/Src/ssd1306.d ./Core/Src/ssd1306.o ./Core/Src/ssd1306.su ./Core/Src/ssd1306_fonts.cyclo ./Core/Src/ssd1306_fonts.d ./Core/Src/ssd1306_fonts.o ./Core/Src/ssd1306_fonts.su ./Core/Src/ssd1306_tests.cyclo ./Core/Src/ssd1306_tests.d ./Core/Src/ssd1306_tests.o ./Core/Src/ssd1306_tests.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su

.PHONY: clean-Core-2f-Src

