################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Inc/stm32f103xx_gpio_driver.cpp 

OBJS += \
./Inc/stm32f103xx_gpio_driver.o 

CPP_DEPS += \
./Inc/stm32f103xx_gpio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Inc/%.o Inc/%.su Inc/%.cyclo: ../Inc/%.cpp Inc/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -g3 -DDEBUG -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -c -I../Inc -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Inc

clean-Inc:
	-$(RM) ./Inc/stm32f103xx_gpio_driver.cyclo ./Inc/stm32f103xx_gpio_driver.d ./Inc/stm32f103xx_gpio_driver.o ./Inc/stm32f103xx_gpio_driver.su

.PHONY: clean-Inc

