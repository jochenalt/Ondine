################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
..\utilities\ClassInterrupt.cpp 

LINK_OBJ += \
.\utilities\ClassInterrupt.cpp.o 

CPP_DEPS += \
.\utilities\ClassInterrupt.cpp.d 


# Each subdirectory must supply rules for building sources it contributes
utilities\ClassInterrupt.cpp.o: ..\utilities\ClassInterrupt.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"C:/Program Files (x86)/Arduino/hardware/teensy/../tools/arm/bin/arm-none-eabi-g++" -c -O2 -g -Wall -ffunction-sections -fdata-sections -nostdlib -fno-exceptions -felide-constructors -std=gnu++0x -fno-rtti -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -D__MK64FX512__ -DTEENSYDUINO=136 -DARDUINO=10802 -DF_CPU=120000000 -DUSB_SERIAL -DLAYOUT_GERMAN  -I"D:\Projects\FlowerPot\code\BotController" -I"D:\Projects\FlowerPot\code\BotController\Encoder" -I"D:\Projects\FlowerPot\code\BotController\utilities" -I"C:\Program Files (x86)\Arduino\hardware\teensy\avr\cores\teensy3" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"
	@echo 'Finished building: $<'
	@echo ' '


