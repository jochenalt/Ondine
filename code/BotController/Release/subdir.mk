################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
..\BLDCController.cpp \
..\Util.cpp \
..\main.cpp 

LINK_OBJ += \
.\BLDCController.cpp.o \
.\Util.cpp.o \
.\main.cpp.o 

CPP_DEPS += \
.\BLDCController.cpp.d \
.\Util.cpp.d \
.\main.cpp.d 


# Each subdirectory must supply rules for building sources it contributes
BLDCController.cpp.o: ..\BLDCController.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"C:/Program Files (x86)/Arduino/hardware/teensy/../tools/arm/bin/arm-none-eabi-g++" -c -O2 -g -Wall -ffunction-sections -fdata-sections -nostdlib -fno-exceptions -felide-constructors -std=gnu++0x -fno-rtti -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -D__MK64FX512__ -DTEENSYDUINO=136 -DARDUINO=10802 -DF_CPU=120000000 -DUSB_SERIAL -DLAYOUT_GERMAN  -I"D:\Projects\FlowerPot\code\BotController" -I"D:\Projects\FlowerPot\code\BotController\Encoder" -I"D:\Projects\FlowerPot\code\BotController\utilities" -I"C:\Program Files (x86)\Arduino\hardware\teensy\avr\cores\teensy3" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"
	@echo 'Finished building: $<'
	@echo ' '

Util.cpp.o: ..\Util.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"C:/Program Files (x86)/Arduino/hardware/teensy/../tools/arm/bin/arm-none-eabi-g++" -c -O2 -g -Wall -ffunction-sections -fdata-sections -nostdlib -fno-exceptions -felide-constructors -std=gnu++0x -fno-rtti -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -D__MK64FX512__ -DTEENSYDUINO=136 -DARDUINO=10802 -DF_CPU=120000000 -DUSB_SERIAL -DLAYOUT_GERMAN  -I"D:\Projects\FlowerPot\code\BotController" -I"D:\Projects\FlowerPot\code\BotController\Encoder" -I"D:\Projects\FlowerPot\code\BotController\utilities" -I"C:\Program Files (x86)\Arduino\hardware\teensy\avr\cores\teensy3" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"
	@echo 'Finished building: $<'
	@echo ' '

main.cpp.o: ..\main.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"C:/Program Files (x86)/Arduino/hardware/teensy/../tools/arm/bin/arm-none-eabi-g++" -c -O2 -g -Wall -ffunction-sections -fdata-sections -nostdlib -fno-exceptions -felide-constructors -std=gnu++0x -fno-rtti -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -D__MK64FX512__ -DTEENSYDUINO=136 -DARDUINO=10802 -DF_CPU=120000000 -DUSB_SERIAL -DLAYOUT_GERMAN  -I"D:\Projects\FlowerPot\code\BotController" -I"D:\Projects\FlowerPot\code\BotController\Encoder" -I"D:\Projects\FlowerPot\code\BotController\utilities" -I"C:\Program Files (x86)\Arduino\hardware\teensy\avr\cores\teensy3" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"
	@echo 'Finished building: $<'
	@echo ' '


