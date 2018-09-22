#include <BrushlessMotorDriver.h>
#include "Arduino.h"
#include "MenuController.h"
#include "Engine.h"
#include "BotController.h"

#include <utilities/PatternBlinker.h>
#include <i2c_t3-v9.1/i2c_t3-v9.1.h>

static uint8_t DefaultPattern[3] = { 0b11001000, 0b00001100, 0b10000000 };	// nice!

PatternBlinker ledBlinker(LED_PIN, 50 /* ms */); // one bit in the patterns above is active for 100ms

HardwareSerial* logger = &Serial5;			// UART used to log
HardwareSerial* command = &Serial5;			// UART used to log

BotController botController;

i2c_t3* Wires[3] = { &Wire, &Wire1, &Wire2};		// we have two I2C buses due to conflicting sensor addresses
i2c_t3* IMUWire = NULL;
i2c_t3* cortexWire = NULL;


void setup()
{

	// let the LED blink two times to indicate that setup is starting now
	digitalWrite(LED_PIN,LOW);
	delay(30);
	digitalWrite(LED_PIN,HIGH);
	delay(30);
	digitalWrite(LED_PIN,LOW);
	delay(30);
	digitalWrite(LED_PIN,HIGH);
	delay(30);
	digitalWrite(LED_PIN,LOW);

	ledBlinker.set(DefaultPattern,sizeof(DefaultPattern));

	command->begin(115200);
	botController.setup(); // this couple of second (mainly due to IMU)

	command->println("BotController - h for help");
}

void loop()
{
	uint32_t now = millis();
	ledBlinker.loop(now);    	// LED on Teensy board and LED on power switch
	botController.loop();
}
