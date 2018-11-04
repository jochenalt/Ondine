#include <BrushlessMotorDriver.h>
#include "Arduino.h"
#include "MenuController.h"
#include "Engine.h"
#include "BotController.h"

#include <PatternBlinker.h>
#include <I2CSlave.h>
#include <BotMemory.h>

#include <common.h>

static uint8_t DefaultPattern[3] = { 0b11001000, 0b00001100, 0b10000000 };	// nice!

PatternBlinker ledBlinker(LED_PIN, 50 /* ms */); // one bit in the patterns above is active for 100ms

HardwareSerial* logger = &Serial5;			// UART used to log
HardwareSerial* command = &Serial5;			// UART used to log

BotController& botController = BotController::getInstance();

i2c_t3* Wires[3] = { &Wire, &Wire1, &Wire2};
i2c_t3* IMUWire = NULL;

I2CSlave* i2cSlave = new I2CSlave(&Wire1);

void setup()
{
	// command input comes via UART or I2C from ESP86266
	command->begin(230400);
	uint32_t now = millis();
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

	botController.setup(); 	// this takes 4.5s seconds (mainly due to IMU initialization)

	i2cSlave->setup(); 		// join the i2c bus with the webserver

	// initialize configuration values coming from EEPROM
	memory.setup();

	command->print("ms BotController - h for help ");
	command->print(millis()-now);
	command->print("ms setup time");
	command->println();
}

void loop()
{
	uint32_t now = millis();
	ledBlinker.loop(now);    	// LED on Teensy board and LED on power switch
	botController.loop();		// do the balancing business
	i2cSlave->loop();			// execute commands from webserver that came in via I2C
	memory.loop(now);
}
