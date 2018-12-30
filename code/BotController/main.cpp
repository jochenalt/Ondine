#include "Arduino.h"
#include <libraries/I2CSlave.h>

#include <BrushlessMotorDriver.h>
#include "Engine.h"
#include "BotController.h"

#include <PatternBlinker.h>
#include <BotMemory.h>

#include <common.h>

static uint8_t DefaultPattern[3] = { 0b11001000, 0b00001100, 0b10000000 };	// nice!

PatternBlinker ledBlinker(LED_PIN, 50 /* ms */); // one bit in the patterns above is active for 100ms

HardwareSerial* logger = &Serial5;			// UART used to log


i2c_t3* Wires[3] = { &Wire, &Wire1, &Wire2};
i2c_t3* IMUWire = NULL;

I2CSlave* i2cSlave = new I2CSlave(&Wire1);

void setup()
{
	logger->begin(460800);
	uint32_t now = millis();

	// let the LED blink two times to indicate startup
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

	// initialize configuration values coming from EEPROM
	memory.setup();

	// join the i2c bus with the webserver
	i2cSlave->setup();

	// this takes 3.5s seconds (mainly due to IMU initialization)
	BotController::getInstance()->setup();

	// initialize LED_PIN after BotController::getInstance()->setup()
	// since SPI does use it default wise for SCK, but this is remapped
	pinMode(LED_PIN, OUTPUT);

	logger->print("BotController - h for help ");
	logger->print(millis()-now);
	logger->print("ms setup time");
	logger->println();
}

void loop()
{
	ledBlinker.loop(millis());    				// LED on Teensy board and LED on power switch
	BotController::getInstance()->loop();		// do the balancing business
	i2cSlave->loop();							// execute commands from webserver that came in via I2C
	memory.loop(millis());						// check if something happened in memory that is to be saved in EEPROM
}
