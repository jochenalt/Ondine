#include <BrushlessMotorDriver.h>
#include "Arduino.h"
#include "MenuController.h"
#include "Engine.h"
#include "BotController.h"

#include <utilities/PatternBlinker.h>
#include <i2c_t3-v9.1/i2c_t3-v9.1.h>
#include <common.h>

static uint8_t DefaultPattern[3] = { 0b11001000, 0b00001100, 0b10000000 };	// nice!

PatternBlinker ledBlinker(LED_PIN, 50 /* ms */); // one bit in the patterns above is active for 100ms

HardwareSerial* logger = &Serial5;			// UART used to log
HardwareSerial* command = &Serial5;			// UART used to log

BotController botController;

i2c_t3* Wires[3] = { &Wire, &Wire1, &Wire2};
i2c_t3* IMUWire = NULL;

i2c_t3* webServerComm = &Wire1;
int requestBytes = 0;

void webServerCommRequestEvent() {
	logger->print("i2cReq"); // respond with message of 6 bytes
	for (int i = 0;i<requestBytes;i++)
		webServerComm->write(1);
	logger->println();
}

int receiveAdr;
String reveiveEvent;

void webServerCommReceiveEvent(unsigned int numBytes) {
	requestBytes = numBytes;
	reveiveEvent = "";
	// first byte is register
	receiveAdr = webServerComm->read();


	while (0 < webServerComm->available()) { // loop through all
		char c = webServerComm->read(); // receive byte as a character
		reveiveEvent += c;
	}
	if (receiveAdr == -1) {
		// empty transaction
		receiveAdr = 0;
		reveiveEvent = "";
	}
	/*
	{
		logger->print("i2cRec(");
		logger->print(numBytes);
		logger->print(",");
		logger->print(receiveAdr);
		logger->print("):[");
		logger->print(reveiveEvent);
		logger->println("]");
	}
	*/
}


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

	// command input comes via UART or I2C from ESP86266
	command->begin(230400);
	botController.setup(); // this couple of second (mainly due to IMU)

	webServerComm->begin(I2C_SLAVE, BotControllerI2CAddress, 0, I2C_PINS_37_38, I2C_PULLUP_EXT, I2C_RATE_400);          // join i2c bus with address #8
	webServerComm->setDefaultTimeout(4000); // 4ms default timeout

	webServerComm->onRequest(webServerCommRequestEvent);
	webServerComm->onReceive(webServerCommReceiveEvent);

	command->println("BotController - h for help");
}

void loop()
{
	uint32_t now = millis();
	ledBlinker.loop(now);    	// LED on Teensy board and LED on power switch
	botController.loop();

	if (reveiveEvent.length() != 0) {
		logger->print("execute (");
		logger->print(receiveAdr);
		logger->print(")=[");
		logger->print(reveiveEvent);
		logger->println("]");

		reveiveEvent = "";
	}
}
