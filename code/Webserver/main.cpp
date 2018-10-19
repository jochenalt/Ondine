/*
 * main.cpp
 *
 *  Created on: 01.10.2018
 *      Author: JochenAlt
 */

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PatternBlinker.h>
#include <WebServer.h>
#include <LogStream.h>
#include <Wire.h>
#include <Util.h>
#include <I2CPortScanner.h>
#include <pins.h>
#include <I2CMaster.h>
#include <Common.h>

// everyone likes a blinking LED
static uint8_t DefaultPattern[3] = { 0b11001000, 0b00001100, 0b10000000 };	// nice!
PatternBlinker ledBlinker(LED_BUILTIN, 50 /* ms */, true); // one bit in the patterns above is active for 50ms

// ESP8266 webserver
WebServer webserver;

// receive log messages from bot controller
HardwareSerial* botControllerLogs = NULL;

// communication to bot controller
I2CMaster* i2cMaster = new I2CMaster(&Wire);

void setup() {
	logger->println("setup webserver");

	digitalWrite(LED_BUILTIN,LOW);
	webserver.setup();
	ledBlinker.set(DefaultPattern,sizeof(DefaultPattern));

	logger->println("setup i2c to bot controller");

	logger->println("setup serial interface to bot controller");
	botControllerLogs = &Serial;
	botControllerLogs->swap();
	botControllerLogs->begin(230400);

	i2cMaster->setup();

	// int noDevices = doI2CPortScan(F("looking for BotControll"), &Wire, logger);
	Wire.beginTransmission(BotControllerI2CAddress);
	int error = Wire.endTransmission();
	if (error == 0) {
		logger->print("connection to Teensy via I2c (0x");
		logger->print(BotControllerI2CAddress, HEX);
		logger->println(" failed!");
	}

	logger->println("setup done");

}

void subloop() {
	// copy log from bot ctrl to log buffer
	while (botControllerLogs->available()) {
		char c = botControllerLogs->read();
		logger->print(c);
	}

	// carry out any pending communication
	i2cMaster->loop();
}

void loop() {
	uint32_t now = millis();
	ledBlinker.loop(now);
	webserver.loop();
	subloop();
}
