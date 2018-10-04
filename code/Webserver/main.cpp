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

// everyone likes a blinking LED
static uint8_t DefaultPattern[3] = { 0b11001000, 0b00001100, 0b10000000 };	// nice!
PatternBlinker ledBlinker(LED_BUILTIN, 50 /* ms */, true); // one bit in the patterns above is active for 50ms

// ESP8266 webserver
WebServer webserver;

// cicrular buffer for leg entries shown on the web page
LogStream* logger = new LogStream();

// receive log messages from bot controller
HardwareSerial* botControllerLogs = NULL;

void setup() {
	logger->println("setup webserver");

	digitalWrite(LED_BUILTIN,LOW);
	webserver.setup();
	ledBlinker.set(DefaultPattern,sizeof(DefaultPattern));

	logger->println("setup i2c to bot controller");
	ctrlComm = &Wire;

	logger->println("setup serial interface to bot controller");
	botControllerLogs = &Serial;
	botControllerLogs->swap();
	botControllerLogs->begin(230400);

	ctrlComm->begin(SDA, SCL);        // join i2c bus as master

	// doI2CPortScan(F("looking for BotControll"), ctrlComm, logger);

	logger->println("setup done");

}

void subloop() {
	while (botControllerLogs->available()) {
		char c = botControllerLogs->read();
		logger->print(c);
	}
	communicate();
}

void loop() {
	uint32_t now = millis();
	ledBlinker.loop(now);
	webserver.loop();
	subloop();
}
