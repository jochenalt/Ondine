/*
 * main.cpp
 *
 *  Created on: 01.10.2018
 *      Author: JochenAlt
 */

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <variant/arduino_pins.h>
#include <PatternBlinker.h>
#include <WebServer.h>
#include <LogStream.h>

static uint8_t DefaultPattern[3] = { 0b11001000, 0b00001100, 0b10000000 };	// nice!
PatternBlinker ledBlinker(LED_BUILTIN, 50 /* ms */, true); // one bit in the patterns above is active for 100ms
WebServer webserver;
LogStream* logger = new LogStream();

void setup() {
	digitalWrite(LED_BUILTIN,LOW);
	webserver.setup();
	ledBlinker.set(DefaultPattern,sizeof(DefaultPattern));
	logger->println("setup");
}

void loop() {
	uint32_t now = millis();
	ledBlinker.loop(now);
	webserver.loop();
}
