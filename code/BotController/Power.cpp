/*
 * Power.cpp
 *
 *  Created on: 22.09.2018
 *      Author: JochenAlt
 */

#include <Arduino.h>
#include <setup.h>
#include <Power.h>

void Power::setup() {
	motorPower(false);
}

void Power::motorPower(bool on) {
	motorOn = on;
	if (motorOn) {
		pinMode(POWER_RELAY_PIN, OUTPUT);
		digitalWrite(POWER_RELAY_PIN, LOW);
	} else {
		// teensy output is 3.3V, since relay works with 5V
		// we need an open collector to release the power-relay
		// (check with schematics)
		pinMode(POWER_RELAY_PIN, INPUT);
	}
}

bool Power::isMotorOn() {
	return motorOn;
}
