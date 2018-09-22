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
	pinMode(POWER_RELAY_PIN, OUTPUT);
	digitalWrite(POWER_RELAY_PIN, LOW);
	motorOn = false;
}

void Power::motorPower(bool on) {
	motorOn = on;
	digitalWrite(POWER_RELAY_PIN, motorOn?HIGH:LOW);
}

bool Power::isMotorOn() {
	return motorOn;
}
