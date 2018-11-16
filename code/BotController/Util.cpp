/*
 * Util.Cpp
 *
 *  Created on: 29.07.2018
 *      Author: JochenAlt
 */

#include "Arduino.h"
#include <Util.h>

void fatalError(const char s[]) {
	if (logger) {
		logger->print("FATAL:");
		logger->println(s);
	}
	delay(100); // wait until serial sent that before crashing
}
void warnMsg(const char s[]) {
	if (logger) {
		logger->print("WARN:");
		logger->println(s);
	}
	delay(100); // wait until serial sent that before crashing
}


void log(float x, uint8_t digitsBeforeComma, uint8_t digitsAfterComma) {
	int d = 1;
	float y = x;
	if (x < 0) {
		d++;
		y = abs(y);
	}
	while (y > 10.0) {
		y /= 10.0;
		d++;
	}
	for (int i = 0;i< digitsBeforeComma-d;i++) {
		logger->print(' ');
	}
	logger->print(x,digitsAfterComma);
}

void logln(float x, uint8_t digitsBeforeComma, uint8_t digitsAfterComma) {
	log(x,digitsBeforeComma,digitsAfterComma);
	logln();
}

void log(String s) {
	logger->print(s);
}
void logln() {
	logger->println();
}
void logln(String s) {
	logger->println(s);
}
