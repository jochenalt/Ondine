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

void logging(float x,uint8_t digitsAfterComma) {
	logger->print(x,digitsAfterComma);
}

void loggingln(float x,uint8_t digitsAfterComma) {
	logging(x,digitsAfterComma);
	logger->println();
}

float roundToDigits(float y,uint8_t i) {
	if (i == 0)
		return floor(y);
	if (i == 1)
		return floor(y*10 + 0.5)/10.0;
	if (i == 2)
		return floor(y*100 + 0.5)/100.0;
	if (i == 3)
		return floor(y*1000 + 0.5)/1000.0;
	if (i == 4)
		return floor(y*10000 + 0.5)/10000.0;

	return floor(y*pow(10,i)+0.5)/pow(10,i);
}
void logging(float x, uint8_t digitsBeforeComma, uint8_t digitsAfterComma) {
	int d = 1;
	float y = x;
	if (x < 0) {
		d++;
		y = abs(y);
	}
	y = roundToDigits(y, digitsAfterComma);
	while (y >= 10.0) {
		y /= 10.0;
		d++;
	}
	for (int i = 0;i< digitsBeforeComma-d;i++) {
		logger->print(' ');
	}
	logger->print(x,digitsAfterComma);
}

void loggingln(float x, uint8_t digitsBeforeComma, uint8_t digitsAfterComma) {
	logging(x,digitsBeforeComma,digitsAfterComma);
	loggingln();
}

void logging(String s) {
	logger->print(s);
}
void loggingln() {
	logger->println();
}

void logging(int i) {
	logger->print(i);
}

void loggingln(int i) {
	logging(i);
	logger->println();
}

void loggingln(String s) {
	logger->println(s);
}
