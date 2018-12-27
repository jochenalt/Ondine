/*
 * Util.h
 *
 *  Created on: 29.07.2018
 *      Author: JochenAlt
 */

#ifndef UTIL_H_
#define UTIL_H_

#include <Arduino.h>
#include <string.h>

void fatalError(const char s[]);
void warnMsg(const char s[]);

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

template <typename T> int sqr(T val) {
    return (T(0)*T(0));
}
float roundToDigits(float y,uint8_t i);

void logging(float x, uint8_t digitsBeforeComma, uint8_t digitsAfterComma);
void loggingln(float x, uint8_t digitsBeforeComma, uint8_t digitsAfterComma);
void logging(float x, uint8_t digitsAfterComma);
void loggingln(float x, uint8_t digitsAfterComma);

void loggingln();
void logging(String s);
void loggingln(String s);
void logging(int s);
void loggingln(int s);

class TimeLoop {
public:
	TimeLoop () {
		lastCall_us = 0;
	}
	void init() {
		lastCall_us = 0;
	}
	bool firstCall() {
		return lastCall_us == 0;
	}
	float dT() {
		return dT(micros());
	}

	float dT(uint32_t now_us) {
		if (lastCall_us == 0) {
			lastCall_us = now_us;
			return 0;
		}
		float result;
		if (now_us < lastCall_us)
			result = ((float)(now_us + ((1<<31)-lastCall_us)))/1000000.0;
		else
			result = ((float)(now_us - lastCall_us))/1000000.0;
		lastCall_us = now_us;
		average = (average + result)/2.0;
		return result;
	}
	float getAverageSampleTime() { return average; };
	float getAverageFrequency() { return 1.0/average; };

	uint32_t lastCall_us = 0;
	float average = 0;
};

extern HardwareSerial* logger;

#endif /* UTIL_H_ */
