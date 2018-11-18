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
#include <i2c_t3-v9.1/i2c_t3-v9.1.h>

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

extern HardwareSerial* logger;
extern HardwareSerial* command;
extern i2c_t3* IMUWire;

#endif /* UTIL_H_ */
