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

extern HardwareSerial* logger;
extern HardwareSerial* command;

#endif /* UTIL_H_ */
