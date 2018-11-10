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
