/*
 * Util.Cpp
 *
 *  Created on: 29.07.2018
 *      Author: JochenAlt
 */

#include "Arduino.h"

void fatalError(const char s[]) {
	Serial1.print("FATAL");
	Serial1.println(s);
}
void warnMsg(const char s[]) {
	Serial1.print("WARN:");
	Serial1.println(s);
}
