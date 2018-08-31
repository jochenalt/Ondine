/*
 * Util.Cpp
 *
 *  Created on: 29.07.2018
 *      Author: JochenAlt
 */

#include "Arduino.h"
#include <Util.h>

void fatalError(const char s[]) {
	logger->print("FATAL");
	logger->println(s);
	delay(100);
}
void warnMsg(const char s[]) {
	logger->print("WARN:");
	logger->println(s);
	delay(100);

}
