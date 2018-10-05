/*
 * I2CSlave.cpp
 *
 *  Created on: 05.10.2018
 *      Author: JochenAlt
 */

#include <I2CSlave.h>


void I2CSlave::setup() {
		webServerComm->begin(I2C_SLAVE, BotControllerI2CAddress, 0, I2C_PINS_37_38, I2C_PULLUP_EXT, I2C_RATE_400);          // join i2c bus with address #8
		webServerComm->setDefaultTimeout(4000); // 4ms default timeout

		// webServerComm->onRequest(webServerCommRequestEvent);
		webServerComm->onRequest([](){ i2cSlave->webServerCommRequestEvent();});

		// webServerComm->onReceive(webServerCommReceiveEvent);
		webServerComm->onReceive([](unsigned int howMany){ i2cSlave->webServerCommReceiveEvent(howMany);});

		requestBytes = 0;
}

void I2CSlave::loop() {
	if (reveiveEvent.length() != 0) {
		logger->print("execute (");
		logger->print(receiveAdr);
		logger->print(")=[");
		logger->print(reveiveEvent);
		logger->println("]");

		reveiveEvent = "";

		switch (receiveAdr) {
			case BotCtrlCmd_SerialCommand:
				break;
			default:
				logger->print("invalid i2c register");
				logger->println(receiveAdr);
		}
	}
}
