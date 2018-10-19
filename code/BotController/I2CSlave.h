/*
 * I2CSlave.h
 *
 *  Created on: 05.10.2018
 *      Author: JochenAlt
 */

#ifndef I2CSLAVE_H_
#define I2CSLAVE_H_

#include <Arduino.h>
#include <common.h>
#include <i2c_t3-v9.1/i2c_t3-v9.1.h>
#include <Util.h>

class I2CSlave {
public:
	I2CSlave(i2c_t3* i2cBus) {
		this->webServerComm = i2cBus;
	};
	virtual ~I2CSlave() {};

	void webServerCommRequestEvent() {
		logger->print("i2cReq"); // respond with message of 6 bytes
		for (int i = 0;i<requestBytes;i++)
			webServerComm->write(1);
		logger->println();
	}

	void webServerCommReceiveEvent(unsigned int numBytes) {
		requestBytes = numBytes;
		reveiveEvent = "";
		// first byte is register
		receiveAdr = webServerComm->read();


		while (0 < webServerComm->available()) { // loop through all
			char c = webServerComm->read(); // receive byte as a character
			reveiveEvent += c;
		}
		if (receiveAdr == -1) {
			// empty transaction
			receiveAdr = 0;
			reveiveEvent = "";
		}
		{
			logger->print("i2cRec(");
			logger->print(numBytes);
			logger->print(",");
			logger->print(receiveAdr);
			logger->print("):[");
			logger->print(reveiveEvent);
			logger->println("]");
		}
	}

	void setup();
	void loop();
private:
	i2c_t3* webServerComm = NULL;
	int requestBytes = 0;
	int receiveAdr = 0;
	String reveiveEvent;

};

extern I2CSlave* i2cSlave;
#endif /* I2CSLAVE_H_ */
