/*
 * I2CMaster.h
 *
 *  Created on: 05.10.2018
 *      Author: JochenAlt
 */

#ifndef I2CMASTER_H_
#define I2CMASTER_H_

#include <Arduino.h>
#include <Wire.h>

class I2CMaster {
public:
	I2CMaster(TwoWire* i2c) {
		this->ctrlComm = i2c;
	};
	virtual ~I2CMaster() {};

	void setup();

	void sendCommand(uint8_t adr, String cmd);
	void sendCommandAsync(int adr, String cmd);
	String requestResponse(int numberOfBytes);

	// carry out buffered communication
	void loop();

private:
	TwoWire* ctrlComm = NULL;
};

extern I2CMaster* i2cMaster;




#endif /* I2CMASTER_H_ */
