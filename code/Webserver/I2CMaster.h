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

/*
 * Class to communicate between web page and bot controller via i2c
 * WebServer has the master role
 */
class I2CMaster {
public:
	I2CMaster(TwoWire* i2c) {
		this->i2cBus = i2c;
	};
	virtual ~I2CMaster() {};

	// initialize i2c line
	void setup();

	// send a command via i2c right away
	void sendCommand(uint8_t adr, String cmd);

	// buffer a command to be sent to bot controller. To execute within next call of loop
	void sendCommandAsync(int adr, String cmd);

	// request a response of the previous command
	String requestResponse(int numberOfBytes);

	// carry out buffered communication
	void loop();

private:
	TwoWire* i2cBus = NULL;
};

extern I2CMaster* i2cMaster;




#endif /* I2CMASTER_H_ */
