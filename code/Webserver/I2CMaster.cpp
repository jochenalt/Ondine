/*
 * I2CMaster.cpp
 *
 *  Created on: 05.10.2018
 *      Author: JochenAlt
 */

#include <I2CMaster.h>
#include <Wire.h>
#include <common.h>
#include <Util.h>


String pendingCommand;
int pendingAdr;

void I2CMaster::setup() {
	i2cBus->begin(PIN_WIRE_SDA, PIN_WIRE_SCL);        // join i2c bus as master
}

void I2CMaster::sendCommandAsync(int adr,String cmd) {
	pendingCommand = cmd;
	pendingAdr = adr;
}

void I2CMaster::sendCommand(uint8_t adr, String cmd) {
	i2cBus->beginTransmission(BotControllerI2CAddress); // transmit to bot controller
	i2cBus->write((uint8_t)adr);
	for (unsigned int i = 0;i<cmd.length();i++)
		i2cBus->write(cmd[i]);
	uint8_t status = i2cBus->endTransmission();    // stop transmitting

	// dont know why, but without this empty transmission the previous
	// transmission is bufferd somewhere and not recived by the bot controller
	i2cBus->beginTransmission(BotControllerI2CAddress);
	status = i2cBus->endTransmission();

	logger->print("send(");
	logger->print(adr);
	logger->print("):[");
	logger->print(cmd);
	logger->print("]=");
	logger->println(status);
}

String I2CMaster::requestResponse(int numberOfBytes) {
	i2cBus->requestFrom(BotControllerI2CAddress, numberOfBytes);
	String result = "";
	while (i2cBus->available()) { // slave may send less than requested
		char c = i2cBus->read(); // receive a byte as character
		result += c;
	}
	return result;
}

void I2CMaster::loop() {
	if (pendingCommand.length() > 0) {
		sendCommand(pendingAdr, pendingCommand);
		pendingCommand = "";
	}
}
