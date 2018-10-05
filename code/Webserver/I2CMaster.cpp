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
	ctrlComm->begin(SDA, SCL);        // join i2c bus as master
}

void I2CMaster::sendCommandAsync(int adr,String cmd) {
	pendingCommand = cmd;
	pendingAdr = adr;
}

void I2CMaster::sendCommand(uint8_t adr, String cmd) {
	ctrlComm->beginTransmission(BotControllerI2CAddress); // transmit to bot controller
	ctrlComm->write((uint8_t)adr);
	for (unsigned int i = 0;i<cmd.length();i++)
		ctrlComm->write(cmd[i]);
	uint8_t status = ctrlComm->endTransmission();    // stop transmitting

	// dont know why, but without this empty transmission the previous transmission is bufferd somewhere
	ctrlComm->beginTransmission(BotControllerI2CAddress);
	status = ctrlComm->endTransmission();

	logger->print("send(");
	logger->print(adr);
	logger->print("):[");
	logger->print(cmd);
	logger->print("]=");
	logger->println(status);

}

String I2CMaster::requestResponse(int numberOfBytes) {
	ctrlComm->requestFrom(BotControllerI2CAddress, numberOfBytes);
	String result = "";
	while (ctrlComm->available()) { // slave may send less than requested
		char c = ctrlComm->read(); // receive a byte as character
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
