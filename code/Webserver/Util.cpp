
#include <Arduino.h>
#include <Util.h>

TwoWire* ctrlComm = NULL;

String pendingCommand;
int pendingAdr;

void sendCommandAsync(int adr,String cmd) {
	pendingCommand = cmd;
	pendingAdr = adr;
}

void sendCommand(uint8_t adr, String cmd) {
	ctrlComm->beginTransmission(ctrlCommAddress); // transmit to bot controller
	ctrlComm->write((uint8_t)adr);
	for (unsigned int i = 0;i<cmd.length();i++)
		ctrlComm->write(cmd[i]);
	uint8_t status = ctrlComm->endTransmission();    // stop transmitting

	// dont know why, but without this empty transmission the previous transmission is bufferd somewhere
	ctrlComm->beginTransmission(ctrlCommAddress);
	status = ctrlComm->endTransmission();

	logger->print("send(");
	logger->print(adr);
	logger->print("):[");
	logger->print(cmd);
	logger->print("]=");
	logger->println(status);

}

String requestResponse(int numberOfBytes) {
	ctrlComm->requestFrom(ctrlCommAddress, numberOfBytes);
	String result = "";
	while (ctrlComm->available()) { // slave may send less than requested
		char c = ctrlComm->read(); // receive a byte as character
		result += c;
	}
	return result;
}

void communicate() {
	if (pendingCommand.length() > 0) {
		sendCommand(pendingAdr, pendingCommand);
		pendingCommand = "";
	}
}
