/*
* I2CPortScanner.h
*
* Created: 02.05.2016 10:03:43
* Author: JochenAlt
*/

#include <i2c_t3-v9.1/i2c_t3-v9.1.h>


int doI2CPortScan(const __FlashStringHelper *str, i2c_t3* bus, Stream* logger)
{
	byte error, address;
	int nDevices;

	nDevices = 0;
	bool deviceFound = false;
	logger->print(str);
	logger->print(" ");

	for(address = 1; address < 127; address++ )
	{
		// The i2c_scanner uses the return value of
		// the Write.endTransmisstion to see if
		// a device did acknowledge to the address.

		bus->beginTransmission(address);
		error = bus->endTransmission();

		if (error == 0)
		{
			if (!deviceFound) {
				deviceFound = true;
			}
			else
				logger->print(F(" "));

			logger->print("0x");
			if (address<16)
				logger->print("0");
			logger->print(address,HEX);

			nDevices++;
		}
		else if (error==4)
		{
			logger->print(F("Unknown error at address 0x"));
			if (address<16)
				logger->print("0");
			logger->println(address,HEX);
		}
	}
	if (deviceFound)
		logger->println();
	if (nDevices == 0)
		logger->println(F("No I2C devices found"));

	return nDevices;
}

bool scanI2CAddress(i2c_t3* bus, uint8_t address, byte &error)
{
	// The i2c_scanner uses the return value of
	// the Write.endTransmisstion to see if
	// a device did acknowledge to the address.
	bus->beginTransmission(address);
	error = bus->endTransmission();
	return error == 0;
}
