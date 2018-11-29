

#ifndef I2C_PORT_SCANNER_
#define I2C_PORT_SCANNER_

#include "Arduino.h"
#include <i2c_t3-v9.1/i2c_t3-v9.1.h>


int doI2CPortScan(const __FlashStringHelper *str, i2c_t3* bus, Stream* logger);
bool scanI2CAddress(i2c_t3* bus, uint8_t address, byte &error);

#endif
