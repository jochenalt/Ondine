

#ifndef I2C_PORT_SCANNER_
#define I2C_PORT_SCANNER_

#include "Arduino.h"
#include <Wire.h>


int doI2CPortScan(const __FlashStringHelper *str, TwoWire* bus, Stream* logger);
bool scanI2CAddress(TwoWire* bus, uint8_t address, byte &error);

#endif
