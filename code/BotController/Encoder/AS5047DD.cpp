/*
 * AS5047D.cpp
 *
 *  Created on: 06.11.2018
 *      Author: JochenAlt
 */



#include <AS5047DD.h>
#include "Arduino.h"
#include "SPI.h"
AS5047D::AS5047D(uint16_t mosiPin, uint16_t misoPin, uint16_t SCKPin, uint16_t SelectPin)
         : selectPin(SelectPin)
{
         pinMode(selectPin, OUTPUT);
         // SPCR = (1<<SPE) | (1<<MSTR) | (1<<CPHA) | (1<<SPR1) | (1<<SPR0); // slow down clock speed, set up spi
         SPI.setClockDivider( SPI_CLOCK_DIV128 );
         SPI.setMOSI(mosiPin);
         SPI.setMISO(misoPin);
         SPI.setSCK(SCKPin);
         SPI.setDataMode(1); // The AS5047D SPI uses mode=1 (CPOL=0, CPHA=1)
         SPI.begin();
}

uint32_t AS5047D::sensorRead(void)
{
	uint16_t angle = readRegister(0x3FFF);
    return angle;
}

float AS5047D::readAngle() {
	int sensorValue = sensorRead();

	int diff = sensorValue - lastSensorRead;
	// did we cross the 0 value coming from large angle ?
	if (abs(diff) > abs ( diff + resolution))
		diff += resolution;
	// did we cross the 0 value coming from small angle ?
	if (abs(diff) > abs ( diff - resolution))
		diff -= resolution;

	currentAngle += ((float)diff)/((float)resolution);
	return currentAngle;
}

float AS5047D::getAngle() {
	return currentAngle;
}

uint32_t AS5047D::readRegister(uint32_t thisRegister)
{
        byte inByte = 0;   					// incoming byte from the SPI
        uint32_t result = 0;   					// result to return
        byte lowbyte = thisRegister & 0b0000000011111111;
        byte highbyte = (thisRegister >> 8);

        digitalWrite(selectPin, LOW);		// select sensor
        SPI.transfer(highbyte); 			// first byte in
        result = SPI.transfer(lowbyte); 	// first byte out
        digitalWrite(selectPin, HIGH);		// unselect sensor

        delayMicroseconds(10);

        digitalWrite(selectPin, LOW);		// select sensor
        int bytesToRead = 2;
        while (bytesToRead-- > 0) {
                // shift the first byte left, then get the second byte:
                result = result << 8;
                inByte = SPI.transfer(0x00);
                result = result | inByte;
        }
        digitalWrite(selectPin, HIGH);		// deselect sensor

        result = result & 0b0011111111111111;
        return(result);
}
