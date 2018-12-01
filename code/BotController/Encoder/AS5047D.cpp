/*
 * AS5047D.cpp
 *
 *  Created on: 06.11.2018
 *      Author: JochenAlt
 */



#include <AS5047D.h>
#include "Arduino.h"
#include "SPI.h"
#include "libraries/Util.h"


bool AS5047D::SPIBusInitialized = false;


void AS5047D::setupBus(uint16_t mosiPin, uint16_t misoPin, uint16_t SCKPin, const uint16_t SS[3]) {
	// do this only once.
	if (!SPIBusInitialized) {
	    pinMode(SS[0], OUTPUT);
	    digitalWrite(SS[0], HIGH);
	    pinMode(SS[1], OUTPUT);
	    digitalWrite(SS[1], HIGH);
	    pinMode(SS[2], OUTPUT);
	    digitalWrite(SS[2], HIGH);

		SPI.setMOSI(mosiPin);
		SPI.setMISO(misoPin);
		SPI.setSCK(SCKPin);
		SPI.begin();
		SPIBusInitialized = true;
	}
}

void AS5047D::setup(uint16_t clientSelectPin)
{
	if (!SPIBusInitialized) {
		fatalError("SPI bus needs to be initialized before sensor setup");
	}
    selectPin = clientSelectPin;

    // dont start with 0 but the absolute position
	reset();

}

uint32_t AS5047D::sensorRead(void)
{
	uint32_t angle = readRegister(0x3FFF);

    return resolution-angle;

}

void AS5047D::reset() {
	float sensorValue = sensorRead();
	currentAngle = ((float)sensorValue)/((float)resolution)*TWO_PI;
	lastSensorRead = sensorValue;
}

float AS5047D::readAngle() {
	int sensorValue = sensorRead();


	int32_t diff = sensorValue - lastSensorRead;
	lastSensorRead = sensorValue;

	// did we cross the 0 value coming from large angle ?
	if (abs(diff) > abs ( diff + resolution))
		diff += resolution;
	// did we cross the 0 value coming from small angle ?
	if (abs(diff) > abs ( diff - resolution))
		diff -= resolution;

	currentAngle += ((float)diff)/((float)resolution)*TWO_PI;
	return currentAngle;
}

float AS5047D::getAngle() {
	return currentAngle;
}

float AS5047D::getSensorRead() {
	return ((float)lastSensorRead)/((float)resolution)*TWO_PI;
}



uint32_t AS5047D::readRegister(uint32_t thisRegister)
{
        byte inByte = 0;   					// incoming byte from the SPI
        uint32_t result = 0;   				// result to return
        byte lowbyte = thisRegister & 0b0000000011111111;
        byte highbyte = (thisRegister >> 8);
        pinMode(selectPin, OUTPUT);

        // AS5047D is capable of running at 10MHz.
        // But 2MHz is more than enough for this purpose
        SPISettings settings(2000000,MSBFIRST,SPI_MODE1);
        SPI.beginTransaction(settings);
        digitalWrite(selectPin, LOW);		// select sensor

        SPI.transfer(highbyte); 			// first byte in
        result = SPI.transfer(lowbyte); 	// first byte out
        digitalWrite(selectPin, HIGH);		// unselect sensor
        SPI.endTransaction();

        delayMicroseconds(10);				// at least 350ns (datasheet)

        SPI.beginTransaction(settings);
        digitalWrite(selectPin, LOW);		// select sensor
        int bytesToRead = 2;
        while (bytesToRead-- > 0) {
                // shift the first byte left, then get the second byte:
                result = result << 8;
                inByte = SPI.transfer(0x00);
                result = result | inByte;
        }
        digitalWrite(selectPin, HIGH);		// deselect sensor
        SPI.endTransaction();

        // mask bits 0-13 (data sheet)
        result &= 0x3FFF;

        return(result);
}
