/*
 * AS5047D.h
 *
 *  Created on: 06.11.2018
 *      Author: JochenAlt
 */

#ifndef ENCODER_AS5047D_H_
#define ENCODER_AS5047D_H_

#include "Arduino.h"

class AS5047D
{
	const int resolution = 0x4000; // =16384

public:
	static void setupBus(uint16_t mosiPin, uint16_t misoPin, uint16_t SCKPin, const uint16_t SS[3]);
    void setup(uint16_t clientSelectPin);

    // read absolute angle [rad] from sensor, not limited to 0..360°.
    // first call after setup is normalized to 0..360°
    float readAngle();

    // return angle recently used
    float getAngle();

    float getSensorRead();
    // reset to angle between 0..360°
    void reset();
private:
	uint16_t selectPin = 0;
	uint32_t sensorRead(void);
	uint32_t readRegister(uint32_t register);
	float currentAngle = 0;
	uint32_t lastSensorRead = 0;
	static bool SPIBusInitialized;
};

#endif /* ENCODER_AS5047D_H_ */
