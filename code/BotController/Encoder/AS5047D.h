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
	const int resolution = 8192;

public:
	static void setupBus(uint16_t mosiPin, uint16_t misoPin, uint16_t SCKPin);
    void setup(uint16_t clientSelectPin);

    // read absolute angle [rad] from sensor, not limited to 0..360°.
    // first call after setup is normalized to 0..360°
    float readAngle();

    // return angle recently used
    float getAngle();

    // reset to angle between 0..360°
    void reset();
private:
	uint16_t selectPin = 0;
	uint32_t sensorRead(void);
	uint32_t readRegister(uint32_t register);
	float currentAngle = 0;
	uint16_t lastSensorRead = 0;
	static bool busInitialized;
};

#endif /* ENCODER_AS5047D_H_ */
