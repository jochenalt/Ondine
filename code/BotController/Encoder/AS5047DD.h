/*
 * AS5047D.h
 *
 *  Created on: 06.11.2018
 *      Author: JochenAlt
 */

#ifndef ENCODER_AS5047DD_H_
#define ENCODER_AS5047DD_H_

#include "Arduino.h"

class AS5047D
{
	const int resolution = 2048;

public:
                AS5047D(uint16_t mosiPin, uint16_t misoPin, uint16_t SCKPin, uint16_t SelectPin);

                // read current angle from sensor
                float readAngle();

                // return angle recently used
                float getAngle();

        private:
                const uint16_t selectPin;
                uint32_t sensorRead(void);
                uint32_t readRegister(uint32_t register);
                float currentAngle = 0;
                uint16_t lastSensorRead = 0;
};

#endif /* ENCODER_AS5047DD_H_ */
