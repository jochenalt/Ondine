/*
 * Power.h
 *
 *  Created on: 22.09.2018
 *      Author: JochenAlt
 */

#ifndef POWER_H_
#define POWER_H_

class Power {
public:
	Power() {};
	virtual ~Power() {};
	void motorPower(bool on);
	bool isMotorOn();

	void setup();
private:
	bool motorOn = false;
};

#endif /* POWER_H_ */
