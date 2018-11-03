/*
 * Power.h
 *
 *  Created on: 22.09.2018
 *      Author: JochenAlt
 */

#ifndef POWERRELAY_H_
#define POWERRELAY_H_

class PowerRelay {
public:
	PowerRelay() {};
	virtual ~PowerRelay() {};
	void power(bool on);
	bool isPowered();

	void setup();
private:
	bool motorOn = false;
};

#endif /* POWERRELAY_H_ */
