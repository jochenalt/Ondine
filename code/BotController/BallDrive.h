/*
 * BallEngine.h
 *
 * Takes three single wheels, adds kinematics and provides methods to
 * set and retrieve speed in terms of (x,y,omega)
 *
 *  Created on: 05.10.2018
 *      Author: JochenAlt
 */

#ifndef BALLDRIVE_H_
#define BALLDRIVE_H_

#include <Engine.h>
#include <MenuController.h>
#include <PowerRelay.h>
#include <types.h>

class BallDrive : public Menuable {
public:
	BallDrive() {};
	virtual ~BallDrive() {};

	void setup(MenuController* menuCtrl);
	void loop();

	void enable(bool doit) {
		engine.enable(doit);
	}

	void power(bool doit) {
		powerRelay.power(doit);
		delay(50); // await powering up, otherwise motors get PWM signals without having full power yet which gives a jerk
	}

	bool isPowered() {
		return powerRelay.isPowered();
	}

	bool isEnabled() { return engine.isEnabled(); };

	// set delta wheel angle to zero
	void reset() {
		engine.resetWheelAngleChange();
		lastCall_ms = millis();

		// set speed to zero
		float wheelSpeed[3] = {0,0,0};
		engine.setWheelSpeed(wheelSpeed);
	}

	void setSpeed(float speedX,float speedY, float omega, float angleX, float angleY);
	void getSpeed(float angleX, float angleY, float &speedX,float &speedY,float & omega);
	void getSetAngle(float &angleX, float &angleY);

	virtual void menuLoop(char ch, bool continously);
	virtual void printHelp();

	Engine engine;
	Kinematix kinematics;
	PowerRelay powerRelay;

private:
	uint32_t lastCall_ms = 0;	// used by getSpeed to compute time since last call

	// members used by the asci menu only
	float menuSpeedX = 0;
	float menuSpeedY = 0;
	float menuOmega = 0;
	float menuAngleX = 0;
	float menuAngleY = 0;

	float lastSetAngleX = 0;
	float lastSetAngleY = 0;

};

#endif /* BALLDRIVE_H_ */
