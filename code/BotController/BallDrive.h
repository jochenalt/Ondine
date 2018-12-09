/*
 * BallEngine.h
 *
 * Takes three single wheels, adds kinematics and provides methods to
 * set and retrieve speed in terms of (x,y,omega).
 *
 *
 *  Created on: 05.10.2018
 *      Author: JochenAlt
 */

#ifndef BALLDRIVE_H_
#define BALLDRIVE_H_

#include <Engine.h>
#include <libraries/MenuController.h>
#include <PowerRelay.h>
#include <types.h>
#include <IMU.h>

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
		delay(100); // await powering up, otherwise motors get PWM signals without having full power yet which gives a jerk
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

	// Set the speed of the ball drive in terms of a cartesic coord system.
	// Considers the tilt angle in computation
	void setSpeed(float speedX /* [m/s] */,float speedY /* [m/s] */, float omega, float angleX, float angleY);

	// return speed as measured by encoders (might be different from speed set in method above)
	void getSpeed(const IMUSample &sample,BotMovement &current);

	// return tilt angles as set in setSpeed
	void getSetAngle(float &angleX, float &angleY);

	// ascii menu, menu commands are implemented there
	virtual void menuLoop(char ch, bool continously);

	// print ascii help to console
	virtual void printHelp();

	float getAvrLoopTime() { return engine.getAvrLoopTime(); };

private:
	Engine engine;				// three independent motors
	Kinematix kinematics;		// computation of speedx/speedy/omega into wheel speed
	PowerRelay powerRelay;		// turn on/off power for motors

	uint32_t lastCall_ms = 0;	// used by getSpeed to compute time since last call
	float lastSpeedX = 0;		// used by getSpeed
	float lastSpeedY = 0;

	// current movement of bot in terms of position and speed
	BotMovement menuMovement;

	// members used by the ascii menu only
	float menuSpeedX = 0;
	float menuSpeedY = 0;
	float menuOmega = 0;
	float menuAngleX = 0;
	float menuAngleY = 0;
};

#endif /* BALLDRIVE_H_ */
