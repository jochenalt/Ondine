/*
 * BallEngine.h
 *
 *  Created on: 05.10.2018
 *      Author: JochenAlt
 */

#ifndef BALLDRIVE_H_
#define BALLDRIVE_H_

#include <Engine.h>
#include <MenuController.h>
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
	bool isEnabled() { return engine.isEnabled(); };

	// set delta wheel angle to zero
	void reset() {
		engine.resetWheelAngleChange();
		lastCall_ms = millis();
	}

	void setSpeed(float speedX,float speedY, float omega, float angleX, float angleY);
	void getSpeed(float angleX, float angleY, float &speedX,float &speedY,float & omega);

	virtual void menuLoop(char ch);
	virtual void printHelp();

	Engine engine;
	Kinematix kinematics;

private:
	uint32_t lastCall_ms = 0;

	float menuSpeedX = 0;
	float menuSpeedY = 0;
	float menuOmega = 0;
	float menuAngleX = 0;
	float menuAngleY = 0;

};

#endif /* BALLDRIVE_H_ */
