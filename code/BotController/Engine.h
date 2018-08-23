/*
 * Engine.h
 *
 *  Created on: 20.08.2018
 *      Author: JochenAlt
 */

#ifndef ENGINE_H_
#define ENGINE_H_

#include <MenuController.h>
#include <OmniWheel.h>
#include <Kinematics.h>

// Engine combines three wheels and allows to pass speed commands to all of them
// Accumulates the movement as according to the wheel

class Engine : public Menuable  {
public:
	Engine() {};
	virtual ~Engine() {};

	// set target speed of all wheels in revolutions per seconds, accelerate as quick as possible
	void setWheelSpeed(float revPerSec[3]);

	// get angle of all wheels. Accumulates when turning
	void getIntegratedWheelAngle(float wheelAngle[3]);

	void setup();
	void loop();
	virtual void menuLoop(char ch);
	virtual void printHelp();
private:
	OmniWheel* wheel[3] = { NULL, NULL, NULL };
	int activeMenuWheel = 0;
	uint32_t averageTime_ms = 0;
	uint32_t lastLoop_ms = 0;
};

#endif /* ENGINE_H_ */
