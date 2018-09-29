/*
 * Engine.h
 *
 *  Created on: 20.08.2018
 *      Author: JochenAlt
 */

#ifndef ENGINE_H_
#define ENGINE_H_

#include <BrushlessMotorDriver.h>
#include <MenuController.h>
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

	// get the change of angles since last invocation
	void getWheelAngleChange(float wheelAngleChange[3]);

	void setup(MenuController* menuCtrl);
	void loop();

	void enable(bool doit);
	bool isEnabled() { return enabled; };

	// return everage time [s] of an engine loop
	float getAvrLoopTime() { return ((float)averageTime_us)/1000000.0; };

	virtual void menuLoop(char ch);
	virtual void printHelp();
private:
	BrushlessMotorDriver* wheel[3] = { NULL, NULL, NULL };
	int activeMenuWheel = 0;
	uint32_t averageTime_us = 0;
	uint32_t lastLoop_ms = 0;
	float lastWheelAngle[3] = {0,0,0};
	bool enabled = false;
};

#endif /* ENGINE_H_ */
