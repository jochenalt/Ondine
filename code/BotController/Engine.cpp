/*
 * Engine.cpp
 *
 *  Created on: 20.08.2018
 *      Author: JochenAlt
 */

#include <libraries/Util.h>
#include <libraries/MenuController.h>
#include <Engine.h>
#include <setup.h>


void Engine::setup(MenuController* menuCtrl) {
	// register out menu
	registerMenuController(menuCtrl);

	// initialize all brushless motors and their encoders
	// unfortunately,sequence of motors in schematics is different than sequence of motors used in code
	static int MotorIdx [3] =  { 0,1,2 };

	// array that assigns the motors pins to the right order
	static bool ReverseDirection[3] = { true, true, true };
	for (int i = 0;i<3;i++) {
		int idx = MotorIdx[i];
		wheel[i].setup(i, menuCtrl, ReverseDirection[idx]); // first call initializes the SPI bus
		wheel[i].setupMotor(BRUSHLESS_DRIVER_ENABLE_PIN, BrushlessDriverPWMPins[idx][0], BrushlessDriverPWMPins[idx][1], BrushlessDriverPWMPins[idx][2]);
		wheel[i].setupEncoder(SS_PIN[idx]);
	}
}

void Engine::loop() {
	uint32_t start = millis();
	for (int i = 0;i<3;i++) {
		wheel[i].loop();
	}
	uint32_t end = millis();
	averageTime_ms = (averageTime_ms + (end - start)) / 2;
}

void Engine::setWheelSpeed(float revPerSec[3]) {
	for (int i = 0;i<3;i++)
		wheel[i].setSpeed(revPerSec[i]);
}

void Engine::getWheelSpeed(float revPerSec[3]) {
	for (int i = 0;i<3;i++)
		revPerSec[i] = wheel[i].getSpeed();
}

// get angle of all wheels since invocation of resetWheelAngle
void Engine::getIntegratedWheelAngle(float wheelAngle[3]) {
	for (int i = 0;i<3;i++)
		wheelAngle[i] = wheel[i].getIntegratedAngle();
}

void Engine::resetWheelAngleChange() {
	float wheelAngleChange[3];
	getWheelAngleChange(wheelAngleChange);
	wheel[0].reset();
	wheel[1].reset();
	wheel[2].reset();
}

void Engine::getWheelAngleChange(float wheelAngleChange[3]) {
	for (int i = 0;i<3;i++) {
		float angle = wheel[i].getIntegratedAngle();
		wheelAngleChange[i] = angle - lastWheelAngle[i];
		lastWheelAngle[i] = angle;
	}
}
void Engine::enable(bool doIt) {
	if (doIt) {
		bool ok = true;
		for (int i = 0;i<3;i++) {
			wheel[i].enable(true);
			if (!wheel[i].isEnabled()) {
				ok = false;
				logging("enable wheel ");
				log(i);
				logging(" failed!");
			}
		}
		if (ok)
			enabled = true;
		else {
			wheel[0].enable(false);
			wheel[1].enable(false);
			wheel[2].enable(false);
			enabled = false;
		}
	} else {
		wheel[0].enable(false);
		wheel[1].enable(false);
		wheel[2].enable(false);
		enabled = false;
	}
}

void Engine::printHelp() {
	loggingln();
	loggingln("Engine Menu");
	loggingln();
	loggingln("e - enable");
	loggingln("0 - set wheel 0");
	loggingln("1 - set wheel 1");
	loggingln("2 - set wheel 2");

	loggingln("ESC");
}

void Engine::menuLoop(char ch, bool continously) {
	bool cmd = true;
	switch (ch) {
	case '0':
		activeMenuWheel = 0;
		wheel[0].pushMenu();
		break;
	case '1':
		activeMenuWheel = 1;
		wheel[1].pushMenu();
		break;
	case '2':
		activeMenuWheel = 2;
		wheel[2].pushMenu();
		break;
	case'e':
		enable(!enabled);
		break;
	case 'h':
		printHelp();
		break;
	default:
		cmd = false;
		break;
	}
	if (cmd) {
		if (enabled)
			logging("enabled.");
		else
			logging("disabled.");
		logging("loop t=");
		log(averageTime_ms);
		logging("ms");

		logging(" active wheel");
		log(activeMenuWheel);

		logging(" angle=(");
		logging(degrees(wheel[0].getIntegratedAngle()),4,0);
		logging(",");
		logging(degrees(wheel[1].getIntegratedAngle()),4,0);
		logging(",");
		logging(degrees(wheel[2].getIntegratedAngle()),4,0);
		logging(")");

		logging(" speed=(");
		logging((wheel[0].getSpeed()),2,3);
		logging(",");
		logging((wheel[1].getSpeed()),2,3);
		logging(",");
		logging((wheel[2].getSpeed()),2,3);
		logging(")");

		loggingln(" >");
	}
}



