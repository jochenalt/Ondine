/*
 * Engine.cpp
 *
 *  Created on: 20.08.2018
 *      Author: JochenAlt
 */

#include <Util.h>
#include <MenuController.h>
#include <Engine.h>


// Pins for Drotek L6234 breakout
//                             PWM1,PWM2,PWM3
const int WheelPins[3][3] = { { 2,    3,   4},
                              { 5,    6,   7},
							  { 8,    9,   10}};

//                                ENCA, ENCB
const int EncoderPins[3][2] = { { 11,   12 },
                                { 25,   26 },
							    { 27,   28 }};

const int EnablePin = 24;


void Engine::setup(MenuController* menuCtrl) {
	// register out menu
	registerMenuController(menuCtrl);

	// initialize all brushless motors and their encoders
	for (int i = 0;i<3;i++) {
		wheel[i] = new OmniWheel();
		wheel[i]->setup(menuCtrl);
		wheel[i]->setupMotor(EnablePin, WheelPins[i][0], WheelPins[i][1], WheelPins[i][2]);
		wheel[i]->setupEncoder(EncoderPins[i][0],EncoderPins[i][1], 1024);
	}
}

void Engine::loop() {
	uint32_t start = millis();
	wheel[0]->loop();
	wheel[1]->loop();
	wheel[2]->loop();
	uint32_t end = millis();

	uint32_t duration_ms = end - start;
	averageTime_ms += duration_ms;
	averageTime_ms /= 2;
}

void Engine::setWheelSpeed(float revPerSec[3]) {
	for (int i = 0;i<3;i++)
		wheel[i]->setSpeed(revPerSec[i]);
}

// get angle of all wheels since invocation of resetWheelAngle
void Engine::getIntegratedWheelAngle(float wheelAngle[3]) {
	for (int i = 0;i<3;i++)
		wheelAngle[i] = wheel[i]->getIntegratedAngle();
}

void Engine::getWheelAngleChange(float wheelAngleChange[3]) {
	for (int i = 0;i<3;i++) {
		float angle = wheel[i]->getIntegratedAngle();
		wheelAngleChange[i] = angle - lastWheelAngle[i];
		lastWheelAngle[i] = angle;
	}
}


void Engine::printHelp() {
	command->println();
	command->println("Engine Menu");
	command->println();
	command->println("0 - set wheel 0");
	command->println("1 - set wheel 1");
	command->println("2 - set wheel 2");

	command->println("ESC");
}

void Engine::menuLoop(char ch) {
	bool cmd = true;
	switch (ch) {
	case '0':
		activeMenuWheel = 0;
		wheel[0]->pushMenu();
		break;
	case '1':
		activeMenuWheel = 1;
		wheel[1]->pushMenu();
		break;
	case '2':
		activeMenuWheel = 2;
		wheel[2]->pushMenu();
		break;
	case 'h':
		printHelp();
		break;
	default:
		cmd = false;
		break;
	}
	if (cmd) {
		command->print("loop t=");
		command->print(averageTime_ms);
		command->print("ms");

		command->print(" active wheel");
		command->print(activeMenuWheel);

		command->print(" angle=(");
		command->print(degrees(wheel[0]->getIntegratedAngle()));
		command->print(",");
		command->print(degrees(wheel[1]->getIntegratedAngle()));
		command->print(",");
		command->print(degrees(wheel[2]->getIntegratedAngle()));
		command->print(")");

		command->print(" speed=(");
		command->print((wheel[0]->getSpeed()));
		command->print(",");
		command->print((wheel[1]->getSpeed()));
		command->print(",");
		command->print((wheel[2]->getSpeed()));
		command->print(")");

		command->println(" >");
	}
}



