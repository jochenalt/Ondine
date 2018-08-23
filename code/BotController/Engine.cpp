/*
 * Engine.cpp
 *
 *  Created on: 20.08.2018
 *      Author: JochenAlt
 */

#include <MenuController.h>
#include <Engine.h>


#define W1_L6234_ENABLE_PIN 2
#define W1_L6234_PWM1 3
#define W1_L6234_PWM2 4
#define W1_L6234_PWM3 5
#define W1_ENCODERA_PIN 6
#define W1_ENCODERB_PIN 7

#define W2_L6234_ENABLE_PIN 8
#define W2_L6234_PWM1 9
#define W2_L6234_PWM2 10
#define W2_L6234_PWM3 11
#define W2_ENCODERA_PIN 12
#define W2_ENCODERB_PIN 13

#define W3_L6234_ENABLE_PIN 14
#define W3_L6234_PWM1 15
#define W3_L6234_PWM2 16
#define W3_L6234_PWM3 17
#define W3_ENCODERA_PIN 18
#define W3_ENCODERB_PIN 19

void Engine::setup() {
	// register out menu
	registerMenuController(menuCtrl);

	// initialize all brushless motors and their encoders
	wheel[0] = new OmniWheel();
	wheel[0]->registerMenuController(menuCtrl);
	wheel[0]->setupMotor(W1_L6234_ENABLE_PIN, W1_L6234_PWM1, W1_L6234_PWM2, W1_L6234_PWM3);
	wheel[0]->setupEncoder(W1_ENCODERA_PIN, W1_ENCODERB_PIN, 1024);

	wheel[1] = new OmniWheel();
	wheel[1]->registerMenuController(menuCtrl);
	wheel[1]->setupMotor(W2_L6234_ENABLE_PIN, W2_L6234_PWM1, W2_L6234_PWM2, W2_L6234_PWM3);
	wheel[1]->setupEncoder(W2_ENCODERA_PIN, W2_ENCODERB_PIN, 1024);
	menuCtrl->registerMenu(wheel[1]);

	wheel[2] = new OmniWheel();
	wheel[2]->registerMenuController(menuCtrl);
	wheel[2]->setupMotor(W3_L6234_ENABLE_PIN, W3_L6234_PWM1, W3_L6234_PWM2, W3_L6234_PWM3);
	wheel[2]->setupEncoder(W3_ENCODERA_PIN, W3_ENCODERB_PIN, 1024);
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
	wheel[0]->setSpeed(revPerSec[0]);
	wheel[1]->setSpeed(revPerSec[1]);
	wheel[2]->setSpeed(revPerSec[2]);
}

// get angle of all wheels since invocation of resetWheelAngle
void Engine::getIntegratedWheelAngle(float wheelAngle[3]) {
	wheelAngle[0] = wheel[0]->getIntegratedAngle();
	wheelAngle[1] = wheel[1]->getIntegratedAngle();
	wheelAngle[2] = wheel[2]->getIntegratedAngle();

}

void Engine::printHelp() {
	Serial1.println("Engine controller");
	Serial1.println("0 - set wheel 0");
	Serial1.println("1 - set wheel 1");
	Serial1.println("2 - set wheel 2");

	Serial1.println("ESC");
}

void Engine::menuLoop(char ch) {
	bool cmd = true;
	switch (ch) {
	case '0':
		activeMenuWheel = 0;
		wheel[0]->activateMenu();
		break;
	case '1':
		activeMenuWheel = 1;
		wheel[1]->activateMenu();
		break;
	case '2':
		activeMenuWheel = 2;
		wheel[2]->activateMenu();
		break;
	case 'h':
		printHelp();
		break;
	case 27:
		deactivateMenu();
		return;
		break;
	default:
		cmd = false;
		break;
	}
	if (cmd) {
		Serial1.print("loop t=");
		Serial1.print(averageTime_ms);
		Serial1.print("ms");

		Serial1.print(" active wheel");
		Serial1.print(activeMenuWheel);

		Serial1.print(" angle=(");
		Serial1.print(degrees(wheel[0]->getIntegratedAngle()));
		Serial1.print(",");
		Serial1.print(degrees(wheel[1]->getIntegratedAngle()));
		Serial1.print(",");
		Serial1.print(degrees(wheel[2]->getIntegratedAngle()));
		Serial1.print(")");

		Serial1.print(" speed=(");
		Serial1.print((wheel[0]->getSpeed()));
		Serial1.print(",");
		Serial1.print((wheel[1]->getSpeed()));
		Serial1.print(",");
		Serial1.print((wheel[2]->getSpeed()));
		Serial1.print(")");

		Serial1.println(" >");
	}
}



