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

void Engine::setup(MenuController& newMenuCtrl) {
	menuCtrl = &newMenuCtrl;
	menuCtrl->registerMenu(this);

	ctrl[0] = new BLDCController();
	menuCtrl->registerMenu(ctrl[0]);
	ctrl[0]->setupMotor(W1_L6234_ENABLE_PIN, W1_L6234_PWM1, W1_L6234_PWM2, W1_L6234_PWM3);
	ctrl[0]->setupEncoder(W1_ENCODERA_PIN, W1_ENCODERB_PIN, 1024);

	ctrl[1] = new BLDCController();
	menuCtrl->registerMenu(ctrl[1]);
	ctrl[1]->setupMotor(W2_L6234_ENABLE_PIN, W2_L6234_PWM1, W2_L6234_PWM2, W2_L6234_PWM3);
	ctrl[1]->setupEncoder(W2_ENCODERA_PIN, W2_ENCODERB_PIN, 1024);
	menuCtrl->registerMenu(ctrl[1]);

	ctrl[2] = new BLDCController();
	menuCtrl->registerMenu(ctrl[2]);
	ctrl[2]->setupMotor(W3_L6234_ENABLE_PIN, W3_L6234_PWM1, W3_L6234_PWM2, W3_L6234_PWM3);
	ctrl[2]->setupEncoder(W3_ENCODERA_PIN, W3_ENCODERB_PIN, 1024);
}

void Engine::loop() {
	ctrl[0]->loop();
	ctrl[1]->loop();
	ctrl[2]->loop();
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
		menuCtrl->activateMenu(ctrl[0]);
		break;
	case '1':
		activeMenuWheel = 1;
		menuCtrl->activateMenu(ctrl[1]);
		break;
	case '2':
		activeMenuWheel = 2;
		menuCtrl->activateMenu(ctrl[2]);
		break;
	case 'h':
		printHelp();
		break;
	case 27:
		return;
		break;
	default:
		cmd = false;
		break;
	}
	if (cmd) {
		Serial1.print("active wheel");
		Serial1.print(activeMenuWheel);
		Serial1.println(" >");
	}
}



