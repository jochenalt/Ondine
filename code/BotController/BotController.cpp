/*
 * BotController.cpp
 *
 *  Created on: 21.08.2018
 *      Author: JochenAlt
 */

#include <BotController.h>


void BotController::setup() {
	engine.registerMenuController(&menuController);
	engine.setup();
}

void BotController::printHelp() {
	Serial1.println("Main controller");
	Serial1.println("e - engine Controller");
}

void BotController::menuLoop(char ch) {
	bool cmd = true;
	switch (ch) {
	case 'e':
		engine.activateMenu();
		break;
	case 'h':
		printHelp();
		break;
	default:
		cmd = false;
		break;
	}
	if (cmd) {
		Serial1.print("active wheel");
		Serial1.println(" >");
	}
}

void BotController::loop() {
	engine.loop();
	menuController.loop();

}


