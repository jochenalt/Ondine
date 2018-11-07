/*
 * BallEngine.cpp
 *
 *  Created on: 05.10.2018
 *      Author: JochenAlt
 */

#include <types.h>
#include <Trajectory.h>
#include <Util.h>

void Trajectory::setup(MenuController* menuCtrl) {
	// register out menu
	registerMenuController(menuCtrl);
}


void Trajectory::printHelp() {
	logger->println();
	logger->println("Trajectory");
	logger->println();

	logger->println("0 - stop");

	logger->println("ESC");
}

void Trajectory::loop() {
}

void Trajectory::menuLoop(char ch, bool continously) {
	bool cmd = true;
	switch (ch) {
	case '0':
		setSpeed(Speed3D(0,0,0));
		break;
	default:
		cmd = false;
		break;
	}
	if (cmd) {
	}
}



