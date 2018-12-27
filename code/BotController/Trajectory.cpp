/*
 * BallEngine.cpp
 *
 *  Created on: 05.10.2018
 *      Author: JochenAlt
 */

#include <types.h>
#include <Trajectory.h>
#include <libraries/Util.h>
#include <BotController.h>

void Trajectory::setup(MenuController* menuCtrl) {
	// register out menu
	registerMenuController(menuCtrl);

	current.x.accel = MaxBotAccel;
	current.y.accel = MaxBotAccel;
}


void Trajectory::setAcceleration(float accel) {
	current.x.accel = accel;
	current.y.accel = accel;
}


void Trajectory::setOdom(const Pose& pose, uint32_t approachingTime, const Speed& targetSpeed) {
	this->targetPose = pose;
	this->targetTime = approachingTime;
	this->targetSpeed = targetSpeed;
}

void Trajectory::printHelp() {
	logger->println();
	logger->println("Trajectory");
	logger->println();

	logger->println("0 - stop");

	logger->println("ESC");
}

void Trajectory::loop() {
	uint32_t now = millis();
	if (lastLoopTime > 0) {
		float dT = ((float)(now - lastLoopTime))/1000.0;

		// end of target, set target speed, no acceleration
		if (millis() > targetTime ) {
			current.x.speed = targetSpeed.x;
			current.y.speed = targetSpeed.y;
			current.x.accel= 0;
			current.y.accel = 0;
			targetTime = 0;
			targetSpeed = Speed(0,0);
			targetPose = Pose(0,0,0);
		}

		// continue trajectory with constant speed
		if (targetTime == 0) {
			current.x.pos += current.x.speed * dT;
			current.y.pos += current.y.speed * dT;
		}

		// @TODO implement trapezoid profile towards target
		BotController::getInstance()->setTarget(current);
	}

	lastLoopTime = now;
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



