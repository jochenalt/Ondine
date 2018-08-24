/*
 * BotController.cpp
 *
 *  Created on: 21.08.2018
 *      Author: JochenAlt
 */

#include <types.h>
#include <Util.h>

#include <BotController.h>
#include <BotMemory.h>


void BotController::setup() {
	registerMenuController(&menuController);
	engine.setup(&menuController);
	imu.setup(&menuController);
	state.setup(&menuController);
}

void BotController::printHelp() {
	command->println();
	command->println("Bot Menu");
	command->println();
	command->println("e - engine");
	command->println("m - save configuration to epprom");

}

void BotController::menuLoop(char ch) {
	bool cmd = true;
	switch (ch) {
	case 'e':
		engine.pushMenu();
		break;
	case 'm':
		memory.save();
		break;
	case 'h':
		printHelp();
		break;
	default:
		cmd = false;
		break;
	}
	if (cmd) {
		command->print(">");
	}
}

void BotController::loop() {
	engine.loop();
	menuController.loop();
	// imu.loop();

	// run main balance loop. Timing is 1determined by IMU sending an
	// interrupt that a new value is there.
	float dT = 0;
	if ((mode == BALANCE) && imu.isNewValueAvailable(dT)) {
		IMUSample& sensorSample = imu.getSample();

		// fetch motor encoder values to compute real wheel position
		float angleChange[3] = {0,0,0};
		engine.getWheelAngleChange(angleChange);

		float currentWheelSpeed[3] = {0,0,0};
		for (int w = 0;w<3;w++)
			currentWheelSpeed[w] = angleChange[w] * TWO_PI / dT;	// compute wheel speed out of delta-angle

		// apply inverse kinematics to get { speed (x,y), omega } out of wheel speed
		BotMovement currentMovement;
		kinematics.computeActualSpeed(  currentWheelSpeed,
										sensorSample.x.angle,sensorSample.y.angle,
										currentMovement.speedX, currentMovement.speedY, currentMovement.omega);

		// compute new movement out of current angle, angular velocity, velocity, position
		state.update(dT, currentMovement, sensorSample, targetBotMovement);


		// apply kinematics to compute wheel speed out of x,y, omega
		float  newWheelSpeed[3];
		kinematics.computeWheelSpeed(   state.getSpeedX(), state.getSpeedY(), state.getOmega(),
										sensorSample.x.angle,sensorSample.y.angle,
										newWheelSpeed);

		// send new speed to motors
		engine.setWheelSpeed(newWheelSpeed);
	}
}


