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

const int LifterEnablePin = 31;
const int LifterIn1Pin = 30;
const int LifterIn2Pin = 29;
const int LifterEncoderAPin = 35;
const int LifterEncoderBPin = 36;
const int LifterCurrentSensePin = A22;

const int LifterCPR = 48;

void BotController::setup() {
	registerMenuController(&menuController);
	power.setup();
	engine.setup(&menuController);
	imu.setup(&menuController);
	imu.setup();
	state.setup(&menuController);
	lifter.setup(&menuController);
	lifter.setupMotor(LifterEnablePin, LifterIn1Pin, LifterIn2Pin,LifterCurrentSensePin);
	lifter.setupEncoder(LifterEncoderAPin, LifterEncoderBPin, LifterCPR);
}

void BotController::printHelp() {
	command->println();
	command->println("Bot Menu");
	command->println();
	command->println("e - engine");
	command->println("i - imu");
	command->println("l - lifter");
	command->println("p - power on/off");
	command->println("b - balance on");

	command->println("m - save configuration to epprom");

}

void BotController::menuLoop(char ch) {
	bool cmd = true;
	switch (ch) {
	case 'b':
		mode = BALANCE;
		break;
	case 'p':
		if (power.isMotorOn()) {
			command->println("turning motor power off");
			power.motorPower(false);
		} else {
			command->println("turning motor power on");
			power.motorPower(true);
		}
		break;
	case 'e':
		engine.pushMenu();
		break;
	case 's':
		memory.save();
		break;
	case 'l':
		lifter.pushMenu();
		break;
	case 'i':
		imu.pushMenu();
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
	uint32_t now = micros();

	engine.loop();
	uint32_t engineTime = micros()-now;

	menuController.loop();
	imu.loop();
	lifter.loop();
	uint32_t imuTime = micros()-now;

	// run main balance loop. Timing is 1determined by IMU sending an
	// interrupt that a new value is there.
	float dT = 0;
	if ((mode == BALANCE) && imu.isNewValueAvailable(dT)) {
		IMUSample sensorSample = imu.getSample();


		// fetch motor encoder values to compute real wheel position
		float angleChange[3] = {0,0,0};
		engine.getWheelAngleChange(angleChange);

		float currentWheelSpeed[3] = {0,0,0};
		currentWheelSpeed[0] = angleChange[0] * TWO_PI / dT;	// compute wheel speed out of delta-angle
		currentWheelSpeed[1] = angleChange[1] * TWO_PI / dT;
		currentWheelSpeed[2] = angleChange[2] * TWO_PI / dT;

		// apply inverse kinematics to get { speed (x,y), omega } out of wheel speed
		BotMovement currentMovement;
		kinematics.computeActualSpeed(  currentWheelSpeed,
										sensorSample.x.angle,sensorSample.y.angle,
										currentMovement.speedX, currentMovement.speedY, currentMovement.omega);
		// compute new movement out of current angle, angular velocity, velocity, position
		state.update(dT, currentMovement, sensorSample, targetBotMovement);

		// apply kinematics to compute wheel speed out of x,y, omega
		float  newWheelSpeed[3] = { 0, 0, 0};
		kinematics.computeWheelSpeed(   state.getSpeedX(), state.getSpeedY(), state.getOmega(),
										sensorSample.x.angle,sensorSample.y.angle,
										newWheelSpeed);

		// send new speed to motors
		engine.setWheelSpeed(newWheelSpeed);
		uint32_t balanceTime = micros()-now;

		logger->print("te=");
		logger->print(engineTime);
		logger->print("ti=");
		logger->print(imuTime);
		logger->print("tb=");
		logger->print(balanceTime);
		logger->println();
	}

}


