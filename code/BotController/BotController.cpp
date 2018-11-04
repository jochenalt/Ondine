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
#include <TimePassedBy.h>

const int LifterEnablePin = 31;
const int LifterIn1Pin = 29;
const int LifterIn2Pin = 30;
const int LifterEncoderAPin = 35;
const int LifterEncoderBPin = 36;
const int LifterCurrentSensePin = A22;

const int LifterCPR = 48;

void BotController::setup() {
	registerMenuController(&menuController);
	ballDrive.setup(&menuController);
	imu.setup(&menuController);
	imu.setup();
	state.setup(&menuController);
	lifter.setup(&menuController);
	lifter.setupMotor(LifterEnablePin, LifterIn1Pin, LifterIn2Pin,LifterCurrentSensePin);
	lifter.setupEncoder(LifterEncoderAPin, LifterEncoderBPin, LifterCPR);

	performanceLogTimer.setRate(5000);
}

void BotController::printHelp() {
	command->println();
	command->println("Bot Menu");
	command->println();
	command->println("e - ball engine");
	command->println("p - engine on/off");
	command->println("s - state controller");
	command->println("i - imu");
	command->println("l - lifter");
	command->println("p - power on/off");
	command->println("b - balance on");

	command->println();
	command->println("1 - performance log on");
	command->println("2 - calibration log on");
	command->println("3 - debug log on");
	command->println("4 - state log on");

	command->println("m - save configuration to epprom");
}

void BotController::powerEngine(bool doIt) {
	if (doIt) {
		ballDrive.power(true);
		if (ballDrive.isPowered()) {
			// relay has been turned on successfully
			ballDrive.enable(true);
			if (!ballDrive.isEnabled())
				ballDrive.power(false);
		}
	} else {
		ballDrive.enable(false);
		ballDrive.power(false);
	}
}

bool BotController::isEnginePowered() {
	return ballDrive.isPowered() && ballDrive.isEnabled();
}

void BotController::menuLoop(char ch, bool continously) {
	bool cmd = true;
	switch (ch) {
	case 'b':
		balanceMode((mode==BALANCE)?OFF:BALANCE);
		break;
	case 'e':
		ballDrive.pushMenu();
		break;
	case 'm':
		memory.save();
		break;
	case 'd':
		memory.setDefaults();
		memory.println();

		break;
	case 'l':
		lifter.pushMenu();
		break;
	case 'i':
		imu.pushMenu();
		break;
	case 's':
		state.pushMenu();
		break;
	case 'p': {
		bool doIt = !isEnginePowered();
		logger->print("turn engine ");
		logger->println((doIt?"on":"off"));
		powerEngine(doIt);
		logger->print("engine is ");
		logger->println((isEnginePowered()?"on":"off"));
		break;
	}
	case '1':
		memory.persistentMem.logConfig.performanceLog = !memory.persistentMem.logConfig.performanceLog;
		break;
	case '2':
		memory.persistentMem.logConfig.calibrationLog = !memory.persistentMem.logConfig.calibrationLog;
		break;
	case '3':
		memory.persistentMem.logConfig.debugBalanceLog = !memory.persistentMem.logConfig.debugBalanceLog;
		break;
	case '4':
		memory.persistentMem.logConfig.debugStateLog = !memory.persistentMem.logConfig.debugStateLog;
		break;

	case 'h':
		printHelp();
		memory.println();
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
	// give other libraries some time
	yield();

	// drive motors
	ballDrive.loop();

	// react on serial line
	menuController.loop();

	// check if new IMU orientation is there
	imu.loop();

	ballDrive.loop();

	// drive the lifter
	lifter.loop();

	// run main balance loop. Timing is determined by IMU that sends an
	// interrupt everytime a new value is there.
	float dT = 0; // set by isNewValueAvailable
	if ((mode == BALANCE) && imu.isNewValueAvailable(dT)) {
		IMUSample sensorSample = imu.getSample();

		// apply inverse kinematics to get { speed (x,y), omega } out of wheel speed
		BotMovement currentMovement;
		ballDrive.getSpeed(sensorSample.plane[Dimension::X].angle, sensorSample.plane[Dimension::Y].angle,
				           currentMovement.speedX, currentMovement.speedY, currentMovement.omega);

		// call this as often as possible to get a smooth motor movement
		// the call above contains kinematics from wheel speed to cartesian speed, this takes just below 1ms
		ballDrive.loop();

		// compute new movement out of current angle, angular velocity, velocity, position
		state.update(dT, currentMovement, sensorSample, targetBotMovement);

		// call this as often as possible to get a smooth motor movement
		// (call above contains lots of computations)
		ballDrive.loop();

		// apply kinematics to compute wheel speed out of x,y, omega
		// and set speed of each wheel
		ballDrive.setSpeed( state.getSpeedX(), state.getSpeedY(), state.getOmega(),
				            sensorSample.plane[Dimension::X].angle,sensorSample.plane[Dimension::Y].angle);

		// call this as often as possible to get a smooth motor movement
		// (call above takes just below 1ms, it contains kinematics from cartesian speed to wheel speed)
		ballDrive.loop();

		if (logTimer.isDue_ms(1000,millis())) {
			if (memory.persistentMem.logConfig.debugBalanceLog) {
				logger->print("a=(");
				logger->print(degrees(sensorSample.plane[Dimension::X].angle));
				logger->print(",");
				logger->print(degrees(sensorSample.plane[Dimension::Y].angle));
				logger->print(") ");
				logger->print("a'=(");
				logger->print(degrees(sensorSample.plane[Dimension::X].angularVelocity));
				logger->print(",");
				logger->print(degrees(sensorSample.plane[Dimension::Y].angularVelocity));
				logger->print(") ");

				logger->print("v=(");
				logger->print(currentMovement.speedX);
				logger->print(",");
				logger->print(currentMovement.speedY);
				logger->print(") ");
				logger->print(" state=(");
				logger->print(state.getSpeedX());
				logger->print(",");
				logger->print(state.getSpeedY());
				logger->print(")");
			}
			if (memory.persistentMem.logConfig.performanceLog) {
				logger->print(" t=(dT=");
				logger->print(dT*1000.0);
				logger->print("ms,imu=");
				logger->print(imu.getAvrLoopTime()*1000.0);
				logger->print("ms,state=");
				logger->print(state.getAvrLoopTime()*1000.0);
				logger->print("ms,eng=");
				logger->print(ballDrive.getAvrLoopTime()*1000.0);
				logger->print("ms");
				logger->println(")");
			}
		}
	}
}



