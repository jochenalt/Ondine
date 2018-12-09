/*
 * BotController.cpp
 *
 *  Created on: 21.08.2018
 *      Author: JochenAlt
 */

#include <AS5047D.h>
#include <types.h>
#include <libraries/Util.h>

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
	command->println("t - set trajectory");

	command->println();
	command->println("1 - performance log on");
	command->println("2 - calibration log on");
	command->println("3 - debug log on");
	command->println("4 - state log on");

	command->println("m - save configuration to epprom");
	command->println("M - reset to factory settings");
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
		balanceMode((mode==BALANCING)?OFF:BALANCING);
		break;
	case 'e':
		ballDrive.pushMenu();
		break;
	case 'm':
		memory.save();
		logger->println("EEPROM saved");
		break;
	case 'M':
		memory.setDefaults();
		logger->println("EPPROM reset to factory settings");
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
		loggingln();
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

void BotController::setTarget(const BotMovement& target) {
	targetBotMovement = target;
}


void BotController::loop() {
	// performance measurement
	uint32_t start = micros();

	// give other libraries some time
	yield();

	// drive motors
	ballDrive.loop();

	// react on serial line
	menuController.loop();

	// check if new IMU orientation is there
	imu.loop();
	IMUSample sensorSample = imu.getSample();

	ballDrive.loop();

	// drive the lifter
	lifter.loop();

	// run main balance loop. Timing is determined by IMU that sends an
	// interrupt everytime a new value is there.
	float dT = 0; // set by isNewValueAvailable
	if ((mode == BALANCING) && imu.isNewValueAvailable(dT)) {

		// apply inverse kinematics to get { speed (x,y), omega } out of wheel speed
		ballDrive.getSpeed(sensorSample,currentMovement);

		// call this as often as possible to get a smooth motor movement
		ballDrive.loop();

		// call balance and speed controller
		state.update(dT, sensorSample, currentMovement, targetBotMovement);

		// call this as often as possible to get a smooth motor movement
		ballDrive.loop();

		// apply kinematics to compute wheel speed out of x,y, omega
		// and set speed of each wheel
		ballDrive.setSpeed( state.getSpeedX(), state.getSpeedY(), state.getOmega(),
				            sensorSample.plane[Dimension::X].angle,sensorSample.plane[Dimension::Y].angle);

		uint32_t end = micros();
		avrLoopTime = (((float)(end-start))/1000000.0 + avrLoopTime)/2.0;

		if (logTimer.isDue_ms(200,millis())) {
			if (memory.persistentMem.logConfig.debugBalanceLog) {
				logging("a=(");
				logging(degrees(sensorSample.plane[Dimension::X].angle),3,1);
				logging(",");
				logging(degrees(sensorSample.plane[Dimension::X].angularVelocity),3,1);
				logging(",");
				logging(currentMovement.x.pos,2,3);
				logging(",");
				logging(currentMovement.x.speed,2,3);
				logging("|");
				logging(degrees(sensorSample.plane[Dimension::Y].angle),3,1);
				logging(",");
				logging(degrees(sensorSample.plane[Dimension::Y].angularVelocity),3,1);
				logging(",");
				logging(currentMovement.y.pos,2,3);
				logging(",");
				logging(currentMovement.y.speed,2,3);
				logging(") ");
				// currentMovement.print();
				logging(" state=(");
				logging(state.getSpeedX(),2,3);
				logging(",");
				logging(state.getAccelX(),2,3);
				logging("|");
				logging(state.getSpeedY(),2,3);
				logging(",");
				logging(state.getAccelY(),2,3);
				logging("|");
				logging(state.getOmega(),2,3);
				logging(")");
			}
			if (memory.persistentMem.logConfig.performanceLog) {
				logger->print(" t=(dT=");
				logger->print(dT*1000.0);
				logger->print("ms,state=");
				logger->print(state.getAvrLoopTime()*1000.0);
				logger->print("ms,eng=");
				logger->print(ballDrive.getAvrLoopTime()*1000.0);
				logger->print("ms, cpu=");
				logger->print((avrLoopTime / SamplingTime) * 100.0,0);
				logger->println("%)");
			}
		}

	}
}



