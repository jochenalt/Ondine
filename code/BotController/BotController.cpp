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
	logger->println();
	logger->println("Bot Menu");
	logger->println();
	logger->println("e - ball engine");
	logger->println("p - engine on/off");
	logger->println("s - state controller");
	logger->println("i - imu");
	logger->println("l - lifter");
	logger->println("p - power on/off");
	logger->println("b - balance on");
	logger->println("t - set trajectory");

	logger->println();
	logger->println("1 - performance log on");
	logger->println("2 - calibration log on");
	logger->println("3 - debug log on");
	logger->println("4 - state log on");

	logger->println("m - save configuration to epprom");
	logger->println("M - reset to factory settings");
}

void BotController::powerEngine(bool doIt) {
	if (doIt) {
		imu.enable(false);
		ballDrive.power(true);
		if (ballDrive.isPowered()) {
			// relay has been turned on successfully
			imu.enable(true);
			ballDrive.enable(true);
			if (!ballDrive.isEnabled())
				ballDrive.power(false);
		} //else
		imu.enable(true);

	} else {
		ballDrive.enable(false);
		imu.enable(false);
		ballDrive.power(false);
		imu.enable(true);

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
		logger->print(">");
	}
}

void BotController::setTarget(const BotMovement& target) {
	targetBotMovement = target;
}

void BotController::loop() {
	// performance measurement
	uint32_t start_us = micros();

	// drive motors
	ballDrive.loop(start_us);

	// react on serial line
	menuController.loop();

	// drive the lifter
	lifter.loop();

	// check if new IMU orientation is there
	imu.loop(start_us);

	// run main balance loop. Timing is determined by IMU that sends an
	// interrupt everytime a new value is there.
	float dT = 0; // set by isNewValueAvailable
	if ((mode == BALANCING) && imu.isNewValueAvailable(dT)) {

			// apply inverse kinematics to get { speed (x,y), omega } out of wheel speed
		IMUSample sensorSample = imu.getSample();
		ballDrive.getSpeed(start_us, sensorSample,currentMovement);

		// call balance and speed controller
		state.update(dT, sensorSample, currentMovement, targetBotMovement);

		// apply kinematics to compute wheel speed out of x,y, omega
		// and set speed of each wheel
		ballDrive.setSpeed( state.getSpeedX(), state.getSpeedY(), state.getOmega(),
								sensorSample.plane[Dimension::X].angle,sensorSample.plane[Dimension::Y].angle);


		uint32_t end_us= micros();
		avrLoopTime_us = ((end_us-start_us) + avrLoopTime_us)/2.0;

			if (logTimer.isDue_ms(100,millis())) {
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
					logging(state.getTiltErrorX(),2,3);
					logging(",");
					logging(state.getPosErrorX(),2,3);
					logging(",");
					logging(state.getSpeedX(),2,3);
					logging(",");
					logging(state.getAccelX(),2,3);
					logging("|");
					logging(state.getTiltErrorY(),2,3);
					logging(",");
					logging(state.getPosErrorY(),2,3);
					logging(",");
					logging(state.getSpeedY(),2,3);
					logging(",");
					logging(state.getAccelY(),2,3);
					logging("|");
					logging(state.getOmega(),2,3);
					logging(")");
				}
				if (memory.persistentMem.logConfig.performanceLog) {
					logger->print(" f=");
					logger->print(1.0/dT,0);
					logger->print("Hz, ");
					logger->print(avrLoopTime_us);
					logger->print("us, cpu=");
					logger->print(((float)(avrLoopTime_us*100.0) / SampleTime_us),0);
					logger->println("%)");
				}
			}
	} else
		delayMicroseconds(50); // ensure that next dT > 0

}



