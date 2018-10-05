/*
 * BallEngine.cpp
 *
 *  Created on: 05.10.2018
 *      Author: JochenAlt
 */

#include <BallDrive.h>
#include <types.h>

void BallDrive::setup(MenuController* menuCtrl) {
	// register out menu
	registerMenuController(menuCtrl);
	engine.setup(menuCtrl);
	kinematics.setup();
	reset();
}


void BallDrive::printHelp() {
	command->println();
	command->println("Ball Drive");
	command->println();
	command->println("e - enable");
	command->println("q/a - modify x speed");
	command->println("w/s - modify y speed");
	command->println("y/x - modify omega speed");
	command->println("o/l - modify tilt x");
	command->println("i/k - modify tilt y");

	command->println("b - single wheel control");

	command->println("0 - nullify speed");

	command->println("ESC");
}

void BallDrive::setSpeed(float speedX,float speedY, float omega,
		 	 	 	 	 float angleX, float angleY) {

	// apply kinematics to compute wheel speed out of x,y, omega
	float  newWheelSpeed[3] = { 0, 0, 0};
	kinematics.computeWheelSpeed(speedX, speedY, omega,
											angleX,angleY,
											newWheelSpeed);

	// send new speed to motors
	engine.setWheelSpeed(newWheelSpeed);
}

void BallDrive::getSpeed(float angleX, float angleY, float &speedX,float &speedY, float &omega) {
	if (lastCall_ms > 0 ) {
		uint32_t now = millis();
		float dT = ((float)(now - lastCall_ms))/1000.0;
		lastCall_ms = now;

		// fetch motor encoder values to compute real wheel position
		float angleChange[3] = {0,0,0};
		engine.getWheelAngleChange(angleChange);

		float currentWheelSpeed[3] = {0,0,0};
		currentWheelSpeed[0] = angleChange[0] * TWO_PI / dT;	// compute wheel speed out of delta-angle
		currentWheelSpeed[1] = angleChange[1] * TWO_PI / dT;
		currentWheelSpeed[2] = angleChange[2] * TWO_PI / dT;

		// apply inverse kinematics to get { speed (x,y), omega } out of wheel speed
		kinematics.computeActualSpeed(  currentWheelSpeed,
										angleX, angleY,
										speedX, speedY, omega);
	}
}

void BallDrive::loop() {
	engine.loop();
}

void BallDrive::menuLoop(char ch) {
	bool cmd = true;
	switch (ch) {
	case '0':
		menuSpeedX = 0;
		menuSpeedY = 0;
		menuOmega = 0;
		menuAngleX = 0;
		menuAngleY = 0;
		setSpeed(menuSpeedX, menuSpeedY,  menuOmega,  menuAngleX,  menuAngleY);
		cmd = true;
		break;
	case 'q':
		menuSpeedX += 0.1;
		setSpeed(menuSpeedX, menuSpeedY,  menuOmega,  menuAngleX,  menuAngleY);
		cmd = true;
		break;
	case 'a':
		menuSpeedX -= 0.1;
		setSpeed(menuSpeedX, menuSpeedY,  menuOmega,  menuAngleX,  menuAngleY);
		cmd = true;
		break;
	case 'w':
		menuSpeedY += 0.1;
		setSpeed(menuSpeedX, menuSpeedY,  menuOmega,  menuAngleX,  menuAngleY);
		cmd = true;
		break;
	case 's':
		menuSpeedY -= 0.1;
		setSpeed(menuSpeedX, menuSpeedY,  menuOmega,  menuAngleX,  menuAngleY);
		cmd = true;
		break;
	case 'y':
		menuOmega += 0.1;
		setSpeed(menuSpeedX, menuSpeedY,  menuOmega,  menuAngleX,  menuAngleY);
		cmd = true;
		break;
	case 'x':
		menuOmega += 0.1;
		setSpeed(menuSpeedX, menuSpeedY,  menuOmega,  menuAngleX,  menuAngleY);
		cmd = true;
		break;
	case 'o':
		menuAngleX += radians(1);
		setSpeed(menuSpeedX, menuSpeedY,  menuOmega,  menuAngleX,  menuAngleY);
		cmd = true;
		break;
	case 'l':
		menuAngleX -= radians(1);
		setSpeed(menuSpeedX, menuSpeedY,  menuOmega,  menuAngleX,  menuAngleY);
		cmd = true;
		break;
	case 'i':
		menuAngleY += radians(1);
		setSpeed(menuSpeedX, menuSpeedY,  menuOmega,  menuAngleX,  menuAngleY);
		cmd = true;
		break;
	case 'k':
		menuAngleY -= radians(1);
		setSpeed(menuSpeedX, menuSpeedY,  menuOmega,  menuAngleX,  menuAngleY);
		cmd = true;
		break;
	case 'e':
		enable(!isEnabled());
		break;
	case 'b':
		engine.pushMenu();
		break;
	case 'h':
		printHelp();
		break;
	case 't':
		kinematics.testInverseKinematics();
		kinematics.testPerformanceKinematics();
		kinematics.testKinematics();
		break;
	default:
		cmd = false;
		break;
	}
	if (cmd) {
		if (isEnabled())
			logger->print("enabled.");
		else
			logger->print("disabled.");
		logger->print(" t=");
		logger->print(engine.getAvrLoopTime()*1000000.0);
		logger->print("us");
		logger->print(" speed=(");
		logger->print(menuSpeedX);
		logger->print(",");
		logger->print(menuSpeedY);
		logger->print(") angle=(");
		logger->print(degrees(menuAngleX));
		logger->print(",");
		logger->print(degrees(menuAngleX));
		logger->print(")");

		command->println(" >");


	}
}



