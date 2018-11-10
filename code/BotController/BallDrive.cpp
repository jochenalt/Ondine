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
	command->println("p - power");
	command->println("e - enable");
	command->println("q/a - modify x speed");
	command->println("w/s - modify y speed");
	command->println("y/x - modify omega speed");
	command->println("o/l - modify tilt x");
	command->println("i/k - modify tilt y");
	command->println("t   - test kinmatics");

	command->println("b - single wheel control");

	command->println("0 - nullify speed");

	command->println("ESC");
}

void BallDrive::getSetAngle(float &angleX, float &angleY) {
	angleX = lastSetAngleX;
	angleY = lastSetAngleY;
}

void BallDrive::setSpeed(float speedX,float speedY, float omega,
		 	 	 	 	 float angleX, float angleY) {

	// apply kinematics to compute wheel speed out of x,y, omega
	float  newWheelSpeed[3] = { 0, 0, 0};
	kinematics.computeWheelSpeed(speedX, speedY, omega,
											angleX,angleY,
											newWheelSpeed);

	/*
	logger->print("kinematics:(");
	logger->print(speedX);
	logger->print(",");
	logger->print(speedY);
	logger->print(",");
	logger->print(omega);
	logger->print(")->(");
	logger->print(newWheelSpeed[0]);
	logger->print(",");
	logger->print(newWheelSpeed[1]);
	logger->print(",");
	logger->print(newWheelSpeed[2]);
	logger->println(")");
	*/
	// send new speed to motors
	engine.setWheelSpeed(newWheelSpeed);
}

// compute the current speed since the last invocation.
// returns 0 when called first (assuming the we start without motion)
void BallDrive::getSpeed(const IMUSample &sample, BotMovement &current) {

	// this function required
	uint32_t now = millis();
	if ((now > lastCall_ms) && (lastCall_ms > 0))  {
		float dT = ((float)(now - lastCall_ms))/1000.0; // [s]
		lastCall_ms = now;

		// fetch motor encoder values to compute real wheel position
		float angleChange[3] = {0,0,0};
		engine.getWheelAngleChange(angleChange);

		float currentWheelSpeed[3];
		currentWheelSpeed[0] = angleChange[0] * TWO_PI / dT;	// compute wheel speed out of delta-angle
		currentWheelSpeed[1] = angleChange[1] * TWO_PI / dT;
		currentWheelSpeed[2] = angleChange[2] * TWO_PI / dT;

		// apply inverse kinematics to get { speed (x,y), omega } out of wheel speed
		kinematics.computeActualSpeed(  currentWheelSpeed,
										sample.plane[X].angle, sample.plane[Y].angle,
										current.x.speed, current.y.speed, current.omega);

		current.x.pos += dT*current.x.speed;
		current.y.pos += dT*current.y.speed;
		current.x.accel = (current.x.speed - lastSpeedX)/dT;
		current.y.accel = (current.y.speed - lastSpeedY)/dT;
		lastSpeedX = current.x.speed;
		lastSpeedY = current.y.speed;

	} else {
		current.x.speed = 0;
		current.y.speed = 0;
		current.x.pos = 0;
		current.y.pos = 0;
		current.x.accel = 0;
		current.y.accel = 0;
		current.omega = 0;
	}
}

void BallDrive::loop() {
	// drive the motors
	// due to use of brushless motors, this requires permanent invokation of loop
	engine.loop();
}

void BallDrive::menuLoop(char ch, bool continously) {
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
		menuSpeedY -= (0.1);
		setSpeed(menuSpeedX, menuSpeedY,  menuOmega,  menuAngleX,  menuAngleY);
		cmd = true;
		break;
	case 'y':
		menuOmega += radians(0.1);
		setSpeed(menuSpeedX, menuSpeedY,  menuOmega,  menuAngleX,  menuAngleY);
		cmd = true;
		break;
	case 'x':
		menuOmega -= radians(0.1);
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
	case 'p':
		if (isPowered()) {
			command->println("turning motor power off");
			powerRelay.power(false);
		} else {
			command->println("turning motor power on");
			powerRelay.power(true);
		}
		break;
	case 't':
		kinematics.testKinematics();
		kinematics.testInverseKinematics();
		kinematics.testPerformanceKinematics();
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
		logger->print(degrees(menuAngleY));
		logger->print(")");

		command->println(" >");


	}
}



